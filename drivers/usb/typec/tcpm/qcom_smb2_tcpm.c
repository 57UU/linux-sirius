// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm PM660 (SMB2) USB Type-C port controller driver
 *
 * The PM660 is paired with an SMB2 switch-mode charger.  Its Type-C block
 * lives in the charger's USBIN register window and, unlike the PM8150B /
 * PMI632 blocks driven by qcom_pmic_typec_port.c, is not TCPCI shaped: the
 * CC state machine (DRP toggling, Rp/Rd detection, orientation and VCONN)
 * runs in hardware and only the decoded result is visible through a handful
 * of status registers.
 *
 * This driver exposes that hardware through struct tcpc_dev so that the
 * generic TCPM state machine can drive the port (attach/detach, role,
 * orientation, VCONN and PD).  The Power Delivery PHY lives in a separate
 * SPMI peripheral (0x1700) and is handled by the existing
 * qcom_pmic_typec_pdphy.c code, which matches the PM660 register layout.
 */

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/usb/typec.h>
#include <linux/usb/tcpm.h>

#include "qcom/qcom_pmic_typec.h"
#include "qcom/qcom_pmic_typec_pdphy.h"

/*
 * Registers of the charger USBIN block, relative to the node's first reg
 * (the block base, 0x1300 on the PM660).
 */
#define USBIN_TYPE_C_STATUS_1		0x0b
#define USBIN_TYPE_C_STATUS_2		0x0c
#define USBIN_TYPE_C_STATUS_4		0x0e
#define USBIN_INT_RT_STS		0x10
#define USBIN_TYPE_C_CFG		0x58
#define USBIN_TYPE_C_CFG_2		0x59
#define USBIN_TYPE_C_CFG_3		0x5a
#define USBIN_TYPE_C_INTRPT_ENB		0x67
#define USBIN_TYPE_C_SW_CTRL		0x68
#define USBIN_CURRENT_LIMIT_CFG		0x70

/* TYPE_C_STATUS_1: sink (UFP) view, the partner's Rp is decoded here */
#define UFP_TYPEC_MASK			GENMASK(7, 5)
#define UFP_TYPEC_RDSTD			BIT(7)
#define UFP_TYPEC_RD1P5			BIT(6)
#define UFP_TYPEC_RD3P0			BIT(5)
#define UFP_TYPEC_OPEN_OPEN		BIT(0)

/* TYPE_C_STATUS_2: source (DFP) view, the partner's Rd is decoded here */
#define DFP_TYPEC_MASK			GENMASK(3, 0)
#define DFP_RD_OPEN			BIT(3)
#define DFP_RD_RA_VCONN			BIT(2)
#define DFP_RD_RD			BIT(1)
#define DFP_RA_RA			BIT(0)
#define DFP_RA_OPEN			BIT(7)

/* TYPE_C_STATUS_4 */
#define UFP_DFP_MODE_STATUS		BIT(7)
#define TYPEC_VBUS_STATUS		BIT(6)
#define CC_ORIENTATION_CC2		BIT(1)
#define CC_ATTACHED			BIT(0)

/* TYPE_C_CFG */
#define FACTORY_MODE_DETECTION_EN	BIT(5)
#define VCONN_OC_CFG			BIT(1)

/* TYPE_C_CFG_2 */
#define DFP_CC_1P4V_OR_1P6V		BIT(6)
#define VCONN_SOFTSTART_MASK		GENMASK(5, 4)

/* TYPE_C_CFG_3 */
#define TYPEC_LEGACY_CABLE_INT_EN	BIT(6)
#define TYPEC_NONCOMPLIANT_LEGACY_INT_EN	BIT(5)
#define EN_TRYSINK_MODE			BIT(2)

/* TYPE_C_INTRPT_ENB */
#define TYPEC_CCSTATE_CHANGE_INT_EN	BIT(2)

/* TYPE_C_INTRPT_ENB_SOFTWARE_CTRL */
#define VCONN_EN_ORIENTATION_CC2	BIT(6)
#define VCONN_EN_SRC			BIT(4)
#define VCONN_EN_VALUE			BIT(3)
#define TYPEC_POWER_ROLE_CMD_MASK	GENMASK(2, 0)
#define UFP_EN_CMD			BIT(2)
#define DFP_EN_CMD			BIT(1)
#define TYPEC_DISABLE_CMD		BIT(0)

/*
 * VBUS boost for the source role.  The charger's control block starts at
 * 0x1000, so this register is addressed absolutely on the PMIC.
 */
#define SMB2_DCDC_CMD_OTG		0x1140
#define SMB2_OTG_EN			BIT(0)

/* Sampling attempts used to pick up a cable that is attached at probe time */
#define SMB2_KICK_TRIES			6
#define SMB2_KICK_MS			400
/* keep DCDC_CMD_OTG set while the charger's own state machine settles */
#define SMB2_VBUS_HOLD_TRIES		20
#define SMB2_VBUS_HOLD_MS		500

/* Input current limit: 25 mA per LSB, same unit the charger driver uses */
#define SMB2_ICL_STEP_UA		25000
#define SMB2_ICL_MIN_RAW		4	/* 100 mA */
#define SMB2_ICL_MAX_MA			3200

/*
 * The charger's Type-C FSM drops CC_ATTACHED for a few hundred milliseconds
 * every time the port latches in Attached.SRC and the 5 V boost comes up (and
 * again around partner enumeration).  Attach is therefore held for this long
 * before a dropout is reported as a disconnect.
 */
#define SMB2_ATTACH_HOLD_MS		800

struct smb2_typec {
	struct pmic_typec	tcpm;
	struct device		*dev;
	struct regmap		*regmap;
	unsigned int		base;
	unsigned int		kick_cnt;
	unsigned int		vbus_hold_cnt;
	int			irq;
	struct tcpm_port	*tcpm_port;
	struct delayed_work	probe_kick;
	struct delayed_work	detach_check;
	struct delayed_work	vbus_kick;
	enum typec_cc_status	last_cc1;
	enum typec_cc_status	last_cc2;
	unsigned long		hold_until;
	bool			attached;
	bool			vbus_on;
	bool			cc2_active;
};

static struct smb2_typec *tcpc_to_smb2(struct tcpc_dev *tcpc)
{
	return dev_get_drvdata(tcpc_to_tcpm(tcpc)->dev);
}

static int smb2_typec_read(struct smb2_typec *chip, unsigned int reg,
			   unsigned int *val)
{
	return regmap_read(chip->regmap, chip->base + reg, val);
}

static int smb2_typec_init(struct tcpc_dev *tcpc)
{
	struct smb2_typec *chip = tcpc_to_smb2(tcpc);
	int ret;

	/*
	 * Only the CC state change is interesting; the legacy cable and
	 * TRY.SINK interrupts stay masked, they are only needed by the
	 * non-compliant accessory handling the vendor stack implements.
	 */
	ret = regmap_write(chip->regmap, chip->base + USBIN_TYPE_C_INTRPT_ENB,
			   TYPEC_CCSTATE_CHANGE_INT_EN);
	if (ret)
		return ret;

	/* Dual role: let the hardware FSM run DRP toggling. */
	ret = regmap_update_bits(chip->regmap, chip->base + USBIN_TYPE_C_SW_CTRL,
				 TYPEC_POWER_ROLE_CMD_MASK, 0);
	if (ret)
		return ret;

	/* Factory (micro-USB) mode detection and VCONN OC workaround off. */
	ret = regmap_update_bits(chip->regmap, chip->base + USBIN_TYPE_C_CFG,
				 FACTORY_MODE_DETECTION_EN | VCONN_OC_CFG, 0);
	if (ret)
		return ret;

	/* Slowest VCONN soft start, 1.6 V CC threshold in source mode. */
	ret = regmap_update_bits(chip->regmap, chip->base + USBIN_TYPE_C_CFG_2,
				 VCONN_SOFTSTART_MASK, VCONN_SOFTSTART_MASK);
	if (ret)
		return ret;

	ret = regmap_update_bits(chip->regmap, chip->base + USBIN_TYPE_C_CFG_2,
				 DFP_CC_1P4V_OR_1P6V, DFP_CC_1P4V_OR_1P6V);
	if (ret)
		return ret;

	return regmap_update_bits(chip->regmap, chip->base + USBIN_TYPE_C_CFG_3,
				  EN_TRYSINK_MODE |
				  TYPEC_LEGACY_CABLE_INT_EN |
				  TYPEC_NONCOMPLIANT_LEGACY_INT_EN, 0);
}

static int smb2_typec_get_vbus(struct tcpc_dev *tcpc)
{
	struct smb2_typec *chip = tcpc_to_smb2(tcpc);
	unsigned int stat4;
	int ret;

	ret = smb2_typec_read(chip, USBIN_TYPE_C_STATUS_4, &stat4);
	if (ret)
		return ret;

	if (stat4 & TYPEC_VBUS_STATUS)
		return 1;

	/*
	 * While we source VBUS the boost being enabled is authoritative: the
	 * charger's Type-C FSM briefly clears the VBUS status bit every time
	 * it relatches, and TCPM would otherwise think the port lost power.
	 */
	return chip->vbus_on;
}

static int smb2_typec_get_cc(struct tcpc_dev *tcpc,
			     enum typec_cc_status *cc1,
			     enum typec_cc_status *cc2)
{
	struct smb2_typec *chip = tcpc_to_smb2(tcpc);
	enum typec_cc_status *active, *idle;
	unsigned int stat1, stat2, stat4, dfp;
	int ret;

	*cc1 = TYPEC_CC_OPEN;
	*cc2 = TYPEC_CC_OPEN;

	ret = smb2_typec_read(chip, USBIN_TYPE_C_STATUS_4, &stat4);
	if (ret)
		return ret;

	ret = smb2_typec_read(chip, USBIN_TYPE_C_STATUS_1, &stat1);
	if (ret)
		return ret;

	ret = smb2_typec_read(chip, USBIN_TYPE_C_STATUS_2, &stat2);
	if (ret)
		return ret;

	if (!(stat4 & CC_ATTACHED)) {
		if (chip->attached && time_before(jiffies, chip->hold_until)) {
			/* Transient dropout of a latched attach: keep the state
			 * and re-sample when the hold expires, so a real
			 * unplug is still picked up. */
			*cc1 = chip->last_cc1;
			*cc2 = chip->last_cc2;
			schedule_delayed_work(&chip->detach_check,
					      max_t(long, 1,
						    chip->hold_until - jiffies));
			return 0;
		}

		chip->attached = false;
		return 0;
	}

	chip->cc2_active = !!(stat4 & CC_ORIENTATION_CC2);
	active = chip->cc2_active ? cc2 : cc1;
	idle = chip->cc2_active ? cc1 : cc2;

	if (stat4 & UFP_DFP_MODE_STATUS) {
		/*
		 * Attached.SRC: we present Rp, the partner presents Rd (or
		 * nothing / Ra for an accessory).  TCPM wants the partner's
		 * termination, that is TYPEC_CC_RD here.
		 */
		dfp = stat2 & DFP_TYPEC_MASK;

		switch (dfp) {
		case DFP_RD_OPEN:
			*active = TYPEC_CC_RD;
			break;
		case DFP_RD_RD:
			/* Debug accessory: Rd on both CC lines */
			*cc1 = TYPEC_CC_RD;
			*cc2 = TYPEC_CC_RD;
			break;
		case DFP_RD_RA_VCONN:
			/* Powered cable: Rd on the active CC, Ra on the other */
			*active = TYPEC_CC_RD;
			*idle = TYPEC_CC_RA;
			break;
		case DFP_RA_RA:
			/* Audio adapter */
			*cc1 = TYPEC_CC_RA;
			*cc2 = TYPEC_CC_RA;
			break;
		default:
			if (stat2 & DFP_RA_OPEN)
				*active = TYPEC_CC_RA;
			break;
		}
	} else {
		/*
		 * Attached.SNK: we present Rd, the partner presents Rp.  TCPM
		 * wants the partner's Rp current level, not TYPEC_CC_RD.
		 */
		switch (stat1 & UFP_TYPEC_MASK) {
		case UFP_TYPEC_RDSTD:
			*active = TYPEC_CC_RP_DEF;
			break;
		case UFP_TYPEC_RD1P5:
			*active = TYPEC_CC_RP_1_5;
			break;
		case UFP_TYPEC_RD3P0:
			*active = TYPEC_CC_RP_3_0;
			break;
		default:
			/*
			 * Legacy micro-USB cable or a floating CC: treat it as
			 * a default-current source so charging still works.
			 */
			if (!(stat1 & UFP_TYPEC_OPEN_OPEN))
				*active = TYPEC_CC_RP_DEF;
			break;
		}
	}

	chip->last_cc1 = *cc1;
	chip->last_cc2 = *cc2;
	chip->attached = true;
	chip->hold_until = jiffies + msecs_to_jiffies(SMB2_ATTACH_HOLD_MS);

	return 0;
}

static int smb2_typec_set_cc(struct tcpc_dev *tcpc, enum typec_cc_status cc)
{
	struct smb2_typec *chip = tcpc_to_smb2(tcpc);
	unsigned int role, stat4;
	int ret;

	switch (cc) {
	case TYPEC_CC_OPEN:
		role = TYPEC_DISABLE_CMD;
		break;
	case TYPEC_CC_RD:
		role = UFP_EN_CMD;
		break;
	default:
		role = DFP_EN_CMD;
		break;
	}

	/*
	 * Re-writing the role command restarts the hardware CC FSM and drops
	 * CC_ATTACHED for a moment, which TCPM then reads as a disconnect.
	 * When the FSM is already latched in the requested role, leave it
	 * alone - the attach is already the answer we would ask for.
	 */
	ret = smb2_typec_read(chip, USBIN_TYPE_C_STATUS_4, &stat4);
	if (ret)
		return ret;

	if ((stat4 & CC_ATTACHED) && role != TYPEC_DISABLE_CMD) {
		bool src = !!(stat4 & UFP_DFP_MODE_STATUS);

		if ((role == DFP_EN_CMD && src) || (role == UFP_EN_CMD && !src))
			return 0;
	}

	return regmap_update_bits(chip->regmap, chip->base + USBIN_TYPE_C_SW_CTRL,
				  TYPEC_POWER_ROLE_CMD_MASK, role);
}

static int smb2_typec_start_toggling(struct tcpc_dev *tcpc,
				     enum typec_port_type port_type,
				     enum typec_cc_status cc)
{
	struct smb2_typec *chip = tcpc_to_smb2(tcpc);
	unsigned int role;

	switch (port_type) {
	case TYPEC_PORT_DRP:
		role = 0;
		break;
	case TYPEC_PORT_SRC:
		role = DFP_EN_CMD;
		break;
	case TYPEC_PORT_SNK:
	default:
		role = UFP_EN_CMD;
		break;
	}

	return regmap_update_bits(chip->regmap, chip->base + USBIN_TYPE_C_SW_CTRL,
				  TYPEC_POWER_ROLE_CMD_MASK, role);
}

static int smb2_typec_set_polarity(struct tcpc_dev *tcpc,
				   enum typec_cc_polarity polarity)
{
	struct smb2_typec *chip = tcpc_to_smb2(tcpc);

	/*
	 * The hardware detects the orientation itself; keep the cached value
	 * in sync so VCONN is routed to the inactive CC.
	 */
	chip->cc2_active = (polarity == TYPEC_POLARITY_CC2);

	return 0;
}

static int smb2_typec_set_vconn(struct tcpc_dev *tcpc, bool on)
{
	struct smb2_typec *chip = tcpc_to_smb2(tcpc);
	unsigned int mask, val = 0;

	if (on) {
		val = VCONN_EN_SRC | VCONN_EN_VALUE;
		if (!chip->cc2_active)
			val |= VCONN_EN_ORIENTATION_CC2;
	}

	mask = VCONN_EN_SRC | VCONN_EN_VALUE | VCONN_EN_ORIENTATION_CC2;

	return regmap_update_bits(chip->regmap, chip->base + USBIN_TYPE_C_SW_CTRL,
				  mask, val);
}

static int smb2_typec_set_current_limit(struct tcpc_dev *tcpc, u32 max_ma,
					u32 mv)
{
	struct smb2_typec *chip = tcpc_to_smb2(tcpc);
	unsigned int raw;

	/*
	 * TCPM works the contract out (Rp current or PD PDO) and tells us here;
	 * the charger's APSD guess is bypassed via qcom,typec-managed-icl in
	 * the board DT.
	 */
	if (!max_ma)
		return 0;

	if (max_ma > SMB2_ICL_MAX_MA)
		max_ma = SMB2_ICL_MAX_MA;

	raw = max_ma * 1000 / SMB2_ICL_STEP_UA;
	if (raw < SMB2_ICL_MIN_RAW)
		raw = SMB2_ICL_MIN_RAW;

	dev_dbg(chip->dev, "input current limit %u mA @ %u mV\n", max_ma, mv);

	return regmap_write(chip->regmap,
			    chip->base + USBIN_CURRENT_LIMIT_CFG, raw);
}

static int smb2_typec_set_vbus(struct tcpc_dev *tcpc, bool on, bool charge)
{
	struct smb2_typec *chip = tcpc_to_smb2(tcpc);
	unsigned int cur;
	int ret;

	/*
	 * Sinking: VBUS comes from the partner and the charger owns the input
	 * path, there is nothing to switch here.
	 */
	if (charge)
		return 0;

	/*
	 * Writing the boost enable again while it is already in the wanted
	 * state makes the charger's Type-C FSM drop CC_ATTACHED for a few
	 * hundred milliseconds (observed on PM660).  Only touch the register
	 * on an actual change.
	 */
	ret = regmap_read(chip->regmap, SMB2_DCDC_CMD_OTG, &cur);
	if (ret)
		return ret;

	if (!!(cur & SMB2_OTG_EN) == on)
		return 0;

	ret = regmap_update_bits(chip->regmap, SMB2_DCDC_CMD_OTG, SMB2_OTG_EN,
				 on ? SMB2_OTG_EN : 0);
	if (ret)
		return ret;

	chip->vbus_on = on;
	chip->vbus_hold_cnt = 0;
	if (chip->tcpm_port)
		schedule_delayed_work(&chip->vbus_kick, msecs_to_jiffies(50));

	return 0;
}

static void smb2_typec_probe_kick(struct work_struct *work)
{
	struct smb2_typec *chip = container_of(work, struct smb2_typec,
					       probe_kick.work);

	/*
	 * TCPM only samples CC state on change events, and the hardware FSM
	 * may already be sitting in an attached state when we start.  The
	 * first sample also moves TCPM into its toggling state, so a cable
	 * that was attached before probe needs a few follow-up samples.
	 */
	tcpm_cc_change(chip->tcpm_port);
	tcpm_vbus_change(chip->tcpm_port);

	if (++chip->kick_cnt < SMB2_KICK_TRIES)
		schedule_delayed_work(&chip->probe_kick,
				      msecs_to_jiffies(SMB2_KICK_MS));
}

static void smb2_typec_vbus_kick(struct work_struct *work)
{
	struct smb2_typec *chip = container_of(work, struct smb2_typec,
					       vbus_kick.work);
	unsigned int cur;
	int ret;

	/*
	 * TCPM only learns about VBUS edges through this callback, and the
	 * edge it cares about happens inside tcpm_src_attach(), right before
	 * the port would otherwise fall back to SRC_UNATTACHED.
	 */
	if (chip->vbus_hold_cnt == 0) {
		tcpm_vbus_change(chip->tcpm_port);
		tcpm_cc_change(chip->tcpm_port);
	}
	chip->vbus_hold_cnt++;

	/*
	 * The PMIC drops DCDC_CMD_OTG by itself a moment after the port
	 * attaches: the boost output shows up on USBIN, the charger treats
	 * it as a new input and its state machine re-initialises the DCDC
	 * block, clearing the enable.  TCPM only stays in SRC_ATTACHED if it
	 * sees VBUS come up within 480 ms, so a silently lost enable turns
	 * into an endless attach/detach loop (and a dead xHCI afterwards).
	 * Re-assert the bit for a few seconds until the charger settles.
	 */
	if (chip->vbus_on) {
		ret = regmap_read(chip->regmap, SMB2_DCDC_CMD_OTG, &cur);
		if (!ret && !(cur & SMB2_OTG_EN)) {
			regmap_update_bits(chip->regmap, SMB2_DCDC_CMD_OTG,
					   SMB2_OTG_EN, SMB2_OTG_EN);
			dev_dbg(chip->dev,
				"OTG boost was cleared, re-asserted\n");
		}

		if (chip->vbus_hold_cnt < SMB2_VBUS_HOLD_TRIES)
			schedule_delayed_work(&chip->vbus_kick,
					      msecs_to_jiffies(SMB2_VBUS_HOLD_MS));
	}
}

static void smb2_typec_detach_check(struct work_struct *work)
{
	struct smb2_typec *chip = container_of(work, struct smb2_typec,
					       detach_check.work);
	unsigned int stat4;

	/* Still attached?  Then the dropout was transient, nothing to do. */
	if (!smb2_typec_read(chip, USBIN_TYPE_C_STATUS_4, &stat4) &&
	    (stat4 & CC_ATTACHED))
		return;

	tcpm_cc_change(chip->tcpm_port);
	tcpm_vbus_change(chip->tcpm_port);
}

static irqreturn_t smb2_typec_irq(int irq, void *data)
{
	struct smb2_typec *chip = data;
	unsigned int rt_sts;

	/*
	 * Reading the real-time status latches clears the PMIC interrupt;
	 * TCPM re-reads the state through the TCPC callbacks below.
	 */
	smb2_typec_read(chip, USBIN_INT_RT_STS, &rt_sts);

	tcpm_cc_change(chip->tcpm_port);
	tcpm_vbus_change(chip->tcpm_port);

	return IRQ_HANDLED;
}

/*
 * The PM660 PD PHY (SPMI 0x1700) uses the same register map and the same
 * interrupt lines as the PM8150B block, except for "sig-tx": that interrupt
 * only produces a dev_err() in the shared driver and fires spuriously on this
 * PHY, so it is left unrequested.  The virq numbers are the PMIC_PDPHY_*_IRQ
 * indices used by qcom_pmic_typec_pdphy.c.
 */
static const struct pmic_typec_pdphy_resources smb2_pdphy_res = {
	.irq_params = {
		{ .virq = 1, .irq_name = "sig-rx" },
		{ .virq = 2, .irq_name = "msg-tx" },
		{ .virq = 3, .irq_name = "msg-rx" },
		{ .virq = 4, .irq_name = "msg-tx-failed" },
		{ .virq = 5, .irq_name = "msg-tx-discarded" },
		{ .virq = 6, .irq_name = "msg-rx-discarded" },
	},
	.nr_irqs = 6,
};

static int smb2_typec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct smb2_typec *chip;
	struct regmap *regmap;
	u32 base, pdphy_base;
	int ret, irq;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	platform_set_drvdata(pdev, chip);

	regmap = dev_get_regmap(dev->parent, NULL);
	if (!regmap) {
		dev_err(dev, "no parent regmap\n");
		return -ENODEV;
	}
	chip->regmap = regmap;

	ret = of_property_read_u32_index(dev->of_node, "reg", 0, &base);
	if (ret) {
		dev_err(dev, "missing Type-C block base\n");
		return ret;
	}
	chip->base = base;

	ret = of_property_read_u32_index(dev->of_node, "reg", 1, &pdphy_base);
	if (ret) {
		dev_err(dev, "missing PD PHY base\n");
		return ret;
	}

	chip->tcpm.dev = dev;

	chip->tcpm.tcpc.init = smb2_typec_init;
	chip->tcpm.tcpc.get_vbus = smb2_typec_get_vbus;
	chip->tcpm.tcpc.get_cc = smb2_typec_get_cc;
	chip->tcpm.tcpc.set_cc = smb2_typec_set_cc;
	chip->tcpm.tcpc.set_polarity = smb2_typec_set_polarity;
	chip->tcpm.tcpc.set_vconn = smb2_typec_set_vconn;
	chip->tcpm.tcpc.set_vbus = smb2_typec_set_vbus;
	chip->tcpm.tcpc.set_current_limit = smb2_typec_set_current_limit;
	chip->tcpm.tcpc.start_toggling = smb2_typec_start_toggling;

	chip->tcpm.tcpc.fwnode = device_get_named_child_node(dev, "connector");
	if (!chip->tcpm.tcpc.fwnode) {
		dev_err(dev, "missing connector node\n");
		return -EINVAL;
	}

	ret = qcom_pmic_typec_pdphy_probe(pdev, &chip->tcpm, &smb2_pdphy_res,
					  regmap, pdphy_base);
	if (ret) {
		dev_err(dev, "PD PHY probe failed: %d\n", ret);
		goto err_put_fwnode;
	}

	irq = platform_get_irq_byname(pdev, "type-c-change");
	if (irq < 0) {
		ret = irq;
		goto err_put_fwnode;
	}

	chip->tcpm_port = tcpm_register_port(dev, &chip->tcpm.tcpc);
	if (IS_ERR(chip->tcpm_port)) {
		ret = PTR_ERR(chip->tcpm_port);
		dev_err(dev, "failed to register TCPM port: %d\n", ret);
		goto err_put_fwnode;
	}

	chip->irq = irq;
	ret = devm_request_threaded_irq(dev, irq, NULL, smb2_typec_irq,
					IRQF_ONESHOT, dev_name(dev), chip);
	if (ret)
		goto err_unregister_tcpm;

	ret = chip->tcpm.pdphy_start(&chip->tcpm, chip->tcpm_port);
	if (ret) {
		dev_err(dev, "failed to start PD PHY: %d\n", ret);
		goto err_unregister_tcpm;
	}

	INIT_DELAYED_WORK(&chip->vbus_kick, smb2_typec_vbus_kick);
	INIT_DELAYED_WORK(&chip->detach_check, smb2_typec_detach_check);
	INIT_DELAYED_WORK(&chip->probe_kick, smb2_typec_probe_kick);
	schedule_delayed_work(&chip->probe_kick, msecs_to_jiffies(500));

	return 0;

err_unregister_tcpm:
	tcpm_unregister_port(chip->tcpm_port);
err_put_fwnode:
	fwnode_handle_put(chip->tcpm.tcpc.fwnode);

	return ret;
}

static void smb2_typec_remove(struct platform_device *pdev)
{
	struct smb2_typec *chip = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&chip->vbus_kick);
	cancel_delayed_work_sync(&chip->detach_check);
	cancel_delayed_work_sync(&chip->probe_kick);
	chip->tcpm.pdphy_stop(&chip->tcpm);
	tcpm_unregister_port(chip->tcpm_port);
	fwnode_handle_put(chip->tcpm.tcpc.fwnode);
}

static const struct of_device_id smb2_typec_table[] = {
	{ .compatible = "qcom,pm660-typec" },
	{ }
};
MODULE_DEVICE_TABLE(of, smb2_typec_table);

static struct platform_driver smb2_typec_driver = {
	.driver = {
		.name = "qcom,pm660-typec",
		.of_match_table = smb2_typec_table,
	},
	.probe = smb2_typec_probe,
	.remove = smb2_typec_remove,
};
module_platform_driver(smb2_typec_driver);

MODULE_DESCRIPTION("Qualcomm PM660/SMB2 USB Type-C port controller driver");
MODULE_LICENSE("GPL");
