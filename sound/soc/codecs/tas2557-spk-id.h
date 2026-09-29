/* Minimal spk-id stub for mainline: speaker variant is a module parameter
 * in tas2557.c (this phone is AAC). The downstream 3-state GPIO helper is
 * not yet ported. */
#ifndef __TAS2557_SPK_ID_H_
#define __TAS2557_SPK_ID_H_
#define PIN_PULL_DOWN		0
#define PIN_PULL_UP		1
#define PIN_FLOAT		2
#define VENDOR_ID_NONE		0
#define VENDOR_ID_AAC		1
#define VENDOR_ID_SSI		2
#define VENDOR_ID_GOER		3
#define VENDOR_ID_UNKNOWN	4
struct device_node;
extern int spk_id_get_pin_3state(struct device_node *np);
#endif
