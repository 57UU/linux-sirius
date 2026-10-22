/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Mainline stub for the downstream <linux/msm_drm_notify.h> API.
 *
 * The FTS521 vendor driver uses the MSM DRM blank notifier to sleep/wake
 * the touch controller on display on/off. That notifier does not exist in
 * mainline. For the sirius port, suspend/resume is handled via the
 * driver's own dev_pm_ops (fts_dev_pm_ops), so these register/unregister
 * helpers are intentionally no-ops. Event callback code using
 * MSM_DRM_* constants is kept for reference but never invoked.
 */
#ifndef __FTS_DRM_NOTIFY_H
#define __FTS_DRM_NOTIFY_H

#include <linux/notifier.h>

enum {
	MSM_DRM_BLANK_UNBLANK = 0,
	MSM_DRM_BLANK_POWERDOWN = 1,
	MSM_DRM_BLANK_LP1 = 2,
	MSM_DRM_BLANK_LP2 = 3,
};

enum {
	MSM_DRM_EARLY_EVENT_BLANK = 0,
	MSM_DRM_EVENT_BLANK = 1,
};

struct msm_drm_notifier {
	void *data;
	int id;
};

static inline int msm_drm_register_client(struct notifier_block *nb)
{
	return 0;
}

static inline int msm_drm_unregister_client(struct notifier_block *nb)
{
	return 0;
}

#endif
