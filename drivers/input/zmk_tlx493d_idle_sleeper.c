#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include "input_tlx493d.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(zmk_tlx493d_sleep, CONFIG_INPUT_LOG_LEVEL);

#define GET_TLX493D(node_id) { .dev = DEVICE_DT_GET(node_id) },

struct tlx493d_device {
    const struct device *dev;
};

static struct tlx493d_device tlx493d_devs[] = {
    DT_FOREACH_STATUS_OKAY(infineon_tlx493d, GET_TLX493D)
};

static int on_activity_state(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);

    if (!state_ev) {
        LOG_WRN("No activity state change event");
        return -EINVAL;
    }

    bool sleep = (state_ev->state != ZMK_ACTIVITY_ACTIVE);
    
    for (size_t i = 0; i < ARRAY_SIZE(tlx493d_devs); i++) {
        const struct device *dev = tlx493d_devs[i].dev;
        if (!device_is_ready(dev)) {
            LOG_ERR("TLX493D device %d not ready", i);
            continue;
        }
        
        int ret = tlx493d_set_sleep(dev, sleep);
        if (ret) {
            LOG_ERR("Failed to set sleep state for device %d: %d", i, ret);
        }
    }

    return 0;
}

ZMK_LISTENER(zmk_tlx493d_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_tlx493d_idle_sleeper, zmk_activity_state_changed);
