/*
 * Copyright (c) 2025 Manus AI
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zmk_behavior_z_axis_morph

#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <zmk/behavior.h>
#include <zmk/event_manager.h>
#include <zmk/keymap.h>
#include <zmk/drivers/sensor/tlx493d_state.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct behavior_z_axis_morph_config {
    struct zmk_behavior_binding normal_binding;
    struct zmk_behavior_binding pressed_binding;
};

struct behavior_z_axis_morph_data {
    bool z_axis_pressed;
};

static int behavior_z_axis_morph_init(const struct device *dev) {
    return 0;
}

static bool get_z_axis_state(void) {
    return tlx493d_get_z_axis_pressed();
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding_device(binding);
    const struct behavior_z_axis_morph_config *config = dev->config;
    
    bool z_pressed = get_z_axis_state();
    
    if (z_pressed) {
        LOG_DBG("Z-axis pressed state: invoking pressed binding");
        return zmk_behavior_invoke_binding(&config->pressed_binding, event, true);
    } else {
        LOG_DBG("Z-axis normal state: invoking normal binding");
        return zmk_behavior_invoke_binding(&config->normal_binding, event, true);
    }
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding_device(binding);
    const struct behavior_z_axis_morph_config *config = dev->config;
    
    bool z_pressed = get_z_axis_state();
    
    if (z_pressed) {
        LOG_DBG("Z-axis pressed state: releasing pressed binding");
        return zmk_behavior_invoke_binding(&config->pressed_binding, event, false);
    } else {
        LOG_DBG("Z-axis normal state: releasing normal binding");
        return zmk_behavior_invoke_binding(&config->normal_binding, event, false);
    }
}

static const struct behavior_driver_api behavior_z_axis_morph_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

#define ZAX_INST(n)                                                                             \
    static struct behavior_z_axis_morph_data behavior_z_axis_morph_data_##n = {};              \
    static struct behavior_z_axis_morph_config behavior_z_axis_morph_config_##n = {            \
        .normal_binding = ZMK_KEYMAP_EXTRACT_BINDING(0, DT_DRV_INST(n)),                      \
        .pressed_binding = ZMK_KEYMAP_EXTRACT_BINDING(1, DT_DRV_INST(n)),                     \
    };                                                                                          \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_z_axis_morph_init, NULL,                              \
                            &behavior_z_axis_morph_data_##n,                                   \
                            &behavior_z_axis_morph_config_##n, POST_KERNEL,                    \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                               \
                            &behavior_z_axis_morph_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ZAX_INST)

#endif
