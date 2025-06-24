/*
 * Copyright (c) 2025 Manus AI
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zmk/drivers/sensor/tlx493d_state.h>
#include <zephyr/kernel.h>

static bool z_axis_pressed = false;
static struct k_mutex z_axis_mutex;

bool tlx493d_get_z_axis_pressed(void) {
    bool pressed;
    k_mutex_lock(&z_axis_mutex, K_FOREVER);
    pressed = z_axis_pressed;
    k_mutex_unlock(&z_axis_mutex);
    return pressed;
}

void tlx493d_set_z_axis_pressed(bool pressed) {
    k_mutex_lock(&z_axis_mutex, K_FOREVER);
    z_axis_pressed = pressed;
    k_mutex_unlock(&z_axis_mutex);
}

static int tlx493d_state_init(const struct device *unused) {
    ARG_UNUSED(unused);
    k_mutex_init(&z_axis_mutex);
    return 0;
}

SYS_INIT(tlx493d_state_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);