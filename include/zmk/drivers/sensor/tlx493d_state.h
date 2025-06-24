/*
 * Copyright (c) 2025 Manus AI
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>

/**
 * @brief Get the current Z-axis state from TLX493D sensor
 * @return true if Z-axis is pressed, false if normal
 */
bool tlx493d_get_z_axis_pressed(void);

/**
 * @brief Set the Z-axis state (called by TLX493D driver)
 * @param pressed true if Z-axis is pressed, false if normal
 */
void tlx493d_set_z_axis_pressed(bool pressed);