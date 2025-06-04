# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_INPUT_TLX493D INPUT_TLX493D.c)

target_sources_ifdef(CONFIG_ZMK_INPUT_TLX493D_IDLE_SLEEPER app PRIVATE zmk_TLX493D_idle_sleeper.c)
