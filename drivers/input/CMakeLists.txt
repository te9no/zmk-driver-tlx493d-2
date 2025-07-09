# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_INPUT_TLX493D input_tlx493d.c)
zephyr_library_sources_ifdef(CONFIG_INPUT_TLX493D ${CMAKE_CURRENT_SOURCE_DIR}/../../src/tlx493d_state.c)
zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../include)
zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../lib)
