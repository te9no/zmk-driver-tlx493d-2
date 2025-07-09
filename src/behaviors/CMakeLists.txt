zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ZMK_BEHAVIOR_Z_AXIS_MORPH behavior_z_axis_morph.c)
zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../include)

