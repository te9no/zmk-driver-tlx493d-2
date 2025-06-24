zephyr_library()

zephyr_library_sources_ifdef(CONFIG_INPUT_TLX493D tlx493d_state.c)
zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

add_subdirectory(behaviors)