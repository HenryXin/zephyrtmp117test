zephyr_library()
zephyr_library_sources(custom_tmp117.c)

# Ensure offsets are generated before compiling the driver
if(TARGET offsets)
  zephyr_library_add_dependencies(offsets)
endif()

# Export the library target name so it can be linked from the main CMakeLists.txt
# ZEPHYR_CURRENT_LIBRARY is set by zephyr_library()
if(DEFINED ZEPHYR_CURRENT_LIBRARY)
    set(CUSTOM_TMP117_LIB_TARGET ${ZEPHYR_CURRENT_LIBRARY} PARENT_SCOPE)
    message(STATUS "TMP117 library target: ${ZEPHYR_CURRENT_LIBRARY}")
else()
    message(WARNING "ZEPHYR_CURRENT_LIBRARY not defined for TMP117 driver")
endif()
