cmake_minimum_required(VERSION 3.21)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(custom_dirver_module)

# Include custom driver module
add_subdirectory(custom_dirver_module)

target_sources(app PRIVATE src/main.c)

target_include_directories(app PRIVATE include)

# Link the custom TMP117 driver library
# Try using the exported variable first, then fallback to direct target name
if(CUSTOM_TMP117_LIB_TARGET)
    message(STATUS "Linking TMP117 library using exported target: ${CUSTOM_TMP117_LIB_TARGET}")
    target_link_libraries(app PRIVATE ${CUSTOM_TMP117_LIB_TARGET})
else()
    # Fallback: construct the target name based on the source path pattern
    # The target name is: ..__tmp__<project_dir>__custom_dirver_module__drivers__sensor__custom_tmp117
    get_filename_component(_project_dir ${CMAKE_SOURCE_DIR} NAME)
    set(_tmp117_target "..__tmp__${_project_dir}__custom_dirver_module__drivers__sensor__custom_tmp117")
    message(STATUS "Linking TMP117 library using constructed target: ${_tmp117_target}")
    if(TARGET ${_tmp117_target})
        target_link_libraries(app PRIVATE ${_tmp117_target})
    else()
        # Last resort: link the library file directly
        set(_tmp117_lib_file "${CMAKE_CURRENT_BINARY_DIR}/custom_dirver_module/drivers/sensor/custom_tmp117/lib${_tmp117_target}.a")
        message(STATUS "Linking TMP117 library file directly: ${_tmp117_lib_file}")
        target_link_libraries(app PRIVATE ${_tmp117_lib_file})
    endif()
endif()
