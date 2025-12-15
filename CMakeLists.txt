cmake_minimum_required(VERSION 3.21)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(custom_dirver_module)

# Include custom driver module
add_subdirectory(custom_dirver_module)

target_sources(app PRIVATE src/main.c)

target_include_directories(app PRIVATE include)

# Link the custom TMP117 driver library
target_link_libraries(app PRIVATE 
    ..__tmp__tmp117driverTest__custom_dirver_module__drivers__sensor__custom_tmp117
)
