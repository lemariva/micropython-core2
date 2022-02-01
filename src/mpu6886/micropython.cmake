# Create an INTERFACE library for our C module.
add_library(usermod_mpu6886 INTERFACE)

# Add our source files to the lib
target_sources(usermod_mpu6886 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/machine_mpu6886.c
)

# Add the current directory as an include directory.
target_include_directories(usermod_mpu6886 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${IDF_PATH}/components/m5core2/i2cdev/include
    ${IDF_PATH}/components/m5core2/mpu6886/include
)

target_compile_definitions(usermod_mpu6886 INTERFACE)

# Link our INTERFACE library to the usermod target.
target_link_libraries(usermod INTERFACE usermod_mpu6886)