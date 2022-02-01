# Create an INTERFACE library for our C module.
add_library(usermod_axp192 INTERFACE)

# Add our source files to the lib
target_sources(usermod_axp192 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/machine_axp192.c
)

# Add the current directory as an include directory.
target_include_directories(usermod_axp192 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${IDF_PATH}/components/m5core2/i2cdev/include
    ${IDF_PATH}/components/m5core2/axp192/include
    
)

target_compile_definitions(usermod_axp192 INTERFACE)

# Link our INTERFACE library to the usermod target.
target_link_libraries(usermod INTERFACE usermod_axp192)