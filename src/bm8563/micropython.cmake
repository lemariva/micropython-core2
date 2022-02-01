# Create an INTERFACE library for our C module.
add_library(usermod_bm8563 INTERFACE)

# Add our source files to the lib
target_sources(usermod_bm8563 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/machine_bm8563.c
)

# Add the current directory as an include directory.
target_include_directories(usermod_bm8563 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${IDF_PATH}/components/m5core2/i2cdev/include
    ${IDF_PATH}/components/m5core2/bm8563/include
)

target_compile_definitions(usermod_bm8563 INTERFACE)

# Link our INTERFACE library to the usermod target.
target_link_libraries(usermod INTERFACE usermod_bm8563)