# Create an INTERFACE library for our C module.
add_library(usermod_ili9341 INTERFACE)

# Add our source files to the lib
target_sources(usermod_ili9341 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/machine_ili9341.c
)

# Add the current directory as an include directory.
target_include_directories(usermod_ili9341 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${IDF_PATH}/components/spiffs/include
    ${IDF_PATH}/components/m5core2/ili9341/include
)

target_compile_definitions(usermod_ili9341 INTERFACE)

# Link our INTERFACE library to the usermod target.
target_link_libraries(usermod INTERFACE usermod_ili9341)