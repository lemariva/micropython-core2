# Create an INTERFACE library for our C module.
add_library(usermod_ft6336u INTERFACE)

# Add our source files to the lib
target_sources(usermod_ft6336u INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/machine_ft6336u.c
)

# Add the current directory as an include directory.
target_include_directories(usermod_ft6336u INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${IDF_PATH}/components/m5core2/i2cdev/include
    ${IDF_PATH}/components/m5core2/ft6336u/include
)

target_compile_definitions(usermod_ft6336u INTERFACE)

# Link our INTERFACE library to the usermod target.
target_link_libraries(usermod INTERFACE usermod_ft6336u)