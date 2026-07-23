cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME clock-radio)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# ST USB Device middleware (from the Cube firmware repo)
set(USB_MW $ENV{HOME}/STM32Cube/Repository/STM32Cube_FW_G0_V1.6.3/Middlewares/ST/STM32_USB_Device_Library)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    App/Src/oled.c
    App/Src/ds3231.c
    App/Src/rda5807.c
    App/Src/input.c
    App/Src/audio.c
    App/Src/ui.c
    App/Src/settings.c
    App/Src/console.c
    App/usb/usbd_conf.c
    App/usb/usbd_desc.c
    App/usb/usbd_cdc_if.c
    App/usb/usb_device.c
    ${USB_MW}/Core/Src/usbd_core.c
    ${USB_MW}/Core/Src/usbd_ctlreq.c
    ${USB_MW}/Core/Src/usbd_ioreq.c
    ${USB_MW}/Class/CDC/Src/usbd_cdc.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    App/Inc
    App/usb
    ${USB_MW}/Core/Inc
    ${USB_MW}/Class/CDC/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Keep clock-radio.bin in sync with the ELF (for dfu-util flashing)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.bin"
)
