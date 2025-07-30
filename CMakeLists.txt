# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(motion_datalogger C CXX ASM)

pico_sdk_init()

include_directories(${CMAKE_SOURCE_DIR}/lib)

add_executable(${PROJECT_NAME} 
    motion_datalogger.c 
    lib/ssd1306.c 
    lib/hw_config.c
    lib/imu.c
)


pico_set_program_name(${PROJECT_NAME} "motion_datalogger")
pico_set_program_version(${PROJECT_NAME} "0.1")

pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_usb(${PROJECT_NAME} 1)

add_subdirectory(lib/FatFs_SPI)    

target_link_libraries(${PROJECT_NAME}
        pico_stdlib
        FatFs_SPI              # Biblioteca FatFs para SPI
        hardware_pwm           # PWM do RP2040
        hardware_clocks        # Clock do RP2040
        hardware_i2c           # I2C do RP2040
        hardware_pio
        hardware_adc           # ADC do RP2040
        hardware_gpio          # GPIO do RP2040
        hardware_spi           # SPI do RP2040
)

target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(${PROJECT_NAME} 
        
        )

pico_add_extra_outputs(${PROJECT_NAME})