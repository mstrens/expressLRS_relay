
cmake_minimum_required(VERSION 3.13)

add_compile_definitions(PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(my_project LANGUAGES C CXX)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(Frsky_Elrs_Relay)

pico_generate_pio_header(Frsky_Elrs_Relay  ${CMAKE_CURRENT_LIST_DIR}/src/sport_uart_tx_rx.pio )
pico_generate_pio_header(Frsky_Elrs_Relay  ${CMAKE_CURRENT_LIST_DIR}/src/crsf_uart_tx_rx.pio )
pico_generate_pio_header(Frsky_Elrs_Relay  ${CMAKE_CURRENT_LIST_DIR}/src/ppm.pio )


file(GLOB Frsky_Elrs_Relay_SRC CONFIGURE_DEPENDS "src/*.h" "src/*.cpp")

target_sources(Frsky_Elrs_Relay PRIVATE ${Frsky_Elrs_Relay_SRC}  )

target_link_libraries(Frsky_Elrs_Relay PRIVATE
        pico_stdlib
        pico_multicore
        hardware_pio
        hardware_i2c
        hardware_watchdog
        hardware_timer
        hardware_dma
        hardware_irq
        hardware_adc
        hardware_pwm
        )

#enable printf on usb
pico_enable_stdio_usb(Frsky_Elrs_Relay 1)
pico_enable_stdio_uart(Frsky_Elrs_Relay 0)


# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(Frsky_Elrs_Relay)


add_custom_command(TARGET Frsky_Elrs_Relay POST_BUILD
  COMMAND ${CMAKE_COMMAND}
  ARGS -E copy Frsky_Elrs_Relay.uf2 ${CMAKE_SOURCE_DIR}   
)

