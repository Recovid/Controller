add_subdirectory(HAL)
add_subdirectory(FreeRTOS)

add_library(lowlevel OBJECT
    stm32f3xx_hal_msp_tick.c
    stm32f3xx_hal_msp.c
    stm32f3xx_it.c

    hardware_simulation.c
    hardware_serial.c
    leds.c
)

target_include_directories(lowlevel PUBLIC ${LOWLEVEL_INC_DIR})
target_link_libraries(lowlevel PUBLIC hal freertos)
