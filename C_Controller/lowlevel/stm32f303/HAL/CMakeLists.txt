add_library(hal STATIC
    stm32f3xx_nucleo.c
    syscalls.c
    sysmem.c
    system_stm32f3xx.c
    Drivers/startup_stm32f303retx.s
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_cortex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_dma.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_exti.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash_ex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_gpio.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c_ex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr_ex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc_ex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim_ex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart_ex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_adc.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_comp.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_crc.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_dac.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_dma.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_exti.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_fmc.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_gpio.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_hrtim.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_i2c.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_opamp.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_pwr.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_rcc.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_rtc.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_spi.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_tim.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_usart.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_usb.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_utils.c
)

target_include_directories(hal PUBLIC
    .
    Core/Inc
    Drivers/STM32F3xx_HAL_Driver/Inc
    Drivers/STM32F3xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F3xx/Include
    Drivers/CMSIS/Include
)

target_compile_options(hal PRIVATE
    -Wno-unused-parameter
)
