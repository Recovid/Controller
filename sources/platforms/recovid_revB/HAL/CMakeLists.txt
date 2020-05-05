add_library(hal OBJECT
    STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_cortex.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_dma.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_exti.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash_ex.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_gpio.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c_ex.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr_ex.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc_ex.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim_ex.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart_ex.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_hal.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_adc.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_comp.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_crc.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_dac.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_dma.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_exti.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_fmc.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_gpio.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_hrtim.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_i2c.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_opamp.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_pwr.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_rcc.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_rtc.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_spi.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_tim.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_usart.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_usb.c
    STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_utils.c
)
target_include_directories(hal PUBLIC
	${PLATFORM_DIR}/core                # for stm32f3xx_hal_conf.h
	CMSIS/Include/
    CMSIS/Device/ST/STM32F3xx/Include
	STM32F3xx_HAL_Driver/Inc/
)



target_compile_options(hal PRIVATE
    -Wno-unused-parameter
)
