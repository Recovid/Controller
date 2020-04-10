SET(LOWLEVEL_SRC_DIR ${CMAKE_SOURCE_DIR}/lowlevel/${TARGET})
SET(HAL_DIR ${LOWLEVEL_SRC_DIR}/HAL/)
SET(LINKER_SCRIPT ${HAL_DIR}/stm32f303retx.ld)
# specify cross compilers and tools
SET(CMAKE_C_COMPILER_WORKS 1)
SET(CMAKE_C_COMPILER arm-none-eabi-gcc)
SET(CMAKE_CXX_COMPILER_WORKS 1)
SET(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)

SET(FREERTOS_DIR_STM32 ${LOWLEVEL_SRC_DIR}/FreeRTOS/)
SET(FREERTOS_SRC_DIR_STM32 ${FREERTOS_DIR_STM32}/Source/)
SET(FREERTOS_INC_DIR_STM32 ${FREERTOS_DIR_STM32}/Source/include/)
file(GLOB_RECURSE SOURCES_STM32 "startup/*.*" "Drivers/*.*" "Core/*.*")
set(SOURCES_STM32 ${SOURCES_STM32}
	${HAL_DIR}/stm32f3xx_nucleo.c
	${HAL_DIR}/syscalls.c
	${HAL_DIR}/sysmem.c
	${HAL_DIR}/system_stm32f3xx.c
	${LOWLEVEL_SRC_DIR}/stm32f3xx_it.c
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_cortex.c
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_dma.c
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_exti.c
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash_ex.c
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash.c
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_gpio.c
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c_ex.c
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c.c
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr_ex.c
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr.c
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc_ex.c
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc.c
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim_ex.c
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim.c
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart_ex.c
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart.c
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal.c
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_usart.c
	${HAL_DIR}/Drivers/startup_stm32f303retx.s
	${FREERTOS_SRC_DIR_STM32}/CMSIS_RTOS_V2/cmsis_os2.c
	${FREERTOS_SRC_DIR_STM32}/timers.c
    ${FREERTOS_SRC_DIR_STM32}/queue.c
    ${FREERTOS_SRC_DIR_STM32}/tasks.c
    ${FREERTOS_SRC_DIR_STM32}/croutine.c
    ${FREERTOS_SRC_DIR_STM32}/list.c
	${FREERTOS_SRC_DIR_STM32}/portable/MemMang/heap_4.c
	${FREERTOS_SRC_DIR_STM32}/portable/GCC/ARM_CM4F/port.c	
	${LOWLEVEL_SRC_DIR}/stm32f3xx_hal_msp.c
	${LOWLEVEL_SRC_DIR}/stm32f3xx_hal_msp_tick.c
	${LOWLEVEL_INC_DIR}/hardware_simulation.h
	${LOWLEVEL_SRC_DIR}/hardware_simulation.c
	${LOWLEVEL_INC_DIR}/hardware_serial.h
	${LOWLEVEL_SRC_DIR}/hardware_serial.c)


target_sources(${EXECUTABLE} PRIVATE ${SHARED_SOURCES} ${SOURCES_STM32})


SET(COMMON_FLAGS " -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F303xE -DUSE_FULL_LL_DRIVER -Os -g -ffunction-sections -fdata-sections -Wall -fstack-usage  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")

SET(CMAKE_C_FLAGS "${COMMON_FLAGS}")

SET(CMAKE_EXE_LINKER_FLAGS "-T ${LINKER_SCRIPT} -Wl,-Map=stm32f303_memory.map --specs=nosys.specs -Wl,--gc-sections")

add_definitions(-D__weak=__attribute__\(\(weak\)\) -D__packed=__attribute__\(\(__packed__\)\) -DUSE_HAL_DRIVER -DSTM32F303xE)


include_directories(${LOWLEVEL_INC_DIR}
	${HAL_DIR}
	${HAL_DIR}/Core/Inc	
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Inc	
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy	
	${HAL_DIR}/Drivers/CMSIS/Device/ST/STM32F3xx/Include	
	${HAL_DIR}/Drivers/CMSIS/Include	
	${FREERTOS_SRC_DIR_STM32}	
	${FREERTOS_INC_DIR_STM32}	
	${FREERTOS_SRC_DIR_STM32}/portable/GCC/ARM_CM4F/)




set(HEX_FILE ${EXECUTABLE}.hex)
set(BIN_FILE ${EXECUTABLE}.bin)

add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${EXECUTABLE}> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${EXECUTABLE}> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}")

