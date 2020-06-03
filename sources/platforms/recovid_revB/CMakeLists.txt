set(PLATFORM_DIR ${CMAKE_CURRENT_LIST_DIR})

add_subdirectory(HAL)
add_subdirectory(FreeRTOS)

add_library(recovid_revB OBJECT 
    core/startup_stm32f303retx.s
    core/system_stm32f3xx.c
    core/syscalls.c
    core/sysmem.c
    core/recovid_revB.c
    core/recovid_revB_hal_msp_tick.c
    core/recovid_revB_hal_msp.c
    core/recovid_revB_hal_it.c
    drivers/time.c
    drivers/uart.c
    drivers/motor.c
    drivers/motor_pep.c
    drivers/sensors.c
    drivers/indicators.c
    drivers/pep_valve.c
    drivers/gpio.c
    drivers/bmp280.c
    drivers/log.c
)

target_include_directories(recovid_revB PUBLIC 
    ${COMMON_INC}
	${APPLICATION_DIR}/include/${TARGET}
	${PLATFORM_DIR}/include
	${PLATFORM_DIR}/core
	${PLATFORM_DIR}/drivers
	${PLATFORM_DIR}/HAL/STM32F3xx_HAL_Driver/Inc/
	${PLATFORM_DIR}/HAL/CMSIS/Include
	${PLATFORM_DIR}/HAL/CMSIS/Device/ST/STM32F3xx/Include/
	${PLATFORM_DIR}/HAL/CMSIS/Include
	${PLATFORM_DIR}/FreeRTOS/Source/include/
	${PLATFORM_DIR}/FreeRTOS/Source/portable/GCC/ARM_CM4F/
	)

add_library(platform INTERFACE )

target_sources(platform INTERFACE 
    $<TARGET_OBJECTS:recovid_revB>
    $<TARGET_OBJECTS:hal>
    $<TARGET_OBJECTS:freertos>
)

target_include_directories(platform INTERFACE 
    ${PLATFORM_DIR}/include 
    ${PLATFORM_DIR}/FreeRTOS/Source/include/
	${PLATFORM_DIR}/FreeRTOS/Source/portable/GCC/ARM_CM4F/)


#if(NO_RASPI_REBOOT)
   add_compile_definitions(NO_RASPI_REBOOT)
#endif()

