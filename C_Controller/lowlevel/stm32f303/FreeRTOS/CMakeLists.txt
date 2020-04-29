set(FREERTOS_DIR_STM32 ${CMAKE_CURRENT_LIST_DIR})
set(FREERTOS_SRC_DIR_STM32 ${FREERTOS_DIR_STM32}/Source)
set(FREERTOS_INC_DIR_STM32 ${FREERTOS_DIR_STM32}/Source/include)

add_library(freertos OBJECT 
    ${FREERTOS_SRC_DIR_STM32}/CMSIS_RTOS_V2/cmsis_os2.c
    ${FREERTOS_SRC_DIR_STM32}/timers.c
    ${FREERTOS_SRC_DIR_STM32}/queue.c
    ${FREERTOS_SRC_DIR_STM32}/tasks.c
    ${FREERTOS_SRC_DIR_STM32}/croutine.c
    ${FREERTOS_SRC_DIR_STM32}/list.c
    ${FREERTOS_SRC_DIR_STM32}/portable/MemMang/heap_4.c
    ${FREERTOS_SRC_DIR_STM32}/portable/GCC/ARM_CM4F/port.c
)

target_include_directories(freertos PUBLIC
    ${FREERTOS_INC_DIR_STM32}
    ${FREERTOS_SRC_DIR_STM32}
    ${FREERTOS_SRC_DIR_STM32}/portable/GCC/ARM_CM4F
    ${LOWLEVEL_INC_DIR} # For FreeRTOSConfig.h
	${LOWLEVEL_SRC_DIR}/HAL/
	${LOWLEVEL_SRC_DIR}/HAL/Drivers/CMSIS/Include/
    ${LOWLEVEL_SRC_DIR}/HAL/Drivers/CMSIS/Device/ST/STM32F3xx/Include
	${LOWLEVEL_SRC_DIR}/HAL/Drivers/STM32F3xx_HAL_Driver/Inc/
	)

message(${LOWLEVEL_INC_DIR})
