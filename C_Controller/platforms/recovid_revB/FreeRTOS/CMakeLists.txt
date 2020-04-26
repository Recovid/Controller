set(FREERTOS_DIR_STM32 ${CMAKE_CURRENT_LIST_DIR})
set(FREERTOS_SRC_DIR_STM32 ${FREERTOS_DIR_STM32}/Source)
set(FREERTOS_INC_DIR_STM32 ${FREERTOS_DIR_STM32}/Source/include)

add_library(freertos STATIC
    ${FREERTOS_SRC_DIR_STM32}/CMSIS_RTOS_V2/cmsis_os2.c
    ${FREERTOS_SRC_DIR_STM32}/timers.c
    ${FREERTOS_SRC_DIR_STM32}/queue.c
    ${FREERTOS_SRC_DIR_STM32}/tasks.c
    ${FREERTOS_SRC_DIR_STM32}/croutine.c
    ${FREERTOS_SRC_DIR_STM32}/event_groups.c
    ${FREERTOS_SRC_DIR_STM32}/list.c
    ${FREERTOS_SRC_DIR_STM32}/portable/MemMang/heap_4.c
    ${FREERTOS_SRC_DIR_STM32}/portable/GCC/ARM_CM4F/port.c
    ../stm32f3xx_hal_msp.c
)

target_include_directories(freertos PUBLIC
    ${FREERTOS_INC_DIR_STM32}
    ${FREERTOS_SRC_DIR_STM32}
    ${FREERTOS_SRC_DIR_STM32}/portable/GCC/ARM_CM4F
    ..                  # For FreeRTOSConfig.h
)

target_link_libraries(freertos PUBLIC hal)
