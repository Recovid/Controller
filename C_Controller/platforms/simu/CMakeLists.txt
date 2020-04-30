add_compile_definitions(MAX_NUMBER_OF_TASKS=300)
add_compile_definitions(MAX_NUMBER_OF_TASKS=300)

if(NOT CMAKE_HOST_WIN32) # !POSIX cannot use FreeRTOS which requires sys/time.h for instance
	set(FREERTOS_DIR_X86        ${PLATFORM_SRC_DIR}/FreeRTOS-Sim)
    set(FREERTOS_SRC_DIR_X86    ${FREERTOS_DIR_X86}/Source)
    set(FREERTOS_INC_DIR_X86    ${FREERTOS_DIR_X86}/Source/include)



    add_library(freertos OBJECT
        ${FREERTOS_SRC_DIR_X86}/croutine.c
        ${FREERTOS_SRC_DIR_X86}/event_groups.c
        ${FREERTOS_SRC_DIR_X86}/list.c
        ${FREERTOS_SRC_DIR_X86}/queue.c
        ${FREERTOS_SRC_DIR_X86}/tasks.c
        ${FREERTOS_SRC_DIR_X86}/timers.c
        ${FREERTOS_SRC_DIR_X86}/portable/MemMang/heap_3.c
        ${FREERTOS_SRC_DIR_X86}/portable/GCC/POSIX/port.c
        vassertcall.c
    )
    target_include_directories(freertos PUBLIC
        ${FREERTOS_SRC_DIR_X86}
        ${FREERTOS_SRC_DIR_X86}/portable/GCC/POSIX
		${FREERTOS_INC_DIR_X86}
        ${PLATFORM_INC_DIR}
		${PLATFORM_SRC_DIR}
    )
    target_link_libraries(freertos PUBLIC pthread)
endif()

###############################################################################

set(PLATFORM_SOURCES
    lowlevel_simulation.c
    simple_indicators.c
    flash.c
    ups.c
    gpio.c
    time.c
)

add_compile_definitions(MAX_NUMBER_OF_TASKS=300)

if(NOT CMAKE_HOST_WIN32)
    set(PLATFORM_SOURCES ${PLATFORM_SOURCES} uart.c)
endif()

add_library(platform OBJECT ${PLATFORM_SOURCES})

target_include_directories(platform PUBLIC 
	${PLATFORM_INC_DIR}
	${PLATFORM_SRC_DIR}
    ${FREERTOS_SRC_DIR_X86}/portable/GCC/POSIX
	${FREERTOS_INC_DIR_X86}
	)

