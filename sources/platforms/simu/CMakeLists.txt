set(PLATFORM_DIR ${CMAKE_CURRENT_LIST_DIR})

add_compile_definitions(MAX_NUMBER_OF_TASKS=300)

#if(NOT CMAKE_HOST_WIN32) # !POSIX cannot use FreeRTOS which requires sys/time.h for instance
	set(FREERTOS_DIR_X86        ${PLATFORM_DIR}/FreeRTOS-Sim)
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
		${PLATFORM_DIR}/include
    )
    target_link_libraries(freertos PUBLIC pthread)
#endif()

###############################################################################
add_library(simu OBJECT
    lowlevel_simulation.c
    simple_indicators.c
    flash.c
    ups.c
    gpio.c
    time.c
    uart.c
    main.c
)

target_include_directories(simu PUBLIC 
    ${COMMON_INC}
    ${PLATFORM_DIR}/include
)


if(NOT CMAKE_HOST_WIN32)
	target_sources(simu INTERFACE $<TARGET_OBJECTS:simu> uart.c)
endif()

target_include_directories(simu PUBLIC
        ${FREERTOS_SRC_DIR_X86}
        ${FREERTOS_SRC_DIR_X86}/portable/GCC/POSIX
		${FREERTOS_INC_DIR_X86}
		${PLATFORM_DIR}
        ${PLATFORM_DIR}/include
)

add_library(platform INTERFACE )
target_sources(platform INTERFACE
    $<TARGET_OBJECTS:simu>
    $<TARGET_OBJECTS:freertos>
)



target_include_directories(platform INTERFACE
	${PLATFORM_DIR}
        ${PLATFORM_DIR}/include
    ${FREERTOS_SRC_DIR_X86}/portable/GCC/POSIX
	${FREERTOS_INC_DIR_X86}
	)

