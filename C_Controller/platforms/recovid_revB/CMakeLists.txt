set(PLATFORM_INC_DIR .)

add_subdirectory(HAL)
add_subdirectory(FreeRTOS)

add_library(platform OBJECT 
    recovid_revB.c
    syscalls.c
    sysmem.c
    stm32f3xx_hal_msp_tick.c
    stm32f3xx_hal_msp.c
    stm32f3xx_it.c
    time.c
    uart.c
    motor.c
    motor_pep.c
    sensors.c
    indicators.c
    pep_valve.c
    gpio.c
	log_timings.c
)

target_include_directories(platform PUBLIC 
	${PLATFORM_INC_DIR}
	${PLATFORM_SRC_DIR}/HAL/
	${PLATFORM_SRC_DIR}/HAL/Drivers/STM32F3xx_HAL_Driver/Inc/
	${PLATFORM_SRC_DIR}/HAL/Drivers/CMSIS/Include
	${PLATFORM_SRC_DIR}/HAL/Drivers/CMSIS/Device/ST/STM32F3xx/Include/
	${PLATFORM_SRC_DIR}/HAL/Drivers/CMSIS/Include
	${PLATFORM_SRC_DIR}/FreeRTOS/Source/include/
	${PLATFORM_SRC_DIR}/FreeRTOS/Source/portable/GCC/ARM_CM4F/
	)

target_include_directories(platform PUBLIC ${PLATFORM_INC_DIR})
target_link_libraries(platform PUBLIC hal freertos)

###############################################################################
# Setup OpenOCD

if(NOT CMAKE_HOST_WIN32)
    add_custom_target(install_udev
        sudo ${CMAKE_SOURCE_DIR}/tools/install_udev.sh
        USES_TERMINAL
    )
endif()

function(Setup_Programming_for)
    set(list_var "${ARGV}")
    foreach(exe ${list_var})

        set(HEX_FILE ${exe}.hex)
        set(BIN_FILE ${exe}.bin)
        add_custom_command(OUTPUT ${HEX_FILE} ${BIN_FILE}
            COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${exe}> ${HEX_FILE}
            COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${exe}> ${BIN_FILE}
            COMMENT "Objcopy ${HEX_FILE}"
			DEPENDS ${exe}
        )

		add_custom_target("${exe}.binary" DEPENDS ${BIN_FILE})

        set(target_name "${exe}.flash")
        message("Created target ${target_name}")
        add_custom_target("${target_name}"
            openocd -f ${OPENOCD_CFG}
                -c "init"
                -c "reset init"
                -c "flash write_image erase ${exe}"
                -c "reset"
                -c "shutdown"
            DEPENDS ${HEX_FILE}
        )
    endforeach()
endfunction()
