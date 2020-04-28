add_subdirectory(HAL)
add_subdirectory(FreeRTOS)

add_library(lowlevel OBJECT 
    stm32f3xx_hal_msp_tick.c
    stm32f3xx_hal_msp.c
    stm32f3xx_it.c

    recovid_revB.c
    hardware_serial.c
    ihm.c
    motor.c
    motor_pep.c
    sensors.c
    buzzer.c
    lights.c
    pep_valve.c
    flash.c
)

target_include_directories(lowlevel PUBLIC 
	${LOWLEVEL_INC_DIR}
	${LOWLEVEL_SRC_DIR}/HAL/
	${LOWLEVEL_SRC_DIR}/HAL/Drivers/STM32F3xx_HAL_Driver/Inc/
	${LOWLEVEL_SRC_DIR}/HAL/Drivers/CMSIS/Include
	${LOWLEVEL_SRC_DIR}/HAL/Drivers/CMSIS/Device/ST/STM32F3xx/Include/
	${LOWLEVEL_SRC_DIR}/HAL/Drivers/CMSIS/Include
	${LOWLEVEL_SRC_DIR}/FreeRTOS/Source/include/
	${LOWLEVEL_SRC_DIR}/FreeRTOS/Source/portable/GCC/ARM_CM4F/
	)

target_compile_definitions(lowlevel PUBLIC NTESTS) # remove unit-tests from target
target_compile_definitions(hal PUBLIC NTESTS) # remove unit-tests from target
target_compile_definitions(freertos PUBLIC NTESTS) # remove unit-tests from target

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
