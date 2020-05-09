set(PLATFORM_DIR ${CMAKE_CURRENT_LIST_DIR})

if(NO_RASPI_REBOOT)
    add_compile_definitions(NO_RASPI_REBOOT)
endif()

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
)

target_include_directories(recovid_revB PUBLIC 
    ${COMMON_INC}
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
