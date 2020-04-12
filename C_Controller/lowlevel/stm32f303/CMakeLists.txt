add_subdirectory(HAL)
add_subdirectory(FreeRTOS)

add_library(lowlevel STATIC
    stm32f3xx_hal_msp_tick.c
    stm32f3xx_hal_msp.c
    stm32f3xx_it.c

    hardware_simulation.c
    hardware_serial.c
    simple_indicators.c
    flash.c
)

target_include_directories(lowlevel PUBLIC ${LOWLEVEL_INC_DIR})
target_link_libraries(lowlevel PUBLIC hal freertos)


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
