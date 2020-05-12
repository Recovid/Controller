


if(native_win32) # !POSIX for high-level deterministic tests only
    set(EXE_EXT .exe)
elseif(NOT ${TARGET} STREQUAL "native")
    set(EXE_EXT .${TARGET}.elf)
endif()

set(EXECUTABLE ${APPLICATION}${EXE_EXT})

add_executable(${EXECUTABLE} 
    ${APPLICATION_DIR}/src/application.c
    ${APPLICATION_DIR}/src/calibration.c
    ${APPLICATION_DIR}/src/hmi.c
    ${APPLICATION_DIR}/src/protocol.c
)

target_include_directories(${EXECUTABLE} PUBLIC 
    ${COMMON_INC}
    ${APPLICATION_DIR}/src
    ${APPLICATION_DIR}/include
    ${APPLICATION_DIR}/include/${TARGET}
)

add_compile_definitions(NO_RASPI_REBOOT)


target_link_libraries(${EXECUTABLE} m platform )
target_compile_definitions(${EXECUTABLE} PUBLIC NTESTS) # remove unit-tests from target



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
            COMMAND ${SIZE} $<TARGET_FILE:${exe}>
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


# Targets for flashing on stm32
if(${TARGET} STREQUAL "recovid_revB")
    Setup_Programming_for(${EXECUTABLE} )
endif()