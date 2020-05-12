
if (NOT ADAPTATION )
    set(ADAPTATION sam)
endif()


if(native_win32) # !POSIX for high-level deterministic tests only
    set(EXE_EXT .exe)
elseif(NOT ${TARGET} STREQUAL "native")
    set(EXE_EXT .${TARGET}.elf)
endif()

set(EXECUTABLE ${APPLICATION}${EXE_EXT})

message("Adding executable ${EXECUTABLE}")
add_executable(${EXECUTABLE} 
    ${APPLICATION_DIR}/src/controller.c
    ${APPLICATION_DIR}/src/monitoring.c
    ${APPLICATION_DIR}/src/breathing.c
    ${APPLICATION_DIR}/src/hmi.c
    ${APPLICATION_DIR}/src/protocol.c
    ${APPLICATION_DIR}/src/${ADAPTATION}/adaptation.c
)

target_include_directories(${EXECUTABLE} PUBLIC 
    ${COMMON_INC}
    ${APPLICATION_DIR}/src
    ${APPLICATION_DIR}/include
    ${APPLICATION_DIR}/include/${TARGET}
)

#if(NO_RASPI_REBOOT)
   add_compile_definitions(NO_RASPI_REBOOT)
#endif()



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

if(${TARGET} STREQUAL "recovid_revB")
    set(HEX_FILE ${EXECUTABLE}.hex)
    set(BIN_FILE ${EXECUTABLE}.bin)
    message("Adding custom command for ${EXECUTABLE}")
    add_custom_command(TARGET ${EXECUTABLE}
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${EXECUTABLE}> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${EXECUTABLE}> ${BIN_FILE}
        COMMAND ${SIZE} $<TARGET_FILE:${EXECUTABLE}>
        COMMENT "Objcopy ${HEX_FILE}"
        POST_BUILD
    )

    set(target_name "${EXECUTABLE}.flash")
    message("Created target ${target_name}")
    add_custom_target("${target_name}"
        openocd -f ${OPENOCD_CFG}
            -c "init"
            -c "reset init"
            -c "flash write_image erase ${EXECUTABLE}"
            -c "reset"
            -c "shutdown"
        DEPENDS ${HEX_FILE}
    )
endif()

