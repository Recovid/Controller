
if (NOT ADAPTATION )
    set(ADAPTATION sam)
endif()


if(native_win32) # !POSIX for high-level deterministic tests only
    set(EXE_EXT .exe)
elseif(NOT ${TARGET} STREQUAL "native")
    set(EXE_EXT .${TARGET}.elf)
endif()

set(EXECUTABLE ${APPLICATION}${EXE_EXT})

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

if(NO_RASPI_REBOOT)
   add_compile_definitions(NO_RASPI_REBOOT)
endif()



target_link_libraries(${EXECUTABLE} m platform )
target_compile_definitions(${EXECUTABLE} PUBLIC NTESTS) # remove unit-tests from target



# Targets for flashing on stm32
if(${TARGET} STREQUAL "recovid_revB")
    Setup_Programming_for(${EXECUTABLE} )
endif()


