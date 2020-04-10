cmake_minimum_required(VERSION 3.7)

###############################################################################
# Use Toolchain file

set(Lowlevel_dir  ${CMAKE_SOURCE_DIR}/lowlevel)

if(CMAKE_HOST_WIN32) # !POSIX for high-level deterministic tests only
    set(TARGET native)
    set(CMAKE_C_FLAGS -Wpedantic)
elseif(NOT TARGET)
    set(TARGET stm32f303) # default target for system tests
endif()

SET(CMAKE_TOOLCHAIN_FILE ${Lowlevel_dir}/${TARGET}/Toolchain.cmake)

###############################################################################
# Project

project(C_Controller C CXX ASM)

###############################################################################
# CFlags

add_compile_options(
    -fdiagnostics-color=always
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-unused-variable
)

# TODO: Remove this because lowlevel shouldn't include top-level
include_directories(.)

###############################################################################
# Libraries

add_subdirectory(lowlevel)
if (NOT TARGET lowlevel)
    message(FATAL_ERROR "lowlevel target ${TARGET} does not define a 'lowlevel' library")
endif()

###############################################################################
# Top-level Targets

if(${TARGET} STREQUAL "stm32f303")
    set(EXECUTABLE ${PROJECT_NAME}.${TARGET}.elf)
else()
    if(CMAKE_HOST_WIN32)
        set(EXECUTABLE simulateur.exe)
    else()
        set(EXECUTABLE simulateur)
    endif()
endif()

SET(SHARED_SOURCES
    main.c
    unit_tests.c
    sensing.c
    controller.c
    configuration.c
    alarms.c
    ihm_communication.c
)
# low-level
if(NOT CMAKE_HOST_WIN32)
    set(SHARED_SOURCES ${SHARED_SOURCES}
        tasks_recovid.c
        TaskMessageManagement.c
        TaskRespirationCycle.c
        TaskSensing.c
        TaskAlarm.c
    )
endif()

add_executable(${EXECUTABLE} ${SHARED_SOURCES})

set_target_properties(${EXECUTABLE} PROPERTIES LINKER_LANGUAGE C)


target_link_libraries(${EXECUTABLE} m lowlevel)


set(HEX_FILE ${EXECUTABLE}.hex)
set(BIN_FILE ${EXECUTABLE}.bin)

add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${EXECUTABLE}> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${EXECUTABLE}> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}"
)
