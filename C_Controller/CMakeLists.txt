cmake_minimum_required(VERSION 3.7)

###############################################################################
# Compilers setup

set(Lowlevel_dir ${CMAKE_SOURCE_DIR}/lowlevel)

if(CMAKE_HOST_WIN32) # !POSIX for high-level deterministic tests only
    set(TARGET native)
elseif(NOT TARGET)
    set(TARGET stm32f303) # default target for system tests
endif()

SET(CMAKE_TOOLCHAIN_FILE ${Lowlevel_dir}/${TARGET}/Toolchain.cmake)

add_compile_definitions(${TARGET})

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache) # Less useful to do it for linking, see edit2
endif(CCACHE_FOUND)

# Export compile_commands.json for IDE integration
set(CMAKE_EXPORT_COMPILE_COMMANDS true)

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
if(CMAKE_HOST_WIN32) # !POSIX for high-level deterministic tests only
    add_compile_options(-Wpedantic)
endif()

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

if(CMAKE_HOST_WIN32) # !POSIX for high-level deterministic tests only
    set(EXECUTABLE ${PROJECT_NAME}.exe)
elseif(${TARGET} STREQUAL "stm32f303")
    set(EXECUTABLE ${PROJECT_NAME}.${TARGET}.elf)
else()
    set(EXECUTABLE ${PROJECT_NAME})
endif()

# high-level
set(SHARED_SOURCES
    sensing.c
    controller.c
    configuration.c
    alarms.c
    ihm_communication.c
)

# low-level
if(CMAKE_HOST_WIN32) # simulation only
    set(LOWLEVEL_SOURCES
        ${LOWLEVEL_SRC_DIR}/lowlevel_simulation.c
    )
else() # mixture of simulation and implementation
    set(LOWLEVEL_SOURCES
        ${LOWLEVEL_SRC_DIR}/lowlevel_simulation.c
        ${LOWLEVEL_SRC_DIR}/hardware_serial.c
        tasks_recovid.c
        TaskMessageManagement.c
        TaskRespirationCycle.c
        TaskSensing.c
        TaskAlarm.c
        )
endif()

add_executable(     ${EXECUTABLE}       main.c ${SHARED_SOURCES} ${LOWLEVEL_SOURCES})
add_executable(test_${EXECUTABLE} tests/main.c ${SHARED_SOURCES} ${LOWLEVEL_SOURCES} unit_tests.c)
target_compile_definitions(test_${EXECUTABLE} PUBLIC TESTS)

enable_testing()
add_test(NAME unit-tests COMMAND test_${EXECUTABLE})

set_target_properties(${EXECUTABLE} PROPERTIES LINKER_LANGUAGE C)
target_link_libraries(${EXECUTABLE} m lowlevel)

if(NOT CMAKE_HOST_WIN32)
    set(HEX_FILE ${EXECUTABLE}.hex)
    set(BIN_FILE ${EXECUTABLE}.bin)

    add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${EXECUTABLE}> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${EXECUTABLE}> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}"
    )

    if(${TARGET} STREQUAL "stm32f303")
        Setup_OpenOCD_for(${HEX_FILE})
    endif()
endif()
