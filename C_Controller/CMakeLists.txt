cmake_minimum_required(VERSION 3.7)

###############################################################################
# Compilers setup

set(Lowlevel_dir ${CMAKE_SOURCE_DIR}/lowlevel)

if( (CMAKE_HOST_WIN32) AND  ( (NOT TARGET) OR ("${TARGET}" STREQUAL "native") ) )
	set(native_win32 TRUE)
else()
	set(native_win32 FALSE)
endif()

if(native_win32 OR NOT TARGET)
    set(TARGET native)
elseif(NOT TARGET)
    set(TARGET stm32f303) # default target for system tests
endif()

if(TEST_MOTOR)
	add_compile_definitions("TEST_MOTOR")
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

enable_testing()

###############################################################################
# CFlags

add_compile_options(
    -fdiagnostics-color=always
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-unused-variable
)

if(native_win32)
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

if(native_win32) # !POSIX for high-level deterministic tests only
    set(EXE_EXT .exe)
elseif(NOT ${TARGET} STREQUAL "native")
    set(EXE_EXT .${TARGET}.elf)
endif()
set(EXECUTABLE ${PROJECT_NAME}${EXE_EXT})



# high-level
set(SHARED_SOURCES
    sensing.c
    controller.c
    configuration.c
    alarms.c
    ihm_communication.c
    compute_motor.c
)


# low-level
if(NOT native_win32) # !POSIX for high-level deterministic tests only
    set(SHARED_SOURCES ${SHARED_SOURCES}
        tasks_recovid.c
        TaskMessageManagement.c
        TaskRespirationCycle.c
		TaskSensing.c
        # Sensing is performed by interrupts once init_sensors is called
        # Alarms are monitored and triggered in TaskRespirationCycle
    )
endif()

add_executable(            ${EXECUTABLE} ${SHARED_SOURCES} main.c)

if(${TARGET} STREQUAL "stm32f303")
	set(OPTIONAL_HAL hal)
endif()
target_link_libraries(${EXECUTABLE} m lowlevel freertos ${OPTIONAL_HAL})
target_compile_definitions(${EXECUTABLE} PUBLIC NTESTS) # remove unit-tests from target

add_executable(       test_${EXECUTABLE} ${SHARED_SOURCES} tests/main.c)
target_link_libraries(test_${EXECUTABLE} m lowlevel freertos  ${OPTIONAL_HAL})

add_test(NAME unit-tests COMMAND test_${EXECUTABLE})

# Targets for flashing on stm32
if(${TARGET} STREQUAL "stm32f303")
    Setup_Programming_for(${EXECUTABLE} test_${EXECUTABLE})
endif()
