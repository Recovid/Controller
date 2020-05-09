cmake_minimum_required(VERSION 3.12)

###############################################################################
# Compilers setup

if( NOT TARGET )
    set(TARGET recovid_revB) # default target
endif()


set(PROJECT_DIR ${CMAKE_CURRENT_LIST_DIR})
set(COMMON_INC  ${PROJECT_DIR}/include)
set(TARGET_DIR  ${PROJECT_DIR}/platforms/${TARGET})

if (NOT EXISTS ${TARGET_DIR})
    message(FATAL_ERROR "Target platform ${TARGET} not supported")
endif()

if(DISABLE_RASPI_WAIT)
   add_compile_definitions(DISABLE_RASPI_WAIT)
endif()

SET(CMAKE_TOOLCHAIN_FILE ${TARGET_DIR}/Toolchain.cmake)

add_compile_definitions(${TARGET})

if(NO_RASPI_REBOOT)
   add_compile_definitions(NO_RASPI_REBOOT)
endif()

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache) # Less useful to do it for linking, see edit2
endif(CCACHE_FOUND)

# Export compile_commands.json for IDE integration
set(CMAKE_EXPORT_COMPILE_COMMANDS true)

###############################################################################
# Project

project(Controller C CXX ASM)

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


###############################################################################
# Platform

add_subdirectory(${TARGET_DIR})

###############################################################################
# Top-level Targets

if(native_win32) # !POSIX for high-level deterministic tests only
    set(EXE_EXT .exe)
elseif(NOT ${TARGET} STREQUAL "native")
    set(EXE_EXT .${TARGET}.elf)
endif()

set(EXECUTABLE ${PROJECT_NAME}${EXE_EXT})


# Controller
set(SOURCES
   src/controller.c
   src/monitoring.c
   src/breathing.c
   src/controller.c
   src/hmi.c
   src/protocol.c
   src/compute_motor.c
)

include_directories(
    ${COMMON_INC}
    src
)

add_executable(${EXECUTABLE} ${SOURCES} )

target_link_libraries(${EXECUTABLE} m platform )
target_compile_definitions(${EXECUTABLE} PUBLIC NTESTS) # remove unit-tests from target

#add_executable(       test_${EXECUTABLE} ${SHARED_SOURCES} ${TARGET_DIR}/tests/main.c)
#target_link_libraries(test_${EXECUTABLE} m platform freertos  ${OPTIONAL_HAL} ${OPTIONAL_PTHREAD})

#add_test(NAME unit-tests COMMAND test_${EXECUTABLE})

# Targets for flashing on stm32
if(${TARGET} STREQUAL "recovid_revB")
    Setup_Programming_for(${EXECUTABLE} )
endif()
