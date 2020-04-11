set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.7)

project(C_Controller C ASM)

if(CMAKE_HOST_WIN32) # !POSIX for high-level deterministic tests only
    set(TARGET native)
    set(CMAKE_C_FLAGS -Wpedantic)
elseif(NOT TARGET)
    set(TARGET stm32f303) # default target for system tests
endif()

set(LOWLEVEL_INC_DIR ${CMAKE_SOURCE_DIR}/lowlevel/include)
set(LOWLEVEL_SRC_DIR ${CMAKE_SOURCE_DIR}/lowlevel/${TARGET})

if(CMAKE_HOST_WIN32)
    set(EXECUTABLE simulateur.exe)
elseif(${TARGET} STREQUAL "stm32f303")
    set(EXECUTABLE ${PROJECT_NAME}.${TARGET}.elf)
else()
    set(EXECUTABLE simulateur)
endif()

add_compile_definitions(${TARGET})

# high-level
set(SHARED_SOURCES
    platform.h
    sensing.h
    sensing.c
    controller.h
    controller.c
    configuration.h
    configuration.c
    alarms.h
    alarms.c
    ihm_communication.h
    ihm_communication.c
    ${LOWLEVEL_INC_DIR}/lowlevel.h
    )
if(CMAKE_HOST_WIN32) # simulation only
set(LOWLEVEL_SOURCES
    ${LOWLEVEL_SRC_DIR}/lowlevel_simulation.c
    )
else() # mixture of simulation and implementation
set(LOWLEVEL_SOURCES
    ${LOWLEVEL_SRC_DIR}/lowlevel_simulation.c
    ${LOWLEVEL_INC_DIR}/hardware_serial.h
    ${LOWLEVEL_SRC_DIR}/hardware_serial.c
    tasks_recovid.c
    tasks_recovid.h
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

if(NOT CMAKE_HOST_WIN32)
include(${CMAKE_SOURCE_DIR}/lowlevel/${TARGET}/${TARGET}.cmake)
endif()

include_directories(${CMAKE_SOURCE_DIR} ${LOWLEVEL_INC_DIR})

target_link_libraries(${EXECUTABLE} m)
set_target_properties(${EXECUTABLE} PROPERTIES LINKER_LANGUAGE C)
