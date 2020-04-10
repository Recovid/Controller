set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.7)

project(C_Controller)

if(CMAKE_HOST_WIN32) # !POSIX for high-level deterministic tests only
    set(TARGET native)
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

# high-level
set(SOURCES
    platform.h
    main.c
    unit_tests.h
    unit_tests.c
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
    ${LOWLEVEL_INC_DIR}/hardware_simulation.h
    ${LOWLEVEL_SRC_DIR}/hardware_simulation.c
)
# low-level
if(NOT CMAKE_HOST_WIN32)
set(SOURCES ${SOURCES}
    ${LOWLEVEL_INC_DIR}/hardware_serial.h
    ${LOWLEVEL_SRC_DIR}/hardware_serial.c
    tasks_recovid.c
    tasks_recovid.h
    TaskMessageManagement.c
    TaskRespirationCycle.c
    TaskSensing.c
    TaskAlarm.c
    )

    include(${CMAKE_SOURCE_DIR}/lowlevel/${TARGET}/${TARGET}.cmake)
endif()

add_executable(${EXECUTABLE} ${SOURCES})

include_directories(${CMAKE_SOURCE_DIR} ${LOWLEVEL_INC_DIR})

target_link_libraries(${EXECUTABLE} m)
set_target_properties(${EXECUTABLE} PROPERTIES LINKER_LANGUAGE C)
