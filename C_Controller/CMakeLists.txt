

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.7)
project(C_Controller C ASM)


if(NOT TARGET)
    set(TARGET stm32f303)
endif()

SET(LOWLEVEL_INC_DIR ${CMAKE_SOURCE_DIR}/lowlevel/include)
SET(LOWLEVEL_SRC_DIR ${CMAKE_SOURCE_DIR}/lowlevel/${TARGET})
if(${TARGET} STREQUAL "stm32f303")
	SET(EXECUTABLE ${PROJECT_NAME}.${TARGET}.elf)
else()
	SET(EXECUTABLE simulateur)
endif()

add_definitions(-D${TARGET})
SET(SOURCES platform.h
    main.c
    controller.h
    controller.c
    configuration.h
    configuration.c
    alarms.h
    alarms.c
    ihm_communication.h
    ihm_communication.c
    tasks_recovid.c
    tasks_recovid.h
    ${LOWLEVEL_INC_DIR}/hardware_simulation.h
    ${LOWLEVEL_SRC_DIR}/hardware_simulation.c
    ${LOWLEVEL_INC_DIR}/hardware_serial.h
    ${LOWLEVEL_SRC_DIR}/hardware_serial.c
    unit_tests.h
    unit_tests.c
	TaskMessageManagement.c
	TaskRespirationCycle.c
	TaskSensing.c
	TaskAlarm.c
)

add_executable(${EXECUTABLE} ${SOURCES})

include(${CMAKE_SOURCE_DIR}/lowlevel/${TARGET}/${TARGET}.cmake)

include_directories(${CMAKE_SOURCE_DIR} ${LOWLEVEL_INC_DIR})
target_link_libraries(${EXECUTABLE} m)
set_target_properties(${EXECUTABLE} PROPERTIES LINKER_LANGUAGE C)
