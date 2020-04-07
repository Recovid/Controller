cmake_minimum_required(VERSION 3.5)

project(C_Controller LANGUAGES C)

add_executable(C_Controller
    platform.h
    main.c
    controller_settings.h
    controller.h
    controller.c
    configuration.h
    configuration.c
    alarms.h
    alarms.c
    ihm_communication.h
    ihm_communication.c
    hardware_simulation.h
    hardware_simulation.c
    unit_tests.h
    unit_tests.c
    )

