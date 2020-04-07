cmake_minimum_required(VERSION 3.5)

project(C_Controller LANGUAGES C)

add_executable(C_Controller
    main.c
    controller_settings.h
    controller.h
    controller.c
    ihm_communication.h
    ihm_communication.c
    hardware_simulation.h
    hardware_simulation.c)

target_link_libraries(C_Controller m)
