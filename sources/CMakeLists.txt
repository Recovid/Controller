cmake_minimum_required(VERSION 3.12)

set(PROJECT_DIR ${CMAKE_CURRENT_LIST_DIR})


if( NOT TARGET )
    set(TARGET recovid_revB) # default target
endif()

if (NOT APPLICATION)
    set(APPLICATION Controller)
endif()


set(TARGET_DIR      ${PROJECT_DIR}/platforms/${TARGET})
set(APPLICATION_DIR ${PROJECT_DIR}/applications/${APPLICATION})


if (NOT EXISTS ${APPLICATION_DIR})
    message(FATAL_ERROR "Application ${APPLICATION} not supported")
endif()

if (NOT EXISTS ${TARGET_DIR})
    message(FATAL_ERROR "Target platform ${TARGET} not supported")
endif()

if (NOT EXISTS ${APPLICATION_DIR}/include/${TARGET})
    message(FATAL_ERROR "Application ${APPLICATION} not supported by ${TARGET}")
endif()

set(COMMON_INC  
    ${PROJECT_DIR}/include
)



SET(CMAKE_TOOLCHAIN_FILE ${TARGET_DIR}/Toolchain.cmake)


find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache) # Less useful to do it for linking, see edit2
endif(CCACHE_FOUND)

# Export compile_commands.json for IDE integration
set(CMAKE_EXPORT_COMPILE_COMMANDS true)

###############################################################################
# Project

project(${APPLICATION} C CXX ASM)

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
# Application Targets

add_subdirectory(${APPLICATION_DIR})


