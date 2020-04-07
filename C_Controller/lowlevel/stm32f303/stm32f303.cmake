message("COUCOU")


SET(HAL_DIR ${LOWLEVEL_SRC_DIR}/HAL/)
SET(LINKER_SCRIPT ${HAL_DIR}/stm32f303retx.ld)
# specify cross compilers and tools
SET(CMAKE_C_COMPILER_WORKS 1)
SET(CMAKE_C_COMPILER arm-none-eabi-gcc)
SET(CMAKE_CXX_COMPILER_WORKS 1)
SET(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)


SET(FREERTOS_DIR_STM32 ${LOWLEVEL_SRC_DIR}/FreeRTOS/)
SET(FREERTOS_SRC_DIR_STM32 ${FREERTOS_DIR_STM32}/Source/)
SET(FREERTOS_INC_DIR_STM32 ${FREERTOS_DIR_STM32}/Source/include/)
file(GLOB_RECURSE SOURCES_STM32 "startup/*.*" "Drivers/*.*" "Core/*.*")
set(SOURCES_STM32 ${SOURCES_STM32} 
	${HAL_DIR}/stm32f3xx_nucleo.c
	${HAL_DIR}/syscalls.c
	${HAL_DIR}/sysmem.c
	${HAL_DIR}/system_stm32f3xx.c
    ${FREERTOS_SRC_DIR_STM32}/timers.c
    ${FREERTOS_SRC_DIR_STM32}/queue.c
    ${FREERTOS_SRC_DIR_STM32}/tasks.c
    ${FREERTOS_SRC_DIR_STM32}/croutine.c
    ${FREERTOS_SRC_DIR_STM32}/list.c
	${LOWLEVEL_SRC_DIR}/stm32f3xx_hal_msp.c)


target_sources(${EXECUTABLE} PRIVATE ${SOURCES} ${SOURCES_STM32})

SET(COMMON_FLAGS "-ffunction-sections -fdata-sections -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mthumb -specs=nosys.specs -DUSE_HAL_DRIVER -DSTM32F303xE")

SET(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++11")
SET(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu99")

SET(CMAKE_EXE_LINKER_FLAGS "-T ${LINKER_SCRIPT} -Wl,--gc-sections -lstdc++_nano -lc -lg")
set(CMAKE_CXX_STANDARD 11)

add_definitions(-D__weak=__attribute__\(\(weak\)\) -D__packed=__attribute__\(\(__packed__\)\) -DUSE_HAL_DRIVER -DSTM32F303xE)


include_directories(${LOWLEVEL_INC_DIR} 
	${HAL_DIR} 
	${HAL_DIR}/Core/Inc	
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Inc	
	${HAL_DIR}/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy	
	${HAL_DIR}/Drivers/CMSIS/Device/ST/STM32F3xx/Include	
	${HAL_DIR}/Drivers/CMSIS/Include	
	${FREERTOS_SRC_DIR_STM32}	
	${FREERTOS_INC_DIR_STM32}	
	${FREERTOS_SRC_DIR_STM32}/portable/GCC/ARM_CM4F/)




set(HEX_FILE ${EXECUTABLE}.hex)
set(BIN_FILE ${EXECUTABLE}.bin)

add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${EXECUTABLE}> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${EXECUTABLE}> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}")

