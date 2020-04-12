set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

###############################################################################
# Set toolchain / compilers

set(toolchain_prefix arm-none-eabi-)

set(CMAKE_C_COMPILER    ${toolchain_prefix}gcc)
set(CMAKE_ASM_COMPILER  ${toolchain_prefix}gcc)
set(CMAKE_CXX_COMPILER  ${toolchain_prefix}g++)
set(CMAKE_AR            ${toolchain_prefix}ar)
set(CMAKE_OBJCOPY       ${toolchain_prefix}objcopy)
set(CMAKE_OBJDUMP       ${toolchain_prefix}objdump)
set(SIZE                ${toolchain_prefix}size)
SET(CMAKE_DEBUGER       ${toolchain_prefix}gdb)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)


###############################################################################
# Common flags

set(COMMON_TOOLCHAIN_CFLAGS "\
    -mthumb -mcpu=cortex-m4 \
    \
    -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
    \
    -mabi=aapcs \
    \
    -fno-builtin \
    -ffast-math \
    -fno-unroll-loops \
    -fdata-sections \
    -ffunction-sections \
    \
    -fomit-frame-pointer \
    -ftree-vectorize \
    -Wall \
")

set(REDEFINED_SYMBOLS
	HAL_MspInit
	NMI_Handler
	HardFault_Handler
	MemManage_Handler
	BusFault_Handler
	UsageFault_Handler
	SVC_Handler
	DebugMon_Handler
	PendSV_Handler
	SysTick_Handler
	DMA1_Channel1_IRQHandler
	EXTI9_5_IRQHandler
	DMA1_Stream5_IRQHandler
	USART2_IRQHandler
	HAL_MspInit
	HAL_I2C_MspInit
	HAL_I2C_MspDeInit
	HAL_TIM_MspPostInit
	HAL_TIM_Base_MspDeInit
	HAL_SuspendTick
	HAL_ResumeTick
	HAL_InitTick
)

list(TRANSFORM REDEFINED_SYMBOLS PREPEND "-u")
string (REPLACE ";" " " REDEFINED_SYMBOLS_FLAGS_STR "${REDEFINED_SYMBOLS}")
set(COMMON_TOOLCHAIN_LFLAGS "\
    ${REDEFINED_SYMBOLS_FLAGS_STR} \
    ${COMMON_TOOLCHAIN_CFLAGS} \
    -Wl,--gc-sections \
    --specs=nosys.specs \
")

SET(CMAKE_C_FLAGS_DEBUG          "-Og -g" CACHE INTERNAL "c compiler flags debug")
SET(CMAKE_CXX_FLAGS_DEBUG        "-Og -g" CACHE INTERNAL "cxx compiler flags debug")
SET(CMAKE_ASM_FLAGS_DEBUG        "-g"     CACHE INTERNAL "asm compiler flags debug")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "-Xlinker -Map=output.map" CACHE INTERNAL "linker flags debug")

SET(CMAKE_C_FLAGS_RELEASE   "-Os -flto -fuse-linker-plugin" CACHE INTERNAL "c compiler flags release")
SET(CMAKE_CXX_FLAGS_RELEASE "-Os -flto -fuse-linker-plugin" CACHE INTERNAL "cxx compiler flags release")
SET(CMAKE_ASM_FLAGS_RELEASE "" CACHE INTERNAL "asm compiler flags release")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "-Xlinker -Map=output.map -s -flto" CACHE INTERNAL "linker flags release")



###############################################################################
# Specific flags (HAL, FreRTOS)

SET(HAL_DIR         "${CMAKE_CURRENT_LIST_DIR}/HAL")
SET(LINKER_SCRIPT   "${HAL_DIR}/stm32f303retx.ld")

set(HAL_CFLAGS "\
    -DUSE_HAL_DRIVER \
    -DSTM32F303xE \
    -DUSE_FULL_LL_DRIVER \
    -D__weak=__attribute__\\(\\(weak\\)\\) \
    -D__packed=__attribute__\\(\\(__packed__\\)\\) \
")
set(HAL_LFLAGS "\
    -T${LINKER_SCRIPT} \
")


# May not work on old openocd versions
set(OPENOCD_CFG board/st_nucleo_f3.cfg)

###############################################################################
# Conclusion


set(CMAKE_C_FLAGS   "${COMMON_TOOLCHAIN_CFLAGS} ${HAL_CFLAGS} -std=gnu99"
    CACHE INTERNAL "c flags")
set(CMAKE_CXX_FLAGS "${COMMON_TOOLCHAIN_CFLAGS} ${HAL_CFLAGS} -std=gnu++11"
    CACHE INTERNAL "c++ flags")

set(CMAKE_ASM_FLAGS
    "-mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Wa,--no-warn -x assembler-with-cpp"
    CACHE INTERNAL "Assembly cflags")

set(CMAKE_EXE_LINKER_FLAGS      "${COMMON_TOOLCHAIN_LFLAGS} ${HAL_LFLAGS}"
    CACHE INTERNAL "executable linker flags")
set(CMAKE_MODULE_LINKER_FLAGS   "${COMMON_TOOLCHAIN_LFLAGS} ${HAL_LFLAGS}"
    CACHE INTERNAL "module linker flags")
set(CMAKE_SHARED_LINKER_FLAGS   "${COMMON_TOOLCHAIN_LFLAGS} ${HAL_LFLAGS}"
    CACHE INTERNAL "shared linker flags")
