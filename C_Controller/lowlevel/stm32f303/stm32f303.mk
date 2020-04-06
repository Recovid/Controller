Makefile_path := $(shell dirname $(abspath $(lastword $(MAKEFILE_LIST))))

CFlags += \
	-I$(Makefile_path)

Toolchain = arm-none-eabi-

CC = $(Toolchain)gcc
CXX= $(Toolchain)g++
LD = $(Toolchain)ld
GDB= $(Toolchain)gdb

ifeq (, $(shell which ccache))
else
	CC := ccache $(CC)
	CXX:= ccache $(CXX)
endif

# Generic ARM definitions

CFlags += \
	-ffunction-sections \
	-fdata-sections

LFlags += \
	-Wl,--gc-sections \
	-lm \
	-lstdc++_nano \
	-lc \
	-lg \

	# -lrdimon
	# -nostartfiles \


# MCU specific flags (stm32f303)
CFlags += \
	-mcpu=cortex-m4 \
	-mthumb \
	-mfloat-abi=hard \
	-mfpu=fpv4-sp-d16 \
	-specs=nosys.specs


##############################################################################
# Use ST's HAL
HAL_Dir = $(Makefile_path)/HAL

CFlags += \
	-DUSE_HAL_DRIVER \
	-DSTM32F303xE \
	-I$(HAL_Dir) \
	-I$(HAL_Dir)/Drivers/CMSIS/Device/ST/STM32F3xx/Include \
	-I$(HAL_Dir)/Drivers/CMSIS/Include \
	-I$(HAL_Dir)/Drivers/STM32F3xx_HAL_Driver/Inc \

LowLevel_Sources += \
	$(HAL_Dir)/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_cortex.c \
	$(HAL_Dir)/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_dma.c \
	$(HAL_Dir)/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_exti.c \
	$(HAL_Dir)/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash_ex.c \
	$(HAL_Dir)/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash.c \
	$(HAL_Dir)/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_gpio.c \
	$(HAL_Dir)/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c_ex.c \
	$(HAL_Dir)/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c.c \
	$(HAL_Dir)/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr_ex.c \
	$(HAL_Dir)/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr.c \
	$(HAL_Dir)/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc_ex.c \
	$(HAL_Dir)/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc.c \
	$(HAL_Dir)/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim_ex.c \
	$(HAL_Dir)/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim.c \
	$(HAL_Dir)/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart_ex.c \
	$(HAL_Dir)/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart.c \
	$(HAL_Dir)/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal.c \
	$(HAL_Dir)/stm32f3xx_nucleo.c \
	$(HAL_Dir)/syscalls.c \
	$(HAL_Dir)/sysmem.c \
	$(HAL_Dir)/system_stm32f3xx.c \
	$(Makefile_path)/stm32f3xx_hal_msp.c

LowLevel_Objects += \
	$(HAL_Dir)/Drivers/startup_stm32f303retx.o

LowLevel_Sources += \
	$(Makefile_path)/hardware_simulation.c


# Use LibOpenCm3
# LIBOPENCM3_DIR = $(Makefile_path)/hal_common/libopencm3
# CFlags += -I $(LIBOPENCM3_DIR)/include -DSTM32F3
# LFlags += -L $(LIBOPENCM3_DIR)/lib -lopencm3_stm32f3

# libopencm3:
# 	$(MAKE) -C $(LIBOPENCM3_DIR) -j

# Openocd configuration
# OPENOCD_CFG = /usr/share/openocd/scripts/board/st_nucleo_f3.cfg

LINKER_SCRIPTS_DIR = $(HAL_Dir)/
LFlags += -T $(LINKER_SCRIPTS_DIR)/stm32f303retx.ld



##############################################################################
# Utilities (objcopy, flash, debug)

%.hex: %.elf
	@arm-none-eabi-objcopy -Oihex $^ $@
	@echo OBJCOPY $@


#please do not put sudo before openocd, please run
#sudo usermod -a -G uucp <user> for serial monitor
#for udev rule:
#nano /etc/udev/rules.d/70-st-link.rules
#  #NUCLEO ST-LINK/V2.1
#  ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b",TAG+="uaccess"
#to find theses values lsusb
#then udevadm control --reload-rules
#unplug and plug st-link

install_udev:
	@echo "Installing udev rules…"
	@sudo ./install_udev.sh
	@echo "Now, unplug and re-plug the st-link."

%.flash: %.hex
	openocd -f $(OPENOCD_CFG) \
		-c "init" \
		-c "reset init" \
		-c "flash write_image erase $^" \
		-c "reset" \
		-c "shutdown"

#NOTE: the files in the gdb dir must correspond to your MCU
%.debug: %.elf
	$(GDB) $^ --command=gdb/attach.gdb
