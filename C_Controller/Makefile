Root_Makefile_path := $(shell dirname $(abspath $(lastword $(MAKEFILE_LIST))))

default: all

###############################################################################
# Variables

# Override this target to build for native
Target := stm32f303
# Target = native

Buildtype := debug # release

Builddir := _build

###############################################################################
# Sources

Controller_Sources = \
	main.c \
	controller.c \
	ihm_communication.c \


###############################################################################
# Generic definitions

# CFlags are common to C and C++
CFlags =
CXXFlags =
LFlags =

# Warning levels
CFlags += \
	-Wall \
	-Wextra \
	-Wno-unused-parameter \
	#-werror

# Generic flags
CFlags += \
	-std=gnu11 \
	-fdiagnostics-color=always \
	-Os \
	-g

CXXFlags += \
	-std=gnu++14


# Dependencies on headers
CFlags += -MMD -MP


# Generic rules

$(Builddir)/%.o: %.cpp Makefile
	@echo CXX $<
	@$(CXX) $(CFlags) $(CXXFlags) $< -o $@ -c

$(Builddir)/%.o: %.c Makefile
	@echo CC $<
	@$(CC) $(CFlags) $< -o $@ -c

$(Builddir)/%.o: %.s Makefile
	@$(CC) $(CFlags) $< -o $@ -c -x assembler-with-cpp

%.elf:
	@echo LINK $@
	@$(CC) $(CFlags) $^ $(LFlags) -o $@


###############################################################################
# Target-specific definitions

CFlags += -I$(Root_Makefile_path)/lowlevel/include

LowLevel_Sources =
LowLevel_Objects =
-include $(Root_Makefile_path)/lowlevel/$(Target)/$(Target).mk

###############################################################################
# Define object files and executables after everything

LowLevel_RelSrcs = $(shell realpath --relative-to $(Root_Makefile_path) $(LowLevel_Sources))
LowLevel_Objects+= $(patsubst %.c,$(Builddir)/%.o,$(LowLevel_RelSrcs))
LowLevel_Depends = $(patsubst %.c,$(Builddir)/%.d,$(LowLevel_RelSrcs))

Controller_RelSrcs = $(shell realpath --relative-to $(Root_Makefile_path) $(Controller_Sources))
Controller_Objects = $(patsubst %.c,$(Builddir)/%.o,$(Controller_RelSrcs))
Controller_Depends = $(patsubst %.c,$(Builddir)/%.d,$(Controller_RelSrcs))

All_Objects = $(LowLevel_Objects) $(Controller_Objects)
All_Depends = $(LowLevel_Depends) $(Controller_Depends)

-include $(LowLevel_Depends)

# Mkdir
all_bin_dirs=$(sort $(dir $(All_Objects)))
mkdir:
	@echo Creating output directories
	@mkdir -p $(all_bin_dirs)

Controller.elf: $(Controller_Objects) $(LowLevel_Objects)

print:
	echo $(LowLevel_Sources)

all: Controller.elf


clean:
		find . \
		\( -not -path  "./hal_common*" \) \
		\( -name "*.o" \
		-o -name "*.a" \
		-o -name "*.d" \
		-o -name "*.hex" \
		-o -name "*.elf" \
		\) -delete
