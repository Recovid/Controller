Root_Makefile_path := $(shell dirname $(abspath $(lastword $(MAKEFILE_LIST))))

default: all
.SECONDEXPANSION:

###############################################################################
# Variables

# Override this target to build for native
Target := stm32f303
# Target = native

Buildtype := debug # release

Builddir := obj/$(Target)

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

$(Builddir)/%.cpp.o: %.cpp | $$(@D)/.
	@echo CXX $<
	@$(CXX) $(CFlags) $(CXXFlags) $< -o $@ -c

$(Builddir)/%.c.o: %.c | $$(@D)/.
	@echo CC $<
	@$(CC) $(CFlags) $< -o $@ -c

$(Builddir)/%.s.o: %.s | $$(@D)/.
	@echo CC $<
	@$(CC) $(CFlags) $< -o $@ -c

%.elf:
	@echo LINK $@
	@$(CC) $(CFlags) $^ $(LFlags) -o $@


###############################################################################
# Target-specific definitions

CFlags += -I$(Root_Makefile_path)/lowlevel/include

-include $(Root_Makefile_path)/lowlevel/$(Target)/$(Target).mk

###############################################################################
# Define object files and executables after everything

LowLevel_RelSrcs = $(shell realpath --relative-to $(Root_Makefile_path) $(LowLevel_Sources))
LowLevel_Objects = $(patsubst %,$(Builddir)/%.o,$(LowLevel_RelSrcs))
LowLevel_Depends = $(patsubst %,$(Builddir)/%.d,$(LowLevel_RelSrcs))

Controller_RelSrcs = $(shell realpath --relative-to $(Root_Makefile_path) $(Controller_Sources))
Controller_Objects = $(patsubst %,$(Builddir)/%.o,$(Controller_RelSrcs))
Controller_Depends = $(patsubst %,$(Builddir)/%.d,$(Controller_RelSrcs))

All_Objects = $(LowLevel_Objects) $(Controller_Objects)
All_Depends = $(LowLevel_Depends) $(Controller_Depends)

$(All_Objects): Makefile

-include $(LowLevel_Depends)

.PRECIOUS: $(Builddir)/. $(Builddir)%/.
$(Builddir)/.:  ; mkdir -p $@
$(Builddir)%/.: ; mkdir -p $@


Controller.$(Target).elf: $(Controller_Objects) $(LowLevel_Objects)


all: Controller.$(Target).elf


clean:
		rm -rf $(Builddir)
