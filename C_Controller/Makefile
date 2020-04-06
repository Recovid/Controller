default: all

###############################################################################
# Variables

# Override this target to build for native
Target := stm32f303
# Target = native

Buildtype := debug # release

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

%.o: %.cpp Makefile
	@echo CXX $<
	@$(CXX) $(CFlags) $(CXXFlags) $< -o $@ -c

%.o: %.c Makefile
	@echo CC $<
	@$(CC) $(CFlags) $< -o $@ -c

%.o: %.s Makefile
	@$(CC) $(CFlags) $< -o $@ -c -x assembler-with-cpp

%.elf:
	@echo LINK $@
	@$(CC) $(CFlags) $^ $(LFlags) -o $@


###############################################################################
# Target-specific definitions

CFlags += -Ilowlevel/include

LowLevel_Sources =
LowLevel_Objects =
-include lowlevel/$(Target)/$(Target).mk

###############################################################################
# Define object files and executables after everything

LowLevel_Objects+= $(patsubst %.c,%.o,$(LowLevel_Sources))
LowLevel_Depends = $(patsubst %.c,%.d,$(LowLevel_Sources))
-include $(LowLevel_Depends)

Controller_Objects = $(patsubst %.c,%.o,$(Controller_Sources))
Controller_Depends = $(patsubst %.c,%.d,$(Controller_Sources))
-include $(Controller_Depends)

# Mkdir
$(foreach dir,$(sort $(dir $(OBJS))),$(eval $(call define_mkdir_target,$(dir))))


Controller.elf: $(Controller_Objects) $(LowLevel_Objects)


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
