Makefile_path := $(shell dirname $(abspath $(lastword $(MAKEFILE_LIST))))

CFlags += \
	-I$(Makefile_path)


LowLevel_Sources += \
	$(Makefile_path)/hardware_simulation.c


LFlags += \
	-lm
