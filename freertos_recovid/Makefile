ARDUINO_CLI="C:\Users\simeon.marijon\bin\arduino-cli.exe"
BOARD="STM32:stm32:Nucleo_64:pnum=NUCLEO_F411RE"
UPLOAD_METHOD="upload_method=MassStorage"
FLASH_METHOD="upload_method=swdMethod"
SERIAL="COM27"
PUTTY="C:\Program Files\PuTTY\putty.exe" 
SPEED=2000000
PROJECT_NAME=main
INO_FILES=$(PROJECT_NAME)/$(wildcard *.ino)
H_FILES=$(PROJECT_NAME)/$(wildcard *.h)

$(PROJECT_NAME)/$(PROJECT_NAME).STM32.stm32.Nucleo_64.bin: $(INO_FILES) $(H_FILES)
	$(ARDUINO_CLI) compile --fqbn $(BOARD) -p $(SERIAL)  $(PROJECT_NAME) -v

.PHONY:
upload: $(PROJECT_NAME)/$(PROJECT_NAME).STM32.stm32.Nucleo_64.bin
	$(ARDUINO_CLI) upload --fqbn $(BOARD),$(UPLOAD_METHOD) -p $(SERIAL) $(PROJECT_NAME)

flash: $(PROJECT_NAME)/$(PROJECT_NAME).STM32.stm32.Nucleo_64.bin
	$(ARDUINO_CLI) upload --fqbn $(BOARD),$(FLASH_METHOD) -p $(SERIAL) $(PROJECT_NAME)

monitor:
	$(PUTTY) -serial $(SERIAL) -sercfg $(SPEED),8,n,1,N

.PHONY:
clean:  
	rm -f $(PROJECT_NAME)/*.bin
	rm -f $(PROJECT_NAME)/*.hex
	rm -f $(PROJECT_NAME)/*.elf
