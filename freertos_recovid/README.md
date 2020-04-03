# Requirements

- Install Arduino environnements
- Follow this tutorial to add stm32 support (Beware it will download a arm toolchain this could be big (1G))
- Install STM32FreeRTOS lib using the library manager (Sketch -> Include Library -> Manage Librayries) of Arduino (https://www.arduino.cc/en/guide/libraries).
  It will directly download the need files
- Open main.ino
- Select your board settings on Tools :
	- Board -> Nucleo 64
	- Board part number -> THE_CORRECT_BOARD_PART_NUMBER (Should be F303XX)
	- Port -> Choose the Serial Port (This should be done automatically)
	- Upload Method -> MassStorage (for non persistent storage)
- Verify and upload

## Optional
### Makefile usage
	You may want to use Makefile : our makefile depends on arduino-cli (https://github.com/arduino/arduino-cli)
	Beware that the makefile target my board for now and not the project one. As soon as board will be available I will change that

### Persistent flash
	You need to use the STM32 programmer (https://my.st.com/content/my_st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-programmers/stm32cubeprog.html#get-software)
	It could be integrated within the arduino environement if you modify your PATH to include the STM32 bin directory.
	After settings your PATH correctly, you can click on Tools->Upload Method->SWD to flash the board parmanently.
	You may need to update the STlink firmware (the STlink is the upper part of the Nucleo board with the USB port). 
	Follwing this link you'll find the updater (https://my.st.com/content/my_st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-programmers/stsw-link007.license=1585859380312.product=STSW-LINK007.version=2.36.26.html#get-software). 

	A makefile target is also ready to flash your board if you use arduino-cli

# TODO:

Fill the code of each tasks :
- void TaskMediumAlarm(void* task); 
- void TaskUrgentAlarm(void* task); 
- void TaskSensing(void* task);
- void TaskMessageManagement(void* task);
- void TaskRespirationCycle(void* task);
- void TaskDataSending(void* task);
- void TaskAlarmSending(void* task);

Based on the "state machine" available here https://minmaxmedical.sharepoint.com/sites/Team-RECOVID/Documents%20partages/LOT08_09_10_Controller_IHM_Arnaud_MMM_soft/Controler_state_machine.dia

