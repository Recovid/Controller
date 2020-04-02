# Requirements

- Install Arduino environnements
- Follow this tutorial to add stm32 support (Beware it will download a arm toolchain this could be big (1G))
- Install STM32FreeRTOS lib using the library manager of Arduino (https://www.arduino.cc/en/guide/libraries). It will directly download the need files
- Open main.ino 
- Verify and upload

## Optional
- You may want to use Makefile : our makefile depends on arduino-cli (https://github.com/arduino/arduino-cli)
- Beware that the makefile target my board for now and not the project one. As soon as board will be available I will change that

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

