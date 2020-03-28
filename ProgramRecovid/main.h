#ifndef CONTROLLER_MAIN_H
#define CONTROLLER_MAIN_H

//Gain to calculate O2Concentration
#define GAIN xxx

//Threshold of the calculation of QPatientBTPS
#define QPLowT -200.0 //QPatient Low Threshold
#define QPHighT 200.0  //QPatient High Threshold

//type of packet to send to the RPi
#define SENDING_PACKET_TYPE_PAW_QPATIENT 0  //packet includes Paw, QPatientSLM
#define SENDING_PACKET_TYPE_FULL 1          //packet includes Paw, QPatientSLM, RawO2 and PAtmo

//Type of packet to receive from the RPi
#define RECEIVED_PACKET_TYPE_SETTINGS 0 //packet indcludes parameters related to the motor
#define RECEIVED_PACKET_TYPE_STOP 1 //packet includes the stop command

#define BAUDRATE 115200   //Transmission speed with the RPi = 115200 bit/s or 115,2 kHz
#define MAX_SENDING_PCKT_LEN 9 //maximum length of a packet to send = 9 bytes
#define MIN_SENDING_PCKT_LEN 5
#define MAX_RECEIVED_PCKT_LEN 8 //maximum length of a packet to receive = 8 bytes

//define states for the electrovalve
#define EV_CLOSE 0
#define EV_OPEN 1


#endif //CONTROLLER_MAIN_H
