#ifndef VARIABLES_H
#define VARIABLES_H

#define RESPIRATION_CYCLE_RING_SIZE 20
extern int RESP_IDX; //On going respiration
extern int RESP_IDX_TO_SEND;//last RESP cycle over, to send to IHM
extern struct respiration_cycle respiration_cycles_array[RESPIRATION_CYCLE_RING_SIZE]; //Ring buffer of the respiration cycle


#define ALARM_CYCLE_RING_SIZE 20
extern int ALARM_IDX; //The current alarm
extern int ALARM_IDX_TO_ACK; //next alarm needing to be acked

extern struct alarm alarms_array[ALARM_CYCLE_RING_SIZE]; // Ring buffer of the alarm
extern int count_ms;


#endif //VARIABLES_H
