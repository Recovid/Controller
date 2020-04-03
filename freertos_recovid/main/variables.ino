#define RESPIRATION_CYCLE_RING_SIZE 20
int RESP_IDX; //On going respiration
int RESP_IDX_TO_SEND;//last RESP cycle over, to send to IHM
struct respiration_cycle respiration_cycles_array[RESPIRATION_CYCLE_RING_SIZE]; //Ring buffer of the respiration cycle


#define ALARM_CYCLE_RING_SIZE 20
int ALARM_IDX; //The current alarm
int ALARM_IDX_TO_ACK; //next alarm needing to be acked

struct alarm alarms_array[ALARM_CYCLE_RING_SIZE]; // Ring buffer of the alarm

int count_ms = 500;
QueueHandle_t xQueueMessage;
