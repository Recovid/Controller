#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "common.h"

bool send_DATA(float P, float VolM, float Vol);
bool send_DATA_X(float P, float VolM, float Vol, float Pplat, float PEP);
bool send_RESP(float IE, float FR, float VTe, float VM, float Pcrete, float Pplat, float PEP);

bool send_INIT(const char* information);

bool send_ALRM(uint32_t alarms);

//! Send queued messages to IHM, then receive messages from IHM and process them including:
//! - update controller settings and acknowledge modified value
//! - answer to INIT
//! \returns false if soft reset received
void send_and_recv();

#ifndef NTESTS
bool TEST_PROTOCOL();
#endif

#endif // PROTOCOL
