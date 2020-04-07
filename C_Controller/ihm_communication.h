#ifndef IHM_COMMUNICATION_H
#define IHM_COMMUNICATION_H

#include "platform.h"

// ------------------------------------------------------------------------------------------------
//! Public interface to send event/data to the IHM and process read messages

bool send_DATA(float P, float VolM, float Vol, float Pplat, float PEP);
bool send_RESP(float IE, float FR, float VTe, float VM, float Pcrete, float Pplat, float PEP);

bool send_INIT(const char* information);

//! Send queued messages to IHM, then receive messages from IHM and process them including:
//! - update controller settings and acknowledge modified value
//! - answer to INIT
//! \returns false if soft reset received
bool send_and_recv();

#endif // IHM_COMMUNICATION_H
