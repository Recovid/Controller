#ifndef IHM_COMMUNICATION_H
#define IHM_COMMUNICATION_H

#include <stdbool.h>

// ------------------------------------------------------------------------------------------------
//! Public interface to send event/data to the IHM and process read messages

bool send_DATA(int P, int VolM, int Vol, int Pplat, int PEP);
bool send_RESP(int IE, int FR, int VTe, int VM, int Pcrete, int Pplat, int PEP);

bool send_INIT(const char* information);

//! Send queued messages to IHM, then receive messages from IHM and process them including:
//! - update controller settings and acknowledge modified value
//! - answer to INIT
//! \returns false if soft reset received
bool send_and_recv();

#endif // IHM_COMMUNICATION_H
