#ifndef __XXXX_H__
#define __XXXX_H__

//********************************************************************************
// Public dependencies                                                                      
// Only the strict minimum of dependencies MUST be included                    
// "common.h" provides all common project dependencies and should be sufficient. 
//********************************************************************************
#include "common.h"

//********************************************************************************
// Public defines and types
//********************************************************************************

#define MAX_PAYLOAD_LEN     (25)

typedef struct {
  uint8_t  command_id;
  uint16_t payload_len;
} message_header_t;


//********************************************************************************
// Public variables
//********************************************************************************

extern uint32_t g_my_public_counter;

//********************************************************************************
// Public functions
//********************************************************************************

void my_public_function(bool activate);

#endif