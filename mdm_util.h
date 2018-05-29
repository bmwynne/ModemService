#ifndef MDM_UTIL_H
#define MDM_UTIL_H

#include <stdint.h>

// Callback definition
typedef void (*mdm_cb_t)(char event, void * object);

// Modem function pointer type
typedef struct {
    mdm_cb_t* cb_func;
    uint32_t timeout;
    void* active_func;
} mdm_cmd_t;

#endif