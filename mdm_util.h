#ifndef MDM_UTIL_H
#define MDM_UTIL_H

#include <stdint.h>

#define OSX 1

#ifdef OSX

extern int uart_fd;
int mdm_write(char* data, int data_len);
int mdm_read(char* data_buf, int data_len);
#endif

// Callback definition
typedef void (*mdm_cb_t)(uint8_t event, void * object);

// Modem function pointer type
typedef struct {
    mdm_cb_t* cb_func;
    uint32_t timeout;
    void* active_func;
} mdm_cmd_t;

// Modem configuration type
typedef struct {
    char APN[70];
    char IP[16];
} mdm_config_t;

// AT Command States
enum cmd_status {
    AT_INACTIVE = 0,
    AT_IDLE = 1,
    AT_PENDING = 2,
    AT_SUCCESS = 3,
    AT_ERROR = 4,
    AT_TIMEOUT = 5,
    AT_FINISHED = 6
};

#endif