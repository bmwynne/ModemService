/*
    Author: Will Johns
    Date: May 22nd, 2018
    Description: CPH Modem Module service library
    
*/

#ifndef MDM_SERVICE
#define MDM_SERVICE

#include <stdint.h>

#define MDM_SOCKET_MODE_UDP 1
#define MDM_SOCKET_MODE_TCP 0

#define OSX 1
#define AUTO_RECCONECT 1

#ifdef OSX

extern int uart_fd;
int mdm_write(char* data, int data_len);
int mdm_read(char* data_buf, int data_len);
#endif

// Callback definition
typedef void (*mdm_cb_t)(uint8_t event, void * object);

// AT Command function type
typedef void (*at_cmd_t)(void);

// Modem function pointer type
typedef struct {
    mdm_cb_t* cb_func;
    uint32_t timeout;
    void* active_func;
} mdm_cmd_t;

// Modem configuration type
typedef struct {
    char APN[70];
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

typedef struct {
    char IP[15];	// 000.000.000.000
    int port;	// 10700
    char mode;
    int connection_id;
} mdm_socket_t;

typedef struct {
    int length;
    char * data;
    mdm_socket_t * socket;
} mdm_packet_t;

typedef struct {
    int cell_strength;
    int connection_status;
} mdm_status_t;

typedef struct { 
    int accuracy;
    int num_satellites;
    int interval;
    int num_polls;
} mdm_loc_config_t;

typedef struct { 
    int utc_time;
    double latitude;
    double longitude;
} mdm_loc_result_t;

typedef struct {
    int sock_id;
    char bound_ip[15];
    int bound_port;
    char conn_ip[15];
    int conn_port;
    int sent_bytes;
    int sent_pnding;
    int rcvd_bytes;
    int rcvd_pnding;
    int state;
} socket_status;

// // 
// Utc_Time
// Latitude
// Longitude
// HDOP
// Altitude
// Fix
// Course
// Speed_Kilometers
// Speed_Knots
// Date_of_Fix
// Num_Satellites

#ifdef OSX
void mdm_config(mdm_config_t cfg, char* port);
#else
void mdm_config(void);
#endif

void mdm_start(void);
void mdm_stop(void);
void mdm_init(mdm_cb_t init_cb);
void mdm_open(mdm_socket_t socket, mdm_cb_t open_cb);
void mdm_send(mdm_socket_t socket, char * data, int data_len, mdm_cb_t send_cb);
uint8_t mdm_data_rdy();
int mdm_recv(char * data, int max_data_len);
void mdm_close(mdm_socket_t socket, mdm_cb_t close_cb);
void mdm_status(mdm_socket_t socket, mdm_cb_t status_cb);
void mdm_loc_config(mdm_loc_config_t * config, mdm_cb_t loc_config_cb);
void mdm_loc_start(mdm_cb_t loc_cb);
void mdm_loc_stop(mdm_cb_t loc_cb);
int test_func();


uint8_t mdm_tick(void);


#endif