/*
    Author: Will Johns
    Date: May 22nd, 2018
    Description: CPH Modem Module service library
    
*/

#ifndef MDM_SERVICE
#define MDM_SERVICE

#include "cellular.h"
#include "gps.h"
#include "power.h"
#include "mdm_util.h"



typedef struct {
    char ip[15];	// 000.000.000.000
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


void mdm_config(void);
void mdm_start(void);
void mdm_stop(void);
void mdm_init(mdm_cb_t * init_cb);
void mdm_open(mdm_socket_t * socket, mdm_cb_t * received_cb);
void mdm_send(mdm_socket_t * socket, unsigned char * data, int data_len, mdm_cb_t * send_cb);
void mdm_close(mdm_socket_t * socket, mdm_cb_t * close_cb);
void mdm_status(mdm_socket_t * socket, mdm_cb_t * status_cb);
void mdm_loc_config(mdm_loc_config_t * config, mdm_cb_t * loc_config_cb);
void mdm_loc_start(mdm_cb_t * loc_cb);
void mdm_loc_stop(mdm_cb_t * loc_cb);
int test_func();


void mdm_tick(void);

mdm_cb_t curr_func;


#endif