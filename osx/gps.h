/*
    Author: Will Johns
    Date: May 22nd, 2018
    Description: Driver file for the GPS connection on Telit ME910 for use with CPH Modem Module service
    
*/
#ifndef GPS_H
#define GPS_H

#include "mdm_util.h"

void gps_tick();
int test_gps();
#endif