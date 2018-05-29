/*
    Author: Will Johns
    Date: May 22nd, 2018
    Description: Driver file for the cellular connection on Telit ME910 for use with CPH Modem Module service
    
*/
#ifndef CELL_H
#define CELL_H

#include "mdm_util.h"

void cell_tick();
int test_cell();

mdm_cmd_t curr_cmd = NULL;
#endif