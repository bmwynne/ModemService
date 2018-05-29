#include "cellular.h"

void cell_tick() {
    if(curr_func) {
        curr_cmd.active_func();
    }
}

int test_cell() {
    return 2;
}