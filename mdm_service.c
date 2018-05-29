#include "mdm_service.h"

void mdm_config(void);
void mdm_start(void) {
    
}
void mdm_stop(void) {

}
void mdm_init(mdm_cb_t * init_cb) {

}
void mdm_open(mdm_socket_t * socket, mdm_cb_t * received_cb);
void mdm_send(mdm_socket_t * socket, unsigned char * data, int data_len, mdm_cb_t * send_cb);
void mdm_close(mdm_socket_t * socket, mdm_cb_t * close_cb);
void mdm_status(mdm_socket_t * socket, mdm_cb_t * status_cb);
void mdm_loc_config(mdm_loc_config_t * config, mdm_cb_t * loc_config_cb);
void mdm_loc_start(mdm_cb_t * loc_cb);
void mdm_loc_stop(mdm_cb_t * loc_cb);

void mod_tick() {
    power_tick();
    cell_tick();
    gps_tick();
}

int test_func() {
    return (test_cell() + test_gps() + test_power());
}