#include "mdm_service.h"
#include "cellular.h"
#include "gps.h"
#include "power.h"
#include <string.h>
#include <stdio.h>

uint8_t test_int = 0;
#define CTRL_Z 26

#define MAX_BUF_SIZE 128
char data_read_buff[MAX_BUF_SIZE];
uint8_t bytes_read = 0;

char send_data_buff[MAX_BUF_SIZE];

enum cmd_status curr_status; 
mdm_cb_t curr_cb;
at_cmd_t* curr_cmds;
int cmd_idx = 0;
int num_cmds = 0;


// Modem AT Command definitions
void at_ok() { 
    char* cmd = "AT\r";
    mdm_write(cmd, strlen(cmd)); 
}
void at_echo_off() { 
    char* cmd = "ATE0\r";
    mdm_write(cmd, strlen(cmd)); 
}
void at_cgdcont() { 
    char* fmt = "AT+CGDCONT=%c,\"%s\",\"%s\"\r";
    char s[64];
    sprintf(s, fmt, '1', "IP", mdm_cfg.APN);
    mdm_write(s, strlen(s));
}
void at_sgact() {
    char* cmd = "AT#SGACT=1,1\r";
    mdm_write(cmd, strlen(cmd)); 
}
void at_sd() {
    char* fmt = "AT#SD=1,%d,%d,\"%s\",0,1,1\r";
    char s[64];
    sprintf(s, fmt, mdm_sckt.mode, mdm_sckt.port, mdm_sckt.IP);
    mdm_write(s, strlen(s));
}
void at_ssend() {
    char* cmd = "AT#SSEND=1\r";
    mdm_write(cmd, strlen(cmd));
}
void socket_send() {
    char* fmt = "%s%c";
    char s[MAX_BUF_SIZE+1];
    sprintf(s, fmt, send_data_buff, CTRL_Z);
    mdm_write(s, strlen(s));
}


// Init commands
at_cmd_t init_cmds[3] = {at_echo_off, at_cgdcont, at_sgact};
void mdm_init(mdm_cb_t init_cb) {
    curr_cb = init_cb;
    curr_cmds = init_cmds;
    num_cmds = 3;
    curr_status = AT_IDLE;
}

at_cmd_t open_cmds[1] = {at_sd};
void mdm_open(mdm_socket_t socket, mdm_cb_t open_cb) {
    curr_cb = open_cb;
    mdm_sckt = socket;
    curr_cmds = open_cmds;
    num_cmds = 1;
    curr_status = AT_IDLE;
}

at_cmd_t send_cmds[2] = {at_ssend, socket_send};
void mdm_send(mdm_socket_t socket, char * data, int data_len, mdm_cb_t send_cb) {
    strncpy(send_data_buff, data, data_len);
    mdm_sckt = socket;
    curr_cb = send_cb;
    curr_cmds = send_cmds;
    num_cmds = 2;
    curr_status = AT_IDLE;
}
void mdm_close(mdm_socket_t socket, mdm_cb_t close_cb);
void mdm_status(mdm_socket_t socket, mdm_cb_t status_cb);
void mdm_loc_config(mdm_loc_config_t * config, mdm_cb_t loc_config_cb);
void mdm_loc_start(mdm_cb_t loc_cb);
void mdm_loc_stop(mdm_cb_t loc_cb);

uint8_t mdm_tick() {
    test_int += 1;
    // Main state loop, handle any active command set if there is one, calls the approriate call back on ERROR or FINISHED
    switch (curr_status) {
        case AT_INACTIVE:
            break;
            
        case AT_IDLE:
            curr_cmds[cmd_idx]();
            curr_status = AT_PENDING;

        case AT_PENDING:
            bytes_read += mdm_read(data_read_buff, MAX_BUF_SIZE);
            if(strstr(data_read_buff, "OK")) { 
                curr_status = AT_SUCCESS;
                memset(data_read_buff, 0, MAX_BUF_SIZE);
            }
            if(strstr(data_read_buff, "ERROR")) { 
                curr_status = AT_ERROR;
                memset(data_read_buff, 0, MAX_BUF_SIZE);
            }
            if(strstr(data_read_buff, ">") && curr_cmds[cmd_idx] == at_ssend) { 
                curr_status = AT_SUCCESS;
                memset(data_read_buff, 0, MAX_BUF_SIZE);
            }
            break;

        case AT_SUCCESS:    
            cmd_idx += 1;
            if(cmd_idx == num_cmds) { curr_status = AT_FINISHED; }
            else { curr_status = AT_IDLE; }
            break;

        default:
            (*curr_cb)(curr_status, NULL);
            cmd_idx = 0;
            num_cmds = 0;
            curr_status = AT_INACTIVE;            
            break;
    }
    
    // Return state on each pass, enabling user to use while(!mdm_tick()); to block if desired.
    return curr_status;
}

int test_func() {
    return (test_int + test_cell() + test_gps() + test_power());
}

// UART connection configuration for OSX (unix) system
#ifdef OSX
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

int uart_fd;

int config_serial(char* port) {
    int file_desc;

    // Hold original terminal attributes
    static struct termios original_tty_attrs;
    struct termios options;

    file_desc = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

    if (file_desc == -1) { printf("error opening port\r\n"); return -1; }

    // Now that the device is open, clear the O_NONBLOCK flag so subsequent I/O will block.
    fcntl(file_desc, F_SETFL, 0);

    // Get the current options and save them so we can restore the default settings later.
    if (tcgetattr(file_desc, &original_tty_attrs) == -1) { printf("Error getting tty attributes %s - %s(%d).\n", port, strerror(errno), errno); }

    options = original_tty_attrs;

    // Set baud rates
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    // Enable receiver and set local mode
    options.c_cflag |= (CLOCAL | CREAD);

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10; // 1 second read timeout
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag &= ~CSIZE; /* Mask the character size bits */
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag |= CS8; 

    // Set new options for port
    tcsetattr(file_desc, TCSANOW, &options);

    return file_desc;
}

void mdm_config(mdm_config_t cfg, char* port) {
    mdm_cfg = cfg;
    // TODO: Add error handling in case port can't be opened
    uart_fd = config_serial(port);
}
int mdm_write(char* data, int data_len) {
    return write(uart_fd, data, data_len);
}
int mdm_read(char* data_buf, int data_len) {
    return read(uart_fd, data_buf, data_len);
}
void mdm_start(void) {
    
}
void mdm_stop(void) {
    if(uart_fd) {
        close(uart_fd);
    }
}
#endif