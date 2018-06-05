#include "mdm_service.h"
#include "cellular.h"
#include "gps.h"
#include "power.h"
#include <string.h>

// Modem AT Command definitions
#define AT_OK       "AT\r"
#define AT_ECHO_OFF "ATE0\r"
#define AT_ECHO_ON  "ATE1\r"
#define AT_SELINT   "AT#SELINT=2\r"
#define AT_CMGF     "AT+CMGF=1\r"
#define AT_CGDCONT  "AT+CGDCONT=1,\"IP\",\"a105.2way.net\"\r"
#define AT_SKIPESC  "AT#SKIPESC=1\r"
#define AT_CMEE     "AT+CMEE=2\r"
#define AT_SGACT    "AT#SGACT=1,1\r"

#define MAX_BUF_SIZE 128
char dataBuff[MAX_BUF_SIZE];
uint8_t bytes_read = 0;

enum cmd_status curr_status; 
mdm_cb_t curr_cb;
char** curr_cmds;
int cmd_idx = 0;
int num_cmds = 0;

uint8_t test_int = 0;

// Init commands
char* init_cmds[] = { AT_ECHO_OFF, AT_CGDCONT, AT_SGACT };
void mdm_init(mdm_cb_t init_cb) {
    curr_cb = init_cb;
    curr_cmds = init_cmds;
    num_cmds = 3;
    curr_status = AT_IDLE;
}

void mdm_open(mdm_socket_t * socket, mdm_cb_t received_cb);
void mdm_send(mdm_socket_t * socket, unsigned char * data, int data_len, mdm_cb_t send_cb);
void mdm_close(mdm_socket_t * socket, mdm_cb_t close_cb);
void mdm_status(mdm_socket_t * socket, mdm_cb_t status_cb);
void mdm_loc_config(mdm_loc_config_t * config, mdm_cb_t loc_config_cb);
void mdm_loc_start(mdm_cb_t loc_cb);
void mdm_loc_stop(mdm_cb_t loc_cb);

uint8_t mdm_tick() {
    // Main state loop, handle any active command set if there is one, calls the approriate call back on ERROR or FINISHED
    switch (curr_status) {
        case AT_INACTIVE:
            break;
            
        case AT_IDLE:
            test_int = mdm_write(curr_cmds[cmd_idx], strlen(curr_cmds[cmd_idx]));
            curr_status = AT_PENDING;

        case AT_PENDING:
            bytes_read += mdm_read(dataBuff, MAX_BUF_SIZE);
            if(strstr(dataBuff, "OK")) { 
                curr_status = AT_SUCCESS;
                memset(dataBuff, 0, MAX_BUF_SIZE);
            }
            if(strstr(dataBuff, "ERROR")) { 
                curr_status = AT_ERROR;
                memset(dataBuff, 0, MAX_BUF_SIZE);
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
mdm_config_t mdm_cfg;

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