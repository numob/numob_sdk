/*
 * Copyright (C) 2016, Numob Robot, Inc.
 * Copyright (C) 2016, 青岛诺动机器人有限公司.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Numob Robot, Inc or 青岛诺动机器人有限公司. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "numob/serial.h"

int numob_serial_debug = 0; //set to 1 to print debug infomation.
int numob_serial_print_raw_message = 0; //set to 1 to print the raw request message and response message

int numob_serial_connect(const char *port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd == -1) {
        return -1;
    }

    //set the port
    struct termios options, optionsOld;
    if (tcgetattr(fd, &optionsOld) != 0) {
        printf("\nFaild to get parameters associated with the serial port.");
        return -1;
    }

    bzero(&options, sizeof(optionsOld));
    tcflush(fd, TCIOFLUSH);

    /* ignore modem controls */
    options.c_cflag |= (CLOCAL | CREAD);
    //data size 8 bits
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    //No Parity
    options.c_cflag &= ~PARENB;
    //baud: 19200
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    // 1 stop bit
    options.c_cflag &= ~CSTOPB;
    /* no hardware flowcontrol */
    options.c_cflag &= ~CRTSCTS;

    /* setup for non-canonical mode */
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 1;

    tcflush(fd, TCIOFLUSH);

    tcsetattr(fd, TCSANOW, &options);

    fcntl(fd, F_SETFL, O_NONBLOCK); //set non-blocking

    //sleep for a while
    numob_serial_sleep_milliseconds(1000);
    if (numob_serial_debug) {
        printf("\nSerial port :%s is opened.\n", port);
    }

    return fd;
}


void numob_serial_disconnect(int fd) {
    close(fd);
    if (numob_serial_debug) {
        printf("\nSerial port (fd:%d) is closed.\n", fd);
    }
}

void numob_serial_request(int fd, long request_id, const char *register_commands, numob_serial_result_t *result_code,
                          long result_values[]) {
    //flush the input buffer
    char read_temp[2] = {'\0'};
    while (read(fd, read_temp, 1) > 0) {
        //just loop.
    }

    char request_message[1024] = {'\0'}; //to store the message sent to mobile base controller
    strcat(request_message, "#");
    char request_id_buffer[33] = {'\0'};
    sprintf(request_id_buffer, "%ld", request_id);
    strcat(request_message, request_id_buffer);
    strcat(request_message, " ");
    strcat(request_message, register_commands);

    // crc
    char crc[5] = {'\0'};
    numob_serial_crc_hex(request_message, strlen(request_message), crc);

    strcat(request_message, " ");
    strcat(request_message, crc);
    strcat(request_message, "\r");
    if (numob_serial_debug || numob_serial_print_raw_message) {
        printf("fd: %d, Request  Message: %s\n", fd, request_message);
    }


    //send request
    long sendLen = write(fd, request_message, strlen(request_message));
    if (numob_serial_debug || numob_serial_print_raw_message) {
        printf("send length: %ld\r\n", sendLen);
    }



    //read response
    struct timespec start_time, end_time;
    long duration_mseconds = 0;
    clock_gettime(CLOCK_MONOTONIC, &start_time);

    char readC[2] = {'\0'};
    int index = -1;
    char response_message[1500] = {'\0'};
    long byte_read = 0;
    while (1) {
        //check timer
        clock_gettime(CLOCK_MONOTONIC, &end_time);
        duration_mseconds = numob_serial_elapsed_mseconds(start_time, end_time);
        if (duration_mseconds > NUMOB_SERIAL_READ_TIMEOUT) {
            //printf("Read response timeout.\r\n");
            //printf("current Read str: %s\n",response_message);
            *result_code = NUMOB_RESULT_ERROR_TIMEOUT;
            return;
        }

        byte_read = read(fd, readC, 1);
        if (byte_read < 0) {
            continue;
        }

        if (readC[0] != '\r') { //the RETURN key is not added to readBuffer buffer
            if (index == -1 && readC[0] != '$') { //start flag is not found yet, discard the garbage character
                continue;
            } else { //read character
                response_message[index] = readC[0];
                index++;
            }
            continue;
        }

        //from here , \r is received.

        //start decode the response message.
        if (numob_serial_debug || numob_serial_print_raw_message) {
            printf("Response Message: $%s\n", response_message);
        }

        //decode the response
        char response_request_id[33] = {'\0'};
        char response_code[3] = {'\0'};
        char response_values[1025] = {'\0'};
        char response_crc[5] = {'\0'};


        int arg_index = 0;
        int element_index = 0;
        long len_response_message = strlen(response_message);
        long iterator_index = 0;

        for (; iterator_index < len_response_message; iterator_index++) {
            if (response_message[iterator_index] == ' ') { //blank space is the separator
                arg_index++;
                element_index = 0;
                if (arg_index == 2 && atoi(response_code) != 0) { //error response, no return value
                    arg_index++; //skip the return value field.
                }
            } else { //fill the array

                if (arg_index == 0) { //request_id
                    if (element_index >= 32) {
                        *result_code = NUMOB_RESULT_ERROR_INVALID_RESPONSE_MESSAGE; //response format error
                        return;
                    }
                    response_request_id[element_index] = response_message[iterator_index];
                } else if (arg_index == 1) { //result_code
                    if (element_index >= 2) {
                        *result_code = NUMOB_RESULT_ERROR_INVALID_RESPONSE_MESSAGE; //response format error
                        return;
                    }
                    response_code[element_index] = response_message[iterator_index];
                } else if (arg_index == 2) { //result_value
                    if (element_index >= 1024) {
                        *result_code = NUMOB_RESULT_ERROR_INVALID_RESPONSE_MESSAGE; //response format error
                        return;
                    }
                    response_values[element_index] = response_message[iterator_index];
                } else if (arg_index == 3) { //result_crc
                    if (element_index >= 4) {
                        *result_code = NUMOB_RESULT_ERROR_INVALID_RESPONSE_MESSAGE; //response format error
                        return;
                    }
                    response_crc[element_index] = response_message[iterator_index];
                } else {
                    *result_code = NUMOB_RESULT_ERROR_INVALID_RESPONSE_MESSAGE; //response format error
                    return;
                }
                element_index++;
            }
        }

        //validate the response id. if not match, it should be result from last command. Discard this result and continue read.
        if(atol(response_request_id) != request_id) {
            index = -1;
            memset(response_message, 0, sizeof(response_message));
            continue;
        }


        //validate crc
        char response_message_nocrc[1051] = {'\0'};
        strcat(response_message_nocrc, "$");
        strcat(response_message_nocrc, response_request_id);
        strcat(response_message_nocrc, " ");
        strcat(response_message_nocrc, response_code);
        if (strlen(response_values)) {
            strcat(response_message_nocrc, " ");
            strcat(response_message_nocrc, response_values);
        }
        if (numob_serial_debug) {
            printf("response request_id:%s.\n", response_request_id);
            printf("response code:%s.\n", response_code);
            printf("response values:%s.\n", response_values);
            printf("response crc:%s.\n", response_crc);
            printf("response_message_nocrc:%s.\n", response_message_nocrc);
        }

        char crc_buf[5] = {'\0'};
        numob_serial_crc_hex(response_message_nocrc, strlen(response_message_nocrc), crc_buf);
        //printf("calculated crc:%s.\n",crc_buf);
        if (strcmp(response_crc, crc_buf) == 0) { //crc is ok
            if (atoi(response_code) == 0) { //there is return values
                //split response values to long array
                int arg_index_res = 0;
                int char_index = 0;
                char arg_value[1024] = {'\0'};
                int len_response_values = (int) strlen(response_values);
                for (int i = 0; i < len_response_values; i++) {
                    char c = response_values[i];
                    if (c == ',') {
                        result_values[arg_index_res] = atoi(arg_value);
                        arg_index_res++;
                        char_index = 0;
                        memset(arg_value, 0, sizeof(arg_value));
                        continue;
                    } else {
                        arg_value[char_index] = c;
                        char_index++;
                    }
                }
                //the last one with no comma terminated.
                result_values[arg_index_res] = atoi(arg_value);
                memset(arg_value, 0, sizeof(arg_value));
            }
            *result_code = (numob_serial_result_t) atoi(response_code);
            return;
        } else {
            *result_code = NUMOB_RESULT_ERROR_INVALID_RESPONSE_CRC; //crc error
            printf("CRC Error, response_message:%s. crc should be:%s\n", response_message, crc_buf);
            return;
        }

    } //end while
}

void numob_serial_crc_hex(const char *msg_buf, long len, char *crc_buf) {
    unsigned short crc = 0xFFFF;

    for (int pos = 0; pos < len; pos++) {
        crc ^= (unsigned short) msg_buf[pos]; // XOR byte into least sig. byte of crc

        for (int i = 8; i != 0; i--) {    // Loop over each bit
            if ((crc & 0x0001) != 0) {      // If the LSB is set
                crc >>= 1;                    // Shift right and XOR 0xA001
                crc ^= 0xA001;
            } else                            // Else LSB is not set
                crc >>= 1;                    // Just shift right
        }
    }
    sprintf(crc_buf, "%04X", crc);
}

long numob_serial_elapsed_mseconds(struct timespec start_time, struct timespec end_time) {
    long elapsed_n, seconds, nseconds;
    seconds = end_time.tv_sec - start_time.tv_sec;
    nseconds = end_time.tv_nsec - start_time.tv_nsec;
    elapsed_n = (seconds) * 1000 * 1000 * 1000 + nseconds;
    return elapsed_n / 1000 / 1000;
}

void numob_serial_sleep_milliseconds(long mseconds) {
    useconds_t t = (useconds_t) mseconds * 1000;
    usleep(t);
}
