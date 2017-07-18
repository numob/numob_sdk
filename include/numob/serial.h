/*
 * Author: Peijun Zhao <z@numob.com>
 *
 * Copyright (c) 2016 Numob Robot, Inc (青岛诺动机器人有限公司 www.numob.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef NUMOB_SERIAL_H
#define NUMOB_SERIAL_H

#define NUMOB_SERIAL_READ_TIMEOUT  50  //ms

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "numob/serial_type.h"

/**
 * open the serial port
 *
 * @param port the port name, such as (/dev/ttyACM0)
 * @return int file descriptor  -1(fail), other:(success).
 */
int numob_serial_connect(const char *port);

/**
 * close the serial port
 *
 * @param fd the the file descriptor returned from openning the serial port.
 * @return void
 */
void numob_serial_disconnect(int fd);

 /**
  * send request message
  *
  * @param fd int file descriptor of the port
  * @param request_id long to identify each request
  * @param register_commands the command string. Please refer to numob_serial_protocol.md for details.
  * @param result_code numob_serial_result_t 0:success other: error
  * @param result_values if the reponse code is NUMOB_RESULT_SUCCESS (0), this array stores the returned values of each register command
  * @return void
  */
void numob_serial_request(int fd, long request_id, const char *register_commands, numob_serial_result_t *result_code, long result_values[]);


/**
 * generate crc for the request message.
 *
 * @param msg_buf the input message to calculate the crc
 * @param len the length of msg_buf
 * @param crc_buf to store the generated crc
 * @return void
 */
void numob_serial_crc_hex(const char *msg_buf, long len, char *crc_buf);

/**
 * helper function to calculate the elapsed time in milliseconds
 *
 * @param start_time
 * @param end_time
 * @return long the elapsed milliseconds
 */
long numob_serial_elapsed_mseconds(struct timespec start_time, struct timespec end_time);

/**
 * helper function to sleep milliseconds
 *
 * @param mseconds milliseconds to sleep
 * @return void
 */
void numob_serial_sleep_milliseconds(long mseconds);


#endif
