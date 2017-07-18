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

#include "numob/serial.h"

#ifdef __linux__
#include <signal.h>
#endif

extern int numob_serial_print_raw_message;


int fd = -1;

void sigint(int a)
{
    printf("SIGINT %d is received.\n",a);
    numob_serial_disconnect(fd);
    exit(1);
}



int main(int argc, char *argv[])
{
  signal(SIGINT, sigint);
/*
  //define serial port name
  #ifdef __APPLE__
      char *port="/dev/cu.usbmodem1421"; //change it to your setting
  #else
      char *port = "/dev/ttyACM0";  //change it to your setting
  #endif
*/

//get port from user's input
if(argc !=2) {
  printf("Usage: %s port_name\n",argv[0]);
  printf("The port_name looks like \"/dev/ttyACM0\" in Linux and \"/dev/cu.usbmodem1421\" in macOS\n");
  exit(1);
}
char *port = argv[1];

printf("\n\n*************** Welcome to Numob Serial CLI ***********************************\n");
printf("Usage:\n");
printf("Enter the register commands separated by comma(,)\n");
printf("E.g, to get the firmware version and controller sn, you enter: 2201,2202\n");
printf("Ctrl + c to exit the cli.\n\n");

printf("DO NOT enter the #request_id and CRC as they will be added automatically.\n");
printf("This cli can accept max 100 commands in a single request.\n\n");
printf("*******************************************************************************\n");


//open the serial port
  printf("\nOpenning Port: %s\n", port);
  fd = numob_serial_connect(port);
  if(fd == -1){
      printf("Cant't open port: %s\n", port);
      exit(1);
  } else {
      printf("Port: %s is opened, fd: %d\n", port, fd);
  }

  //enable the message output
  numob_serial_print_raw_message = 1;

  //request related
  long request_id = 1; //identifier each request

  //response related
  numob_serial_result_t result_code = NUMOB_RESULT_ERROR_UNSPECIFIED; //to store the code from response message.
  long result_values[100]= {0}; //to store the return values from the response message. 1000 should be big enough

  while(1) {
    char register_commands[1000] = {'\0'};
    printf( "\n\nEnter Register Commands: ");
    scanf("%s", register_commands);
    printf("\n");
    numob_serial_request(fd,request_id, register_commands, &result_code, result_values);
    request_id ++;
  }

  numob_serial_disconnect(fd);
}
