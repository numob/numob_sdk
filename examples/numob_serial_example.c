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

#include "numob_serial.h"

extern int numob_serial_print_raw_message;


//to count the register command number
int commands_count(const char *register_commands){
  int count =1 ;
  for(int i =0 ; i< strlen(register_commands); i ++) {
    char c =register_commands[i];
    if (c == ',') count++;
  }
  return count;
}

int main(int argc, char *argv[])
{

  /*
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

  //open the serial port
  int fd = numob_serial_connect(port);
  if(fd == -1){
      printf("\nCant't open port: %s\n\n", port);
      exit(1);
  } else {
      printf("\nPort: %s is opened, fd: %d\n", port, fd);
  }

  //enable the message output
  numob_serial_print_raw_message = 1;

  //request related
  long request_id = 1000; //identifier each request

  //response related
  numob_serial_result_t result_code = NUMOB_RESULT_ERROR_UNSPECIFIED; //to store the code from response message.
  long result_values[100]= {0}; //to store the return values from the response message. increase the size if 100 is not enough for you.

  //examples. uncomment the command you want to test
  //const char *register_commands = "1022=0,1023=1,1024=1,1025=1"; //set pinmode 22 as INPUT, pin 23 24 25 as OUTPUT
  //const char *register_commands = "1123=0,1124=1,1123,1124"; //digital write pin 23 to 0, pin 24 to 1; Then read pin 23 and pin 24. (Upper pinmode command is required run once)
  const char *register_commands = "2201,2202"; //read firmware version and controller sn

  //send out the request message
  printf("\nRegister Commands: %s.\n\n", register_commands);
  numob_serial_request(fd,request_id, register_commands, &result_code, result_values);

  //decode the response message
  printf("\n----------Start Decode the reponse message-------------\n");
  printf("Request id: %ld\n", request_id);
  printf("Result_code: %d\n", result_code);
  if(result_code == NUMOB_RESULT_SUCCESS){
    printf("Result Values of each register command:\n");
    int count =commands_count(register_commands);
    for (int j =1; j<=count; j++) {
      printf("Command %d: %ld\n",j,result_values[j]);

    }
    printf("\n----------END Decode ------------------------------\n\n\n");
  }


  numob_serial_disconnect(fd);
}
