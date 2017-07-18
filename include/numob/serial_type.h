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

#ifndef NUMOB_SERIAL_TYPE_H
#define NUMOB_SERIAL_TYPE_H

typedef enum {
    NUMOB_REGISTER_PAGE_PIN_MODE = 10,
    NUMOB_REGISTER_PAGE_DIGITAL = 11,
    NUMOB_REGISTER_PAGE_ANALOG = 12,
    NUMOB_REGISTER_PAGE_PWM = 13,
    NUMOB_REGISTER_PAGE_RANGE = 14,
    NUMOB_REGISTER_PAGE_BUTTON = 15,
    NUMOB_REGISTER_PAGE_LED = 16,
    NUMOB_REGISTER_PAGE_VOLTAGE = 17,
    NUMOB_REGISTER_PAGE_WHEEL_SPEED = 18,
    NUMOB_REGISTER_PAGE_WHEEL_DISTANCE = 19,
    NUMOB_REGISTER_PAGE_MOTOR_SPEED = 20,
    NUMOB_REGISTER_PAGE_MOTOR_ENCODER = 21,
    NUMOB_REGISTER_PAGE_INFO = 22,
    NUMOB_REGISTER_PAGE_CONFIG = 23,
    NUMOB_REGISTER_PAGE_IMU = 24
} numob_serial_register_page_t;

typedef enum {
    NUMOB_RESULT_SUCCESS = 0,
    NUMOB_RESULT_ERROR_TIMEOUT = 10,
    NUMOB_RESULT_ERROR_INVALID_MESSAGE = 11,
    NUMOB_RESULT_ERROR_INVALID_CRC = 12,
    NUMOB_RESULT_ERROR_READ_ONLY = 13,
    NUMOB_RESULT_ERROR_WRITE_ONLY = 14,
    NUMOB_RESULT_ERROR_MISSING_SEQUENCEID = 15,
    NUMOB_RESULT_ERROR_INVALID_REGISTER_ADDRESS = 16,
    NUMOB_RESULT_ERROR_INVALID_REGISTER_VALUE = 17,
    NUMOB_RESULT_ERROR_MOTOR_IN_RC_MODE = 18, //motor is under controlled by rc, commands from serial port will be ignored.
    NUMOB_RESULT_ERROR_INVALID_RESPONSE_MESSAGE = 30,
    NUMOB_RESULT_ERROR_INVALID_RESPONSE_CRC = 31,
    NUMOB_RESULT_ERROR_MOTOR_DRIVER_BUS_BUSY = 40,
    NUMOB_RESULT_ERROR_IMU_BUS_BUSY = 41,
    NUMOB_RESULT_ERROR_UNSPECIFIED = 99
} numob_serial_result_t;

typedef enum {
    NUMOB_PINMODE_INPUT = 0,
    NUMOB_PINMODE_OUTPUT = 1,
    NUMOB_PINMODE_INPUT_PULLUP = 2
} numob_serial_pin_mode_t;

typedef enum {
    NUMOB_MOTOR_DRIVER_MODEL_UNKNOWN = 0,// 0
    NUMOB_MOTOR_DRIVER_MODEL_BDC201 = 1,// Brush DC, 2 EN pin , 1 PWM pin
    NUMOB_MOTOR_DRIVER_MODEL_BLDC48501 = 2,// Brushless DC, RS485 control
} numob_serial_motor_driver_model_t;

typedef enum {
    NUMOB_DRIVE_MECHANISM_UNKNOWN = 0,
    NUMOB_DRIVE_MECHANISM_DIFF_DRIVE = 1
} numob_serial_drive_mechanism_t;

typedef enum {
    NUMOB_WHEEL_TYPE_UNKNOWN = 0,
    NUMOB_WHEEL_TYPE_NORMAL = 1
} numob_serial_wheel_type_t;

typedef enum {
    NUMOB_CONTROLLER_MODEL_UNKNOW = 0,
    NUMOB_CONTROLLER_MODEL_MEGA2560_1 = 1
} numob_serial_controller_model_t;

typedef enum {
    NUMOB_ROBOT_MODEL_UNKNOW = 0,
    NUMOB_ROBOT_MODEL_NR5 = 1
} numob_serial_robot_model_t;
#endif
