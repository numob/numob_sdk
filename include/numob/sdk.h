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

#ifndef NUMOB_SDK_H
#define NUMOB_SDK_H

extern "C" {
#include "numob/serial.h"
}

#include <vector>
#include <sstream>
#include <iostream>
#include <cmath>
#include <map>

#ifndef PI
#define PI 3.14159
#endif

namespace numob {
    class MobileBase {
    public:
        /**
         * Constructor. Using default configuration of the mobile base
         */
        MobileBase();

        /**
         * Open the serial port to mobile base.
         * After connected, it will check the IMU
         * @param port std::string the port name, such as /dev/ttyACM0, /dev/numob.
         * @param auto_reconnect bool reconnect to mobile base if it is set to true and _consecutive_error_count reaches _consecutive_error_threshold
         * @return bool true: success (IMU is also ok), false: fail.
         */
        bool connect(std::string port, bool auto_reconnect);

        /**
         * Close the serial port to mobile base.
         *
         * @return bool true: success, false: fail.
         */
        bool disConnect();

        /**
         * Get the wheel diameter (meter).
         *
         * @return double wheel diameter in meter. return 0.0 if fails.
         */
        double getWheelDiameter();

        /**
         * Set the wheel diameter (meter).
         *
         * @param wheelDiameter double the diameter of the wheel
         * @return bool true: success, false: fail.
         */
        bool setWheelDiameter(double wheelDiameter);

        /**
         * Get the wheel track (meter, The distance between the center of two wheels.).
         *
         * @return double wheel track in meter. return 0.0 if fails.
         */
        double getWheelTrack();

        /**
         * Set the wheel track (meter, The distance between the center of two wheels.).
         *
         * @param wheelTrack double the wheel track
         * @return bool true: success, false: fail.
         */
        bool setWheelTrack(double wheelTrack);

        /**
         * Get the wheel encoder resolution (ticks per revolution),  (of the wheel, after the motor gear reduction).
         *
         * @return long wheel encoder resolution. return 0 if fails.
         */
        long getWheelEncoderResolution();

        /**
         * Set the wheel encoder resolution (ticks per revolution),  (of the wheel, after the motor gear reduction).
         *
         * @param wheelEncoderResolution long wheel encoder resolution (ticks per resolution)
         * @return bool true: success, false: fail.
         */
        bool setWheelEncoderResolution(long wheelEncoderResolution);



        /**
         * Get the total travelled distance(meter) of wheels.
         *
         * @param wheels std::vector<int> wheel location list,  range: 1-4. For diff drive robot, 1 (left wheel), 2(right wheel)
         * @return std::vector<double> a vector holding wheels travelled distance in meter. return empty vector if fail.
         */
        std::vector<double> getWheelTravelledDistance(std::vector<int> wheels);

        /**
         * Reset the total travelled distance(meter) of wheels to Zero(0). (Reset motor encoder to 0).
         *
         * @param wheels std::vector<int> wheel location list,  range: 1-4. For diff drive robot, 1 (left wheel), 2(right wheel)
         * @return bool true: success, false: fail.
         */
        bool resetWheelTravelledDistance(std::vector<int> wheels);


        /**
         * Get the IMU Heading (radians counter-clockwise).
         *
         * @return std::vector<double> index 0: heading radians counter-clockwise. Return empty vector if fails
         */
        std::vector<double> getHeading();

        /**
         * Reset the IMU Heading offset to current heading value. As a result, the calculated heading is set to Zero(0).
         * This is usually done before starting slam or navigation. Or after first connect();
         * @return bool true: success, false: fail.
         */
        bool resetHeadingOffset();

        /**
         * Get the IMU Heading (radians counter-clockwise) and rotation speed(radians/s, counter clockwise ).
         *
         * @return std::vector<double> index 0: heading, index 1: roation speed
         */
        std::vector<double> getHeadingAndRoationSpeed();


        /**
         * Get the total travelled distance(meter) of wheel1, wheel2, imu heading and rotation speed.
         *
         * @return std::vector<double> a vector holding wheels travelled distance of left wheel(index 0),rightWheel(index 1), heading(index 2), rotation speed(index3), imu timestamp(index 4). return empty vector if fail.
         */
        std::vector<double> getOdometry();

        /**
         * Get the IMU sensor data.
         *
         * @return std::vector<double> a vector holding IMU(time_stamp, angular x,y,z quaternion w,x,y,z linear_accel x,y,z ) . return empty vector if fail.
         */
        std::vector<double> getIMU();

        /**
         * Set speed for wheel.
         *
         * @param wheelAndSpeed std::map<int,double> wheel location and speed pair. Wheel location range: 1-4, speed unit: m/s.
         *                      For diff drive robot, location 1: left, location 2: right
         *
         * @return bool true: success, false: fail.
         */
        bool setWheelSpeed(std::map<int,double> wheelAndSpeed);

        /**
         * Set pin mode.
         *
         * @param pinAndMode std::map<int,int> pin and mode pair. Mode value range 0-3: 0(INPOUT), 1(OUTPUT), 2(INPUT_PULLUP)
         *
         * @return bool true: success, false: fail.
         */
        bool setPinMode(std::map<int,int> pinAndMode);


        /**
         * Digital Read.
         *
         * @param pins std::vector<int> pin vector
         * @return std::vector<int> The value of each pin, 0: HIGH 1:LOW
         */
        std::vector<int> digitalRead(std::vector<int> pins);

        /**
         * Digital Write.
         *
         * @param pinAndValue std::map<int,int> pin and value pair. Value range 0-1
         * @return bool true: success, false: fail.
         */
        bool digitalWrite(std::map<int,int> pinAndValue);

        /**
         * Analog Read.
         *
         * @param pins std::vector<int> pin vector
         * @return std::vector<int> The value of each pin. Value range 0-1023
         */
        std::vector<int> analogRead(std::vector<int> pins);

        /**
         * PWM Write.
         *
         * @param pinAndValue std::map<int,int> pin and value pair. Value range 0-255
         * @return bool true: success, false: fail.
         */
        bool pwmlWrite(std::map<int,int> pinAndValue);

        /**
         * Get the ultrasonic range sensor value (meter).
         *
         * @return std::vector<double> a vector holding 6 sensor's distance in meter. value 0.0 (meter) means data is invalid(too close or out of range).
         *         return empty vector if fail.
         */
        std::vector<double> getRangeSensor();

        /**
         * Get the Button state.
         *
         * @return std::vector<int> a vector holding 3 buttons' state.  value range 0-1, 0:OFF, 1:ON. Button index 0:RED, 1:Green 2:RC
         *         return empty vector if fail.
         */
        std::vector<int> getButtonState();

        /**
         * Get the LED state.
         *
         * @return std::vector<int> a vector holding 3 LED' state.  value range 0-1, 0:OFF, 1:ON. LED index 0:RED, 1:Yellow 2:Green
         *         return empty vector if fail.
         */
        std::vector<int> getLEDState();

        /**
         * Set LED state
         *
         * @param ledAndState std::map<int,int> led location and value pair. led location: 0(red), 1(yellow) 2(green), Value 0(OFF), 1(ON)
         * @return bool true: success, false: fail.
         */
        bool setLEDState(std::map<int,int> ledAndState);


        /**
         * Get the Battery Voltage.
         *
         * @return std::vector<double> a vector holding 1 value with battery's voltage.
         *         return empty vector if fail.
         */
        std::vector<double> getBatteryVoltage();

        /**
         * Print the mobile base info and configuration
         *
         *
         * @return void
         */
        void printDeviceInfo();

    private:

        /**
         * Calculate CRC and serial number, then send command to mobile base.
         * if _consecutive_error_count reach threshold and _auto_reconnect is set to true , it will reconnect the port.
         *
         * @param register_commands std::string register string without CRC and serial number
         * @param result_values long[] store the return value of the command.
         * @return numob_serial_result_t return NUMOB_RESULT_SUCCESS if success, or other values if fail.
         */
        numob_serial_result_t sendCommand(std::string register_commands, long result_values[]); //send serial command to mobile base

        std::string _port; //the serial port name
        bool _auto_reconnect;  //whether auto reconnect to mobile base if _consecutive_error_threshold is reached.
        bool _is_connecting; //a flag to mark if in the connecting process.
        int _fd = -1; //file descriptor of the serial port connection
        long _serial_request_id = 0; //serial serial number, increased by 1 for each command
        double _imu_Heading_offset = 0.0; //the original heading data
        int _consecutive_error_count = 0; //increase by 1 for consecutively failed sendCommand(). Reset to 0 once a sendCommand() succeed.
        int _consecutive_error_threshold = 10; //if _consecutive_error_count reach this value and _auto_reconnect is set to true, reconnect to mobile base

    };
} //end namespace

#endif
