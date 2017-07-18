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

#include "numob/sdk.h"

numob::MobileBase::MobileBase() {

}

bool numob::MobileBase::connect(std::string port, bool auto_reconnect) {
    _port = port;
    _auto_reconnect = auto_reconnect;
    _is_connecting = true;

    //try to connect several times
    int connect_retry_count = 3;
    for (int i = 0; i < connect_retry_count; i++) {
        _fd = numob_serial_connect(port.c_str());
        if (_fd > -1) {
            std::cout << "Connected to port: " << port <<std::endl;
            break;
        }
        if (i < connect_retry_count - 1) {
            std::cerr << "Try again to connect to port: " << port <<std::endl;
        }

    }
    if (_fd == -1) {
        std::cerr << "Failed to connect to port: " << port <<std::endl;
        _is_connecting = false;
        return false;
    }

    //wait for a while until the connection is ok.
    numob_serial_sleep_milliseconds(1000);

    //test IMU is ok, retry 10 times.
    int max_loop_count = 10;
    int loop_count = 0;
    std::vector<double> result_v; //result of getHeading()
    while (true) {
        loop_count++;
        result_v = getHeading();
        if (!result_v.empty()) {
            break;
        }

        if (loop_count == max_loop_count) {
            break;
        }
    }

    _is_connecting = false;

    if (!result_v.empty()) {
        return true;
    } else {
        std::cerr << "Failed to get initial IMU data, disconnect connection." << std::endl;
        disConnect();
        return false;
    }

}

bool numob::MobileBase::disConnect() {
    numob_serial_disconnect(_fd);
    _fd = -1;
    std::cout << "Disconnect with port: " << _port <<std::endl;
    return true;
}


double numob::MobileBase::getWheelDiameter() {
    std::string register_commands = "2306";
    long result_values[1] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    if (result_code == NUMOB_RESULT_SUCCESS) {
        return double(result_values[0]) / 1000.0; //convert mm to meter
    } else {
        return 0.0;
    }
}

bool numob::MobileBase::setWheelDiameter(double wheelDiameter) {
    long value = long(wheelDiameter * 1000); //convert m to mm
    std::string register_commands = "2306=" + std::to_string(value);
    long result_values[1] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    return result_code == NUMOB_RESULT_SUCCESS;
}

double numob::MobileBase::getWheelTrack() {
    std::string register_commands = "2307";
    long result_values[1] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    if (result_code == NUMOB_RESULT_SUCCESS) {
        return double(result_values[0]) / 1000.0; //convert mm to meter
    } else {
        return 0.0;
    }
}

bool numob::MobileBase::setWheelTrack(double wheelTrack) {
    long value = long(wheelTrack * 1000); //convert m to mm
    std::string register_commands = "2307=" + std::to_string(value);
    long result_values[1] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    return result_code == NUMOB_RESULT_SUCCESS;
}

long numob::MobileBase::getWheelEncoderResolution() {
    std::string register_commands = "2302";
    long result_values[1] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    if (result_code == NUMOB_RESULT_SUCCESS) {
        return result_values[0];
    } else {
        return 0;
    }
}

bool numob::MobileBase::setWheelEncoderResolution(long wheelEncoderResolution) {
    long value = wheelEncoderResolution;
    std::string register_commands = "2302=" + std::to_string(value);
    long result_values[1] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    return result_code == NUMOB_RESULT_SUCCESS;
}





std::vector<double> numob::MobileBase::getWheelTravelledDistance(std::vector<int> wheels) {
    std::vector<double> v_ret;
    //form the commands
    std::string register_commands = "";
    for (std::vector<int>::iterator it = wheels.begin(); it != wheels.end(); ++it) {
        int wheel = *it;
        if (register_commands != "") { //add comma if needed
            register_commands = register_commands + ",";
        }
        if (wheel < 10) {
            register_commands += std::string("190") + std::to_string(wheel);
        } else {
            register_commands += std::string("19") + std::to_string(wheel);
        }
    }


    long result_values[100] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    if (result_code == NUMOB_RESULT_SUCCESS) {
        for (int i = 0; i < wheels.size(); i++) {
            v_ret.push_back((result_values[i]/1000.0)); //convert mm to meter
        }
    }
    return v_ret;



}

bool numob::MobileBase::resetWheelTravelledDistance(std::vector<int> wheels) {

    //form the commands
    std::string register_commands = "";
    for (std::vector<int>::iterator it = wheels.begin(); it != wheels.end(); ++it) {
        int wheel = *it;
        if (register_commands != "") { //add comma if needed
            register_commands = register_commands + ",";
        }
        if (wheel < 10) {
            register_commands += std::string("190") + std::to_string(wheel) + std::string("=") + std::to_string(0);;
        } else {
            register_commands += std::string("19") + std::to_string(wheel) + std::string("=") + std::to_string(0);;
        }
    }

    long result_values[100] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    return result_code == NUMOB_RESULT_SUCCESS;
}


std::vector<double> numob::MobileBase::getHeading() {
    std::vector<double> result_v;
    std::string register_commands = "2406";
    long result_values[1] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    if (result_code == NUMOB_RESULT_SUCCESS) {
        double degree_clockwise = result_values[0] / 1000.0;
        //convert to radians counter-clockwise
        double heading_radian_raw = (360 - degree_clockwise) * PI / 180.0;
        double heading_radian = heading_radian_raw - _imu_Heading_offset; //apply the offset
        result_v.push_back(heading_radian);
    }
    return result_v;
}

bool numob::MobileBase::resetHeadingOffset() {
    std::vector<double> result_v = getHeading();
    if (!result_v.empty()) {
        _imu_Heading_offset = result_v[0];
        return true;
    } else {
        return false;
    }
}

std::vector<double> numob::MobileBase::getHeadingAndRoationSpeed() {
    std::vector<double> result_v;
    std::string register_commands = "2406,2411";
    long result_values[2] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    if (result_code == NUMOB_RESULT_SUCCESS) {
        double degree_clockwise = result_values[0] / 1000.0;
        //convert to radians counter-clockwise
        double heading_radian_raw = (360 - degree_clockwise) * PI / 180.0;
        double heading_radian = heading_radian_raw - _imu_Heading_offset; //apply the offset
        double roation_speed_rps = result_values[1] / 1000.0;
        result_v.push_back(heading_radian);
        result_v.push_back(roation_speed_rps);
    }
    return result_v;
}

std::vector<double> numob::MobileBase::getOdometry() {
    std::vector<double> v_ret;
    //form the commands
    std::string register_commands = "1901,1902,2406,2411,2401";

    long result_values[5] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    if (result_code == NUMOB_RESULT_SUCCESS) {
        //left wheel
        v_ret.push_back((result_values[0]/1000.0)); //left wheel distance, convert mm to meter

        //right wheel
        v_ret.push_back((result_values[1]/1000.0)); //right wheel distance, convert mm to meter

        //heading
        double degree_clockwise = result_values[2] / 1000.0;
        //convert to radians counter-clockwise
        double heading_radian_raw = (360 - degree_clockwise) * PI / 180.0;
        double heading_radian = heading_radian_raw - _imu_Heading_offset; //apply the offset
        v_ret.push_back(heading_radian);

        //rotation speed
        double roation_speed_rps = result_values[3] / 1000.0;
        v_ret.push_back(roation_speed_rps);

        //imu timestamp
        v_ret.push_back(double(result_values[4]));
    }
    return v_ret;

}

std::vector<double> numob::MobileBase::getIMU(){
    std::vector<double> v_ret;
    //form the commands
    std::string register_commands = "2401,2409,2410,2411,2412,2413,2414,2415,2416,2417,2418";

    long result_values[11] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    if (result_code == NUMOB_RESULT_SUCCESS) {
        //left wheel
        v_ret.push_back(double(result_values[0])); //time_stamp
        v_ret.push_back((result_values[1]/1000.0)); //angular_x
        v_ret.push_back((result_values[2]/1000.0)); //angular_y
        v_ret.push_back((result_values[3]/1000.0)); //angular_z
        v_ret.push_back((result_values[4]/1000.0)); //quaternion_w
        v_ret.push_back((result_values[5]/1000.0)); //quaternion_x
        v_ret.push_back((result_values[6]/1000.0)); //quaternion_y
        v_ret.push_back((result_values[7]/1000.0)); //quaternion_z
        v_ret.push_back((result_values[8]/1000.0)); //linear_accel_x
        v_ret.push_back((result_values[9]/1000.0)); //linear_accel_y
        v_ret.push_back((result_values[10]/1000.0)); //linear_accel_z

    }
    return v_ret;
}


bool numob::MobileBase::setWheelSpeed(std::map<int,double> wheelAndSpeed) {

    //form the commands
    std::string register_commands = "";
    for (std::map<int, double>::iterator it = wheelAndSpeed.begin(); it != wheelAndSpeed.end(); ++it) {
        int wheel = it->first;
        long speed = long(it->second * 1000.0); //convert meter to mm
        if (register_commands != "") { //add comma if needed
            register_commands = register_commands + ",";
        }
        if (wheel < 10) {
            register_commands += std::string("180") + std::to_string(wheel) + std::string("=") + std::to_string(speed);
        } else {
            register_commands += std::string("18") + std::to_string(wheel) + std::string("=") + std::to_string(speed);
        }
    }
    long result_values[100] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    return result_code == NUMOB_RESULT_SUCCESS;
}

bool numob::MobileBase::setPinMode(std::map<int, int> pinAndMode) {
    //form the commands
    std::string register_commands = "";
    for (std::map<int, int>::iterator it = pinAndMode.begin(); it != pinAndMode.end(); ++it) {
        int pin = it->first;
        int mode = it->second;
        if (register_commands != "") { //add comma if needed
            register_commands = register_commands + ",";
        }
        if (pin < 10) {
            register_commands += std::string("100") + std::to_string(pin) + std::string("=") + std::to_string(mode);
        } else {
            register_commands += std::string("10") + std::to_string(pin) + std::string("=") + std::to_string(mode);
        }
    }
    long result_values[100] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    return result_code == NUMOB_RESULT_SUCCESS;

}

std::vector<int> numob::MobileBase::digitalRead(std::vector<int> pins) {
    std::vector<int> v_ret;
    //form the commands
    std::string register_commands = "";
    for (std::vector<int>::iterator it = pins.begin(); it != pins.end(); ++it) {
        int pin = *it;
        if (register_commands != "") { //add comma if needed
            register_commands = register_commands + ",";
        }
        if (pin < 10) {
            register_commands += std::string("110") + std::to_string(pin);
        } else {
            register_commands += std::string("11") + std::to_string(pin);
        }
    }


    long result_values[100] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    if (result_code == NUMOB_RESULT_SUCCESS) {
        for (int i = 0; i < pins.size(); i++) {
            v_ret.push_back(int(result_values[i]));
        }
    }
    return v_ret;
}


bool numob::MobileBase::digitalWrite(std::map<int, int> pinAndValue) {
    //form the commands
    std::string register_commands = "";
    for (std::map<int, int>::iterator it = pinAndValue.begin(); it != pinAndValue.end(); ++it) {
        int pin = it->first;
        int value = it->second;
        if (register_commands != "") { //add comma if needed
            register_commands = register_commands + ",";
        }
        if (pin < 10) {
            register_commands += std::string("110") + std::to_string(pin) + std::string("=") + std::to_string(value);
        } else {
            register_commands += std::string("11") + std::to_string(pin) + std::string("=") + std::to_string(value);
        }
    }
    long result_values[100] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    return result_code == NUMOB_RESULT_SUCCESS;
}


std::vector<int> numob::MobileBase::analogRead(std::vector<int> pins) {
    std::vector<int> v_ret;
    //form the commands
    std::string register_commands = "";
    for (std::vector<int>::iterator it = pins.begin(); it != pins.end(); ++it) {
        int pin = *it;
        if (register_commands != "") { //add comma if needed
            register_commands = register_commands + ",";
        }
        if (pin < 10) {
            register_commands += std::string("120") + std::to_string(pin);
        } else {
            register_commands += std::string("12") + std::to_string(pin);
        }
    }


    long result_values[100] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    if (result_code == NUMOB_RESULT_SUCCESS) {
        for (int i = 0; i < pins.size(); i++) {
            v_ret.push_back(int(result_values[i]));
        }
    }
    return v_ret;
}


bool numob::MobileBase::pwmlWrite(std::map<int, int> pinAndValue) {
    //form the commands
    std::string register_commands = "";
    for (std::map<int, int>::iterator it = pinAndValue.begin(); it != pinAndValue.end(); ++it) {
        int pin = it->first;
        int value = it->second;
        if (register_commands != "") { //add comma if needed
            register_commands = register_commands + ",";
        }
        if (pin < 10) {
            register_commands += std::string("130") + std::to_string(pin) + std::string("=") + std::to_string(value);
        } else {
            register_commands += std::string("13") + std::to_string(pin) + std::string("=") + std::to_string(value);
        }
    }
    long result_values[100] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    return result_code == NUMOB_RESULT_SUCCESS;
}

std::vector<double> numob::MobileBase::getRangeSensor() {
    std::vector<double> result_v;
    std::string register_commands = "1401,1402,1403,1404,1405,1406";
    long result_values[6] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    if (result_code == NUMOB_RESULT_SUCCESS) {
        result_v.push_back(result_values[0] / 1000.0); //convert mm to meter
        result_v.push_back(result_values[1] / 1000.0); //convert mm to meter
        result_v.push_back(result_values[2] / 1000.0); //convert mm to meter
        result_v.push_back(result_values[3] / 1000.0); //convert mm to meter
        result_v.push_back(result_values[4] / 1000.0); //convert mm to meter
        result_v.push_back(result_values[5] / 1000.0); //convert mm to meter
    }

    return result_v;
}


std::vector<int> numob::MobileBase::getButtonState() {
    std::vector<int> result_v;
    std::string register_commands = "1501,1502,1503";
    long result_values[3] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    if (result_code == NUMOB_RESULT_SUCCESS) {
        result_v.push_back(int(result_values[0]));
        result_v.push_back(int(result_values[1]));
        result_v.push_back(int(result_values[2]));
    }

    return result_v;
}

std::vector<int> numob::MobileBase::getLEDState() {
    std::vector<int> result_v;
    std::string register_commands = "1601,1602,1603";
    long result_values[3] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    if (result_code == NUMOB_RESULT_SUCCESS) {
        result_v.push_back(int(result_values[0]));
        result_v.push_back(int(result_values[1]));
        result_v.push_back(int(result_values[2]));
    }

    return result_v;
}

bool numob::MobileBase::setLEDState(std::map<int, int> ledAndState) {
    //form the commands
    std::string register_commands = "";
    for (std::map<int, int>::iterator it = ledAndState.begin(); it != ledAndState.end(); ++it) {
        int led = it->first;
        int value = it->second;
        if (register_commands != "") { //add comma if needed
            register_commands = register_commands + ",";
        }
        if (led < 10) {
            register_commands += std::string("160") + std::to_string(led) + std::string("=") + std::to_string(value);
        } else {
            register_commands += std::string("16") + std::to_string(led) + std::string("=") + std::to_string(value);
        }
    }
    long result_values[100] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    return result_code == NUMOB_RESULT_SUCCESS;
}

std::vector<double> numob::MobileBase::getBatteryVoltage() {
    std::vector<double> result_v;
    std::string register_commands = "1701";
    long result_values[1] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    if (result_code == NUMOB_RESULT_SUCCESS) {
        result_v.push_back(result_values[0] / 1000.0); //convert mv to v
    }
    return result_v;
}

void numob::MobileBase::printDeviceInfo() {
    std::cout <<".....Robot information start....." <<std::endl;
    std::string register_commands = "2201,2202,2203,2204,2205,2206,2207,2208,2209,2210";
    long result_values[100] = {0};
    numob_serial_result_t result_code = sendCommand(register_commands, result_values);
    if (result_code == NUMOB_RESULT_SUCCESS) {
        std::cout << "机器人序列号: \t\t\t" << result_values[0] << std::endl;
        std::cout << "机器人型号: \t\t\t\t" << result_values[1] << std::endl;
        std::cout << "机器人版本号: \t\t\t" << result_values[2] << std::endl;
        std::cout << "控制器序列号: \t\t\t" << result_values[3] << std::endl;
        std::cout << "控制器型号: \t\t\t\t" << result_values[4] << std::endl;
        std::cout << "控制器固件版本: \t\t\t" << result_values[5] << std::endl;
        std::cout << "底盘驱动方式: \t\t\t" << result_values[6] << std::endl;
        std::cout << "电机驱动器型号: \t\t\t" << result_values[7] << std::endl;
        std::cout << "电机数量: \t\t\t\t" << result_values[8] << std::endl;
        std::cout << "轮子类型: \t\t\t\t" << result_values[9] << std::endl;
    }

    std::string register_commands2 = "2301,2302,2303,2304,2305,2306,2307,2308,2309";
    result_code = sendCommand(register_commands2, result_values);
    if (result_code == NUMOB_RESULT_SUCCESS) {
        std::cout << "电机自动停止时间（ms): \t" << result_values[0] << std::endl;
        std::cout << "电机编码器分辨率: \t\t\t" << result_values[1] << std::endl;
        std::cout << "电机PID调速的P值: \t\t" << result_values[2] / 1000.0 << std::endl;
        std::cout << "电机PID调速的I值: \t\t" << result_values[3] / 1000.0 << std::endl;
        std::cout << "电机PID调速的D值: \t\t" << result_values[4] / 1000.0 << std::endl;
        std::cout << "轮子直径(m): \t\t\t\t" << result_values[5] / 1000.0 << std::endl;
        std::cout << "轮子间距(m): \t\t\t\t" << result_values[6] / 1000.0 << std::endl;
        std::cout << "遥控下最大速度(m/s): \t\t" << result_values[7] / 1000.0 << std::endl;
        std::cout << "超声波安全距离(m): \t\t" << result_values[8] / 1000.0 << std::endl;
    }
    std::cout <<".....Robot information end......" <<std::endl;
}

numob_serial_result_t numob::MobileBase::sendCommand(std::string register_commands, long result_values[]) {

    //request
    numob_serial_result_t result_code = NUMOB_RESULT_ERROR_UNSPECIFIED;
    _serial_request_id++;
    numob_serial_request(_fd, _serial_request_id, register_commands.c_str(), &result_code, result_values);
    if (result_code != NUMOB_RESULT_SUCCESS) {
        std::cerr << "Failed to execute: #" << _serial_request_id << " " << register_commands << ". Response code: "
                  << result_code <<std::endl;
        _consecutive_error_count++;
    } else {
        _consecutive_error_count = 0; //reset
    }
    if (!_is_connecting && _consecutive_error_count >= _consecutive_error_threshold) {
        std::cout << "Try to re-connect to port: " << _port <<std::endl;
        disConnect();
        numob_serial_sleep_milliseconds(2000);
        connect(_port, _auto_reconnect);
    }
    return result_code;
}





