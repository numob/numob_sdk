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

int main(int argc, char *argv[]){
    if(argc !=2) {
        printf("Usage: %s port_name\n",argv[0]);
        printf("The port_name looks like \"/dev/ttyACM0\" in Linux and \"/dev/cu.usbmodem1421\" in macOS\n");
        exit(1);
    }
    std::string port = std::string(argv[1]);

    //connect
    numob::MobileBase mobileBase;
    bool result = mobileBase.connect(port, true);
    if(!result) {
        return  1;
    }
    //config
    mobileBase.setWheelDiameter(0.15);
    mobileBase.setWheelTrack(0.45);
    mobileBase.setWheelEncoderResolution(2200);

    //reset heading offset
    mobileBase.resetHeadingOffset();

    //reset wheels distance
    std::vector<int> wheels = {1,2};
    mobileBase.resetWheelTravelledDistance(wheels);

    //set wheel speed
    std::cout <<"Set wheel speed: " <<std::endl;
    std::map<int,double> wheelAndSpeed;
    wheelAndSpeed[1] = -1.0;
    wheelAndSpeed[2] = 0.5;
    mobileBase.setWheelSpeed(wheelAndSpeed);
    std::cout <<"sleep for a while: " <<std::endl;
    sleep(2);
    //read heading
    std::vector<double> heading = mobileBase.getHeading();
    if(!heading.empty()){
        std::cout << "Heading: " << heading[0] << std::endl;
    }

    //read wheels' travelled distance
    std::vector<int> wheels2 = {1,2};
    std::vector<double> distance = mobileBase.getWheelTravelledDistance(wheels2);
    std::cout << "Distance: " ;
    for(auto it = distance.begin(); it != distance.end(); ++it) {
        std::cout << *it <<",";
    }
    std::cout <<std::endl;

    //read odom
    std::vector<double> odom = mobileBase.getOdometry();
    std::cout << "Odom: " ;
    for(auto it = odom.begin(); it != odom.end(); ++it) {
        std::cout << *it <<",";
    }
    std::cout <<std::endl;


    //set LED
    std::map<int,int> ledSate;
    ledSate[1] = 1;
    ledSate[2] = 1;
    ledSate[3] = 1;
    mobileBase.setLEDState(ledSate);



    //pinMode
    std::map<int,int> pinAndMode = { {2,1}, {3,0}, {4,2}, {5,2}, {6,2},{54, 2}, {55, 2} };
    mobileBase.setPinMode(pinAndMode);

    //digital read
    std::vector<int> digitalReadPins = {2,3,4,5,6,54,55};
    mobileBase.digitalRead(digitalReadPins);

    //digital write (pinMode first, then write)
    std::map<int,int> pinAndMode2 = { {2,1}, {3,1}, {4,1}, {5,1}, {6,1},{54, 1}, {55, 1} };
    mobileBase.setPinMode(pinAndMode2);
    std::map<int,int> pinAndValue = { {2,1}, {3,1}, {4,1}, {5,1}, {6,1},{54, 1}, {55, 1} };
    mobileBase.digitalWrite(pinAndValue);

    //analog read
    std::vector<int> analogReadPins = {54,55};
    mobileBase.analogRead(analogReadPins);

    //pwm write
    std::map<int,int> pwmAndValue =  { {2,100}, {3,0}, {4,255}, {5,255}, {6,0}};
    mobileBase.pwmlWrite(pwmAndValue);

    //more functions
    std::cout << "Wheel Track: " << mobileBase.getWheelTrack() <<std::endl;
    std::cout << "Wheel Diameter: " << mobileBase.getWheelDiameter() <<std::endl;
    std::cout << "Encoder Resolution: " << mobileBase.getWheelEncoderResolution() <<std::endl;

    auto voltage = mobileBase.getBatteryVoltage();
    std::cout << "Voltage: " << voltage.at(0) <<std::endl;

    auto buttons = mobileBase.getButtonState();
    std::cout << "Buttons state: " << buttons.at(0) << "," << buttons.at(1) <<"," <<buttons.at(2) <<std::endl;

    auto leds = mobileBase.getLEDState();
    std::cout << "Led state: " << leds.at(0) << "," << leds.at(1) <<"," <<leds.at(2) <<std::endl;

    auto rangeSensors = mobileBase.getRangeSensor();
    for (auto it = rangeSensors.begin(); it!=rangeSensors.end() ; ++it) {
        std::cout << "Range value: " << *it <<std::endl;
    }


    auto headingAndRotation = mobileBase.getHeadingAndRoationSpeed();
    std::cout << "Heading : " << headingAndRotation.at(0) << ", Rotation: " << headingAndRotation.at(1) << std::endl;


    //read odom again
    odom = mobileBase.getOdometry();
    std::cout << "Odom again: " ;
    for(auto it = odom.begin(); it != odom.end(); ++it) {
        std::cout << *it <<",";
    }
    std::cout <<std::endl;

    //print info
    mobileBase.printDeviceInfo();

    //disconnect
    mobileBase.disConnect();
    return 0;
}