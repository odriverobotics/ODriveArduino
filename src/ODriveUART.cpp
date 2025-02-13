// Author: ODrive Robotics Inc.
// License: MIT
// Documentation: https://docs.odriverobotics.com/v/latest/guides/arduino-uart-guide.html

#include "Arduino.h"
#include "ODriveUART.h"

// Print with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

ODriveUART::ODriveUART(Stream& serial)
    : serial_(serial) {}

void ODriveUART::clearErrors() {
    serial_ << F("sc\n");
}

void ODriveUART::selectAxis(int axis){
    if(axis >= 0 && axis < 2){
        selected_axis = axis;
    }
}

int ODriveUART::getAxis(){
    return selected_axis;
}

void ODriveUART::setPosition(float position) {
    setPosition(position, 0.0f, 0.0f);
}

void ODriveUART::setPosition(float position, float velocity_feedforward) {
    setPosition(position, velocity_feedforward, 0.0f);
}

void ODriveUART::setPosition(float position, float velocity_feedforward, float torque_feedforward) {
    serial_ << F("p ") << selected_axis  << F(" ") << position << F(" ") << velocity_feedforward << F(" ") << torque_feedforward << F("\n");
}

void ODriveUART::setVelocity(float velocity) {
    setVelocity(velocity, 0.0f);
}

void ODriveUART::setVelocity(float velocity, float torque_feedforward) {
    serial_ << F("v ") << selected_axis  << F(" ") << velocity << F(" ") << torque_feedforward << F("\n");
}

void ODriveUART::setTorque(float torque) {
    serial_ << F("c ") << selected_axis << F(" ") << torque << F("\n");
}

void ODriveUART::trapezoidalMove(float position) {
    serial_ << F("t ") << selected_axis << F(" ") << position << F("\n");
}

ODriveFeedback ODriveUART::getFeedback() {
    // Flush RX
    while (serial_.available()) {
        serial_.read();
    }

    serial_ << F("f ") << selected_axis << F("\n");

    String response = readLine();

    int spacePos = response.indexOf(' ');
    if (spacePos >= 0) {
        return {
            response.substring(0, spacePos).toFloat(),
            response.substring(spacePos+1).toFloat()
        };
    } else {
        return {0.0f, 0.0f};
    }
}

String ODriveUART::getParameterAsString(const String& path) {
    serial_ << F("r ") << path << F("\n");
    return readLine();
}

void ODriveUART::setParameter(const String& path, const String& value) {
    serial_ << F("w ") << path << F(" ") << value << F("\n");
}

void ODriveUART::setState(ODriveAxisState requested_state) {
    String parameter = "axis" + String(selected_axis) + ".requested_state";
    setParameter(parameter, String((long)requested_state));
}

ODriveAxisState ODriveUART::getState() {
    String parameter = "axis" + String(selected_axis) + ".current_state";
    return (ODriveAxisState)getParameterAsInt(parameter);
}

String ODriveUART::readLine(unsigned long timeout_ms) {
    String str = "";
    unsigned long timeout_start = millis();
    for (;;) {
        while (!serial_.available()) {
            if (millis() - timeout_start >= timeout_ms) {
                return str;
            }
        }
        char c = serial_.read();
        if (c == '\n')
            break;
        str += c;
    }
    return str;
}
