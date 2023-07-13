// Author: ODrive Robotics Inc.
// License: MIT
// Documentation: https://docs.odriverobotics.com/v/latest/arduino-guide.html

#include "Arduino.h"
#include "ODriveArduino.h"

static const int kMotorNumber = 0;

// Print with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

ODriveArduino::ODriveArduino(Stream& serial)
    : serial_(serial) {}

void ODriveArduino::clearErrors() {
    serial_ << "sc\n";
}

void ODriveArduino::setPosition(float position) {
    setPosition(position, 0.0f, 0.0f);
}

void ODriveArduino::setPosition(float position, float velocity_feedforward) {
    setPosition(position, velocity_feedforward, 0.0f);
}

void ODriveArduino::setPosition(float position, float velocity_feedforward, float torque_feedforward) {
    serial_ << "p " << kMotorNumber  << " " << position << " " << velocity_feedforward << " " << torque_feedforward << "\n";
}

void ODriveArduino::setVelocity(float velocity) {
    setVelocity(velocity, 0.0f);
}

void ODriveArduino::setVelocity(float velocity, float torque_feedforward) {
    serial_ << "v " << kMotorNumber  << " " << velocity << " " << torque_feedforward << "\n";
}

void ODriveArduino::setTorque(float torque) {
    serial_ << "c " << kMotorNumber << " " << torque << "\n";
}

void ODriveArduino::trapezoidalMove(float position) {
    serial_ << "t " << kMotorNumber << " " << position << "\n";
}

ODriveFeedback ODriveArduino::getFeedback() {
    // Flush RX
    while (serial_.available()) {
        serial_.read();
    }

    serial_ << "f " << kMotorNumber << "\n";

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

String ODriveArduino::getParameterAsString(const String& path) {
    serial_ << "r " << path << "\n";
    return readLine();
}

void ODriveArduino::setParameter(const String& path, const String& value) {
    serial_ << "w " << path << " " << value << "\n";
}

void ODriveArduino::setState(ODriveAxisState requested_state) {
    setParameter("axis0.requested_state", String((long)requested_state));
}

ODriveAxisState ODriveArduino::getState() {
    return getParameterAsInt("axis0.current_state");
}

String ODriveArduino::readLine(unsigned long timeout_ms) {
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
