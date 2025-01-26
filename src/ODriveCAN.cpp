// Author: ODrive Robotics Inc.
// License: MIT
// Documentation: https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html

#include "ODriveCAN.h"

#include <Arduino.h> // needed for debug printing

bool ODriveCAN::clearErrors() {
    Clear_Errors_msg_t msg;
    return send(msg);
}

bool ODriveCAN::setPosition(float position, float velocity_feedforward, float torque_feedforward) {
    Set_Input_Pos_msg_t msg;

    msg.Input_Pos = position;
    msg.Vel_FF = velocity_feedforward;
    msg.Torque_FF = torque_feedforward;

    return send(msg);
}

bool ODriveCAN::setVelocity(float velocity, float torque_feedforward) {
    Set_Input_Vel_msg_t msg;

    msg.Input_Vel = velocity;
    msg.Input_Torque_FF = torque_feedforward;

    return send(msg);
}

bool ODriveCAN::setControllerMode(uint8_t control_mode, uint8_t input_mode) {
    Set_Controller_Mode_msg_t msg;

    msg.Control_Mode = control_mode;
    msg.Input_Mode = input_mode;

    return send(msg);
}

bool ODriveCAN::setTorque(float torque) {
    Set_Input_Torque_msg_t msg;

    msg.Input_Torque = torque;

    return send(msg);
}

bool ODriveCAN::setState(ODriveAxisState requested_state) {
    Set_Axis_State_msg_t msg;

    msg.Axis_Requested_State = (uint32_t)requested_state;

    return send(msg);
}

bool ODriveCAN::setLimits(float velocity_limit, float current_soft_max) {
    Set_Limits_msg_t msg;

    msg.Velocity_Limit = velocity_limit;
    msg.Current_Limit = current_soft_max;

    return send(msg);
}

bool ODriveCAN::setPosGain(float pos_gain) {
    Set_Pos_Gain_msg_t msg;

    msg.Pos_Gain = pos_gain;

    return send(msg);
}

bool ODriveCAN::setVelGains(float vel_gain, float vel_integrator_gain) {
    Set_Vel_Gains_msg_t msg;

    msg.Vel_Gain = vel_gain;
    msg.Vel_Integrator_Gain = vel_integrator_gain;

    return send(msg);
}

bool ODriveCAN::setAbsolutePosition(float abs_pos) {
    Set_Absolute_Position_msg_t msg;

    msg.Position = abs_pos;

    return send(msg);
}

bool ODriveCAN::setTrapezoidalVelLimit(float vel_limit) {
    Set_Traj_Vel_Limit_msg_t msg;

    msg.Traj_Vel_Limit = vel_limit;

    return send(msg);
}

bool ODriveCAN::setTrapezoidalAccelLimits(float accel_limit, float decel_limit) {
    Set_Traj_Accel_Limits_msg_t msg;

    msg.Traj_Accel_Limit = accel_limit;
    msg.Traj_Decel_Limit = decel_limit;

    return send(msg);
}

bool ODriveCAN::getCurrents(Get_Iq_msg_t& msg, uint16_t timeout_ms) {
    return request(msg, timeout_ms);
}

bool ODriveCAN::getTemperature(Get_Temperature_msg_t& msg, uint16_t timeout_ms) {
    return request(msg, timeout_ms);
}

bool ODriveCAN::getError(Get_Error_msg_t& msg, uint16_t timeout_ms) {
    return request(msg, timeout_ms);
}

bool ODriveCAN::getVersion(Get_Version_msg_t& msg, uint16_t timeout_ms) {
    return request(msg, timeout_ms);
}

bool ODriveCAN::getFeedback(Get_Encoder_Estimates_msg_t& msg, uint16_t timeout_ms) {
    return request(msg, timeout_ms);
}

bool ODriveCAN::getBusVI(Get_Bus_Voltage_Current_msg_t& msg, uint16_t timeout_ms) {
    return request(msg, timeout_ms);
}

bool ODriveCAN::getPower(Get_Powers_msg_t& msg, uint16_t timeout_ms) {
    return request(msg, timeout_ms);
}

void ODriveCAN::onReceive(uint32_t id, uint8_t length, const uint8_t* data) {
#ifdef DEBUG
    int byte_index = length - 1;
    Serial.println(F("received:"));
    Serial.print(F("  id: 0x"));
    Serial.println(id, HEX);
    Serial.print(F("  data: 0x"));
    while (byte_index >= 0)
        Serial.print(msg.data[byte_index--], HEX);
    Serial.println(F(""));
#endif // DEBUG
    if (node_id_ != (id >> ODriveCAN::kNodeIdShift))
        return;
    switch (id & ODriveCAN::kCmdIdBits) {
        case Get_Encoder_Estimates_msg_t::cmd_id: {
            Get_Encoder_Estimates_msg_t estimates;
            estimates.decode_buf(data);
            if (feedback_callback_)
                feedback_callback_(estimates, feedback_user_data_);
            break;
        }
        case Get_Torques_msg_t::cmd_id: {
            Get_Torques_msg_t estimates;
            estimates.decode_buf(data);
            if (torques_callback_)
                torques_callback_(estimates, torques_user_data_);
            break;
        }
        case Heartbeat_msg_t::cmd_id: {
            Heartbeat_msg_t status;
            status.decode_buf(data);
            if (axis_state_callback_ != nullptr)
                axis_state_callback_(status, axis_state_user_data_);
            else
                Serial.println(F("missing callback"));
            break;
        }
        default: {
            if (requested_msg_id_ == REQUEST_PENDING)
                return;
#ifdef DEBUG
            Serial.print(F("waiting for: 0x"));
            Serial.println(requested_msg_id_, HEX);
#endif // DEBUG
            if ((id & ODriveCAN::kCmdIdBits) != requested_msg_id_)
                return;
            memcpy(buffer_, data, length);
            requested_msg_id_ = REQUEST_PENDING;
        }
    }
}

bool ODriveCAN::awaitMsg(uint16_t timeout_ms) {
    uint64_t start_time = micros();
    while (requested_msg_id_ != REQUEST_PENDING) {
        can_intf_.pump_events(); // pump event loop while waiting
        if ((micros() - start_time) > (1000 * timeout_ms))
            return false;
    }
    return true;
}
