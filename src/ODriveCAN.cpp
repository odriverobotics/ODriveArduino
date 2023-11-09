#ifdef DEBUG
#include <Arduino.h> // needed for debug printing
#endif // DEBUG

#include "ODriveCAN.hpp"

bool ODriveCANBase::clearErrors() {
    Clear_Errors_msg_t clear_errors_msg;
    return send(clear_errors_msg);
}

bool ODriveCANBase::setPosition(float position, float velocity_feedforward, float torque_feedforward) {
    Set_Input_Pos_msg_t input_pos_msg;
    input_pos_msg.Input_Pos = position;
    input_pos_msg.Vel_FF = velocity_feedforward;
    input_pos_msg.Torque_FF = torque_feedforward;
    return send(input_pos_msg);
}

bool ODriveCANBase::setVelocity(float velocity, float torque_feedforward) {
    Set_Input_Vel_msg_t input_vel_msg;
    input_vel_msg.Input_Vel = velocity;
    input_vel_msg.Input_Torque_FF = torque_feedforward;
    return send(input_vel_msg);
}

bool ODriveCANBase::setControllerMode(uint8_t control_mode, uint8_t input_mode) {
    Set_Controller_Mode_msg_t ctrl_mode_msg;
    ctrl_mode_msg.Control_Mode = control_mode;
    ctrl_mode_msg.Input_Mode = input_mode;
    return send(ctrl_mode_msg);
}

bool ODriveCANBase::setTorque(float torque) {
    Set_Input_Torque_msg_t input_torque_msg;
    input_torque_msg.Input_Torque = torque;
    return send(input_torque_msg);
}

bool ODriveCANBase::trapezoidalMove(float position) {
    Set_Input_Pos_msg_t input_pos_msg;
    input_pos_msg.Input_Pos = position;
    return send(input_pos_msg);
}

bool ODriveCANBase::setState(ODriveAxisState requested_state) {
    Set_Axis_State_msg_t axist_state_msg;
    axist_state_msg.Axis_Requested_State = requested_state;
    return send(axist_state_msg);
}

void ODriveCANBase::onReceive(const can_Message_t& msg) {
#ifdef DEBUG
    int byte_index = msg.len - 1;
    Serial.println("received:");
    Serial.print("  id: 0x");
    Serial.println(msg.id, HEX);
    Serial.print("  data: 0x");
    while (byte_index >= 0) Serial.print(msg.data[byte_index--], HEX);
    Serial.println("");
#endif // DEBUG
    if (node_id_ != (msg.id >> ODriveCANBase::kNodeIdShift)) return;
    switch(msg.id & ODriveCANBase::kCmdIdBits) {
        case Get_Encoder_Estimates_msg_t::cmd_id: {
            Get_Encoder_Estimates_msg_t estimates;
            estimates.decode(msg.data);
            if (feedback_callback_) feedback_callback_(estimates, feedback_user_data_);
            break;
        }
        case Heartbeat_msg_t::cmd_id: {
            Heartbeat_msg_t status;
            status.decode(msg.data);
            if (axis_state_callback_ != nullptr) axis_state_callback_(status, axis_state_user_data_);
            else Serial.println("missing callback");
            break;
        }
        default: {
            if (requested_msg_id_ == REQUEST_PENDING) return;
#ifdef DEBUG
            Serial.print("waiting for: 0x");
            Serial.println(requested_msg_id_, HEX);
#endif // DEBUG
            if ((msg.id & ODriveCANBase::kCmdIdBits) != requested_msg_id_) return;
            memcpy(buffer_, msg.data, msg.len);
            requested_msg_id_ = REQUEST_PENDING;
        }
    }
}

bool ODriveCANBase::awaitMsg(uint16_t timeout) {
    uint64_t start_time = millis();
    while (requested_msg_id_ != REQUEST_PENDING) {
        events(); // pump event loop while waiting
        if ((millis() - start_time) > 1000 * timeout) return false;
    }
    return true;
}
