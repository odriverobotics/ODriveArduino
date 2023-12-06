#pragma once

#include "ODriveEnums.h"
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"

// #define DEBUG

#define REQUEST_PENDING 0xff

#define CREATE_CAN_INTF_WRAPPER(TIntf) \
    ODriveCanIntfWrapper wrap_can_intf(TIntf& intf) { \
        return { \
            &intf, \
            [](void* intf, uint32_t id, uint8_t length, const uint8_t* data) { return sendMsg(*(TIntf*)intf, id, length, data); }, \
            [](void* intf) { pumpEvents(*(TIntf*)intf); } \
        }; \
    }


struct ODriveCanIntfWrapper {
    bool sendMsg(uint32_t id, uint8_t length, const uint8_t* data) {
        return (*send_msg_)(can_intf_, id, length, data);
    }
    void pump_events() {
        (*pump_events_)(can_intf_);
    }

    void* can_intf_;
    bool (*send_msg_)(void* intf, uint32_t id, uint8_t length, const uint8_t* data);
    void (*pump_events_)(void* intf);
};

class ODriveCAN {
public:
    ODriveCAN(const ODriveCanIntfWrapper& can_intf, uint32_t node_id)
        : can_intf_(can_intf), node_id_(node_id) {};

    /**
     * @brief Clear all errors on the ODrive.
     */
    bool clearErrors();
    
    /**
     * @brief Sets the control mode and input mode of the ODrive.
     */
    bool setControllerMode(uint8_t control_mode, uint8_t input_mode);
    
    /**
     * @brief Sends a position setpoint with optional velocity and torque feedforward.
     */
    bool setPosition(float position, float velocity_feedforward = 0.0f, float torque_feedforward = 0.0f);

    /**
     * @brief Sends a velocity setpoint with optional torque feedforward.
     */
    bool setVelocity(float velocity, float torque_feedforward = 0.0f);

    /**
     * @brief Sends a torque setpoint to the ODrive.
     */
    bool setTorque(float torque);

    /**
     * @brief Initiates a trapezoidal trajectory move to a specified position.
     */
    bool trapezoidalMove(float position);

    /**
     * @brief Registers a callback for ODrive feedback processing.
     */
    void onFeedback(void (*callback)(Get_Encoder_Estimates_msg_t& feedback, void* user_data), void* user_data = nullptr) {
        feedback_callback_ = callback; 
        feedback_user_data_ = user_data;
    }

    /**
     * @brief Registers a callback for ODrive axis state feedback.
     */
    void onStatus(void (*callback)(Heartbeat_msg_t& feedback, void* user_data), void* user_data = nullptr) {
        axis_state_callback_ = callback; 
        axis_state_user_data_ = user_data;
    }
    
    /**
     * @brief Tells the ODrive to change its axis state.
     */
    bool setState(ODriveAxisState requested_state);

    /**
     * @brief Processes received CAN messages for the ODrive.
     */
    void onReceive(uint32_t id, uint8_t length, const uint8_t* data);

    /**
     * @brief Sends a request message and awaits a response.
     */
    template<typename T>
    bool request(T& msg, uint16_t timeout = 10) {
        requested_msg_id_ = msg.cmd_id;
        can_intf_.sendMsg(
            (node_id_ << ODriveCAN::kNodeIdShift) | msg.cmd_id,
            0, // no data
            nullptr // RTR=1
        );
        if (!awaitMsg(timeout)) return false;
        msg.decode_buf(buffer_);
        return true;
    }

    /**
     * @brief Sends a specified message over CAN.
     */
    template<typename T>
    bool send(T& msg) {
        uint8_t data[8] = {};
        msg.encode_buf(data);
        return can_intf_.sendMsg(
            (node_id_ << ODriveCAN::kNodeIdShift) | msg.cmd_id,
            msg.msg_length,
            data
        );
    }

private:
    bool awaitMsg(uint16_t timeout);

    ODriveCanIntfWrapper can_intf_;
    uint32_t node_id_;

    volatile uint8_t requested_msg_id_ = REQUEST_PENDING;

    uint8_t buffer_[8];

    static const uint8_t kNodeIdShift = 5;
    static const uint8_t kCmdIdBits = 0x1F;

    void* axis_state_user_data_;
    void* feedback_user_data_;
    
    void (*axis_state_callback_)(Heartbeat_msg_t& feedback, void* user_data) = nullptr;
    void (*feedback_callback_)(Get_Encoder_Estimates_msg_t& feedback, void* user_data) = nullptr;
};
