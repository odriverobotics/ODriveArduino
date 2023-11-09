#pragma once

#include "Arduino.h"
#include "ODriveEnums.h"
#include "can_simple_messages.hpp"

// #define DEBUG

#define REQUEST_PENDING 0xff

class ODriveCANBase {
public:

    ODriveCANBase(uint32_t node_id, bool is_extended)
     : node_id_(node_id), is_extended_(is_extended) {};

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
    void onReceive(const can_Message_t& msg);
    /**
     * @brief Sends a request message and awaits a response.
     */
    template<typename T>
    bool request(T& msg, uint16_t timeout = 10) {
        can_Message_t requested;
        requested.id = (node_id_ << ODriveCANBase::kNodeIdShift) | msg.cmd_id;
        requested.rtr = true;
        sendMsg(requested);
        return await(msg);
    }
    /**
     * @brief Waits for a specific response message.
     */
    template<typename T>
    bool await(T& msg, uint16_t timeout = 10) {
        requested_msg_id_ = msg.cmd_id;
        if (!awaitMsg(timeout)) return false;
        msg.decode(buffer_);
        return true;
    }

    /**
     * @brief Sends a specified message over CAN.
     */
    template<typename T>
    bool send(T& msg) {
        can_Message_t txmsg;
        txmsg.id = (node_id_ << ODriveCANBase::kNodeIdShift) | msg.cmd_id;
        txmsg.len = msg.msg_length;
        msg.encode(txmsg.data);
        return sendMsg(txmsg);
    }

    virtual uint64_t events();

protected:

    virtual bool sendMsg(can_Message_t msg);

    uint32_t node_id_;
    bool is_extended_;
    volatile uint8_t requested_msg_id_ = REQUEST_PENDING;

private:

    bool awaitMsg(uint16_t timeout);

    uint8_t buffer_[8];

    static const uint8_t kNodeIdShift = 5;
    static const uint8_t kCmdIdBits = 0x1F;

    void* axis_state_user_data_;
    void* feedback_user_data_;
    
    void (*axis_state_callback_)(Heartbeat_msg_t& feedback, void* user_data) = nullptr;
    void (*feedback_callback_)(Get_Encoder_Estimates_msg_t& feedback, void* user_data) = nullptr;

};

template <typename CANInterfaceType>
class ODriveCAN : public ODriveCANBase {
public:
    /**
     * @brief Constructs an ODriveFlexCAN object with a specified node ID.
     */
    ODriveCAN(CANInterfaceType& can_intf, uint32_t node_id, bool is_extended=false)
    : ODriveCANBase(node_id, is_extended), can_intf_(can_intf) {}; // HERE

    template <class CANMsgType>
    static can_Message_t formatPacket(CANMsgType& data);

    static can_Message_t formatPacket(int packet_size, CANInterfaceType& can_intf);

    uint64_t events();

private:
    bool sendMsg(can_Message_t msg);
    CANInterfaceType& can_intf_;
};