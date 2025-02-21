#pragma once

#include "ODriveEnums.h"
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"

// #define DEBUG

#define REQUEST_PENDING 0xff

#define CREATE_CAN_INTF_WRAPPER(TIntf) \
    static inline ODriveCanIntfWrapper wrap_can_intf(TIntf& intf) { \
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
     * 
     * This function returns immediately and does not check if the ODrive
     * received the CAN message.
     */
    bool clearErrors();
    
    /**
     * @brief Tells the ODrive to change its axis state.
     * 
     * This function returns immediately and does not check if the ODrive
     * received the CAN message.
     */
    bool setState(ODriveAxisState requested_state);
    
    /**
     * @brief Sets the control mode and input mode of the ODrive.
     * 
     * This function returns immediately and does not check if the ODrive
     * received the CAN message.
     */
    bool setControllerMode(uint8_t control_mode, uint8_t input_mode);
    
    /**
     * @brief Sends a position setpoint with optional velocity and torque feedforward.
     * 
     * This function returns immediately and does not check if the ODrive
     * received the CAN message.
     */
    bool setPosition(float position, float velocity_feedforward = 0.0f, float torque_feedforward = 0.0f);

    /**
     * @brief Sends a velocity setpoint with optional torque feedforward.
     * 
     * This function returns immediately and does not check if the ODrive
     * received the CAN message.
     */
    bool setVelocity(float velocity, float torque_feedforward = 0.0f);

    /**
     * @brief Sends a torque setpoint to the ODrive.
     * 
     * This function returns immediately and does not check if the ODrive
     * received the CAN message.
     */
    bool setTorque(float torque);

    /**
     * @brief Sets the velocity and current limits
     * 
     * This function returns immediately and does not check if the ODrive
     * received the CAN message.
     */
    bool setLimits(float velocity_limit, float current_soft_max);

    /**
     * @brief Sets the position gain
     * 
     * This function returns immediately and does not check if the ODrive
     * received the CAN message.
     */
    bool setPosGain(float pos_gain);
    
    /**
     * @brief Sets the velocity and velocity integrator gains
     * 
     * This function returns immediately and does not check if the ODrive
     * received the CAN message.
     */
    bool setVelGains(float vel_gain, float vel_integrator_gain);
    
    /**
     * @brief Sets the encoder's absolute position and enables absolute positioning
     * 
     * This function returns immediately and does not check if the ODrive
     * received the CAN message.
     */
    bool setAbsolutePosition(float abs_pos);
    
    /**
     * @brief Sets the coast velocity for subsequent trapezoidal moves
     * 
     * This function returns immediately and does not check if the ODrive
     * received the CAN message.
     */
    bool setTrapezoidalVelLimit(float vel_limit);
    
    /**
     * @brief Sets the acceleration and deceleration values for subsequent trapezoidal moves
     * 
     * This function returns immediately and does not check if the ODrive
     * received the CAN message.
     */
    bool setTrapezoidalAccelLimits(float accel_limit, float decel_limit);

    /**
     * @brief Requests motor current.  Iq_measured represents torque-generating current
     * 
     * This function will block and wait for up to timeout_ms (default 10msec) for ODrive to reply
     */
    bool getCurrents(Get_Iq_msg_t& msg, uint16_t timeout_ms = 10);

    /**
     * @brief Requests motor temperature 
     * 
     * This function will block and wait for up to timeout_ms (default 10msec) for ODrive to reply
     */
    bool getTemperature(Get_Temperature_msg_t& msg, uint16_t timeout_ms = 10);

    /**
     * @brief Requests error information
     * 
     * This function will block and wait for up to timeout_ms (default 10msec) for ODrive to reply
     */
    bool getError(Get_Error_msg_t& msg, uint16_t timeout_ms = 10);

    /**
     * @brief Requests hardware and firmware version information
     * 
     * This function will block and wait for up to timeout_ms (default 10msec) for ODrive to reply
     */
    bool getVersion(Get_Version_msg_t& msg, uint16_t timeout_ms = 10);

    /**
     * @brief Requests encoder feedback data.  May trigger onFeedback callback if it's registered
     * 
     * This function will block and wait for up to timeout_ms (default 10msec) for ODrive to reply
     */
    bool getFeedback(Get_Encoder_Estimates_msg_t& msg, uint16_t timeout_ms = 10);

    /**
     * @brief Requests ODrive DC bus voltage and current
     * 
     * This function will block and wait for up to timeout_ms (default 10msec) for ODrive to reply
     */
    bool getBusVI(Get_Bus_Voltage_Current_msg_t& msg, uint16_t timeout_ms = 10);

    /**
     * @brief Requests mechanical and electrical power data (used for spinout detection)
     * 
     * This function will block and wait for up to timeout_ms (default 10msec) for ODrive to reply
     */
    bool getPower(Get_Powers_msg_t& msg, uint16_t timeout_ms = 10);
    
    enum ResetAction {
        Reboot,
        SaveConfiguration,
        EraseConfiguration
    };
    
    /**
     * @brief Resets the ODrive with the given action
     * 
     * Valid actions:
     *   - Reboot (0)
     *   - Save (1)
     *   - Erase (2)
     *
     */
    bool reset(ResetAction action = ResetAction::Reboot);
    
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
     * @brief Registers a callback for ODrive torques feedback processing.
     */
    void onTorques(void (*callback)(Get_Torques_msg_t& feedback, void* user_data), void* user_data = nullptr) {
        torques_callback_ = callback; 
        torques_user_data_ = user_data;
    }

    /**
     * @brief Processes received CAN messages for the ODrive.
     */
    void onReceive(uint32_t id, uint8_t length, const uint8_t* data);

    /**
     * @brief Sends a request message and awaits a response.
     * 
     * Blocks until the response is received or the timeout is reached. Returns
     * false if the ODrive does not respond within the specified timeout.
     */
    template<typename T>
    bool request(T& msg, uint16_t timeout_ms = 10) {
        requested_msg_id_ = msg.cmd_id;
        can_intf_.sendMsg(
            (node_id_ << ODriveCAN::kNodeIdShift) | msg.cmd_id,
            0, // no data
            nullptr // RTR=1
        );
        if (!awaitMsg(timeout_ms)) return false;
        msg.decode_buf(buffer_);
        return true;
    }

    /**
     * @brief Sends a specified message over CAN.
     */
    template<typename T>
    bool send(const T& msg) {
        uint8_t data[8] = {};
        msg.encode_buf(data);
        return can_intf_.sendMsg(
            (node_id_ << ODriveCAN::kNodeIdShift) | msg.cmd_id,
            msg.msg_length,
            data
        );
    }

    /**
     * @brief Get value at the endpoint
     * 
     * @tparam T The data type expected from the endpoint
     * @param endpoint_id Unique ID from flat_endpoints.json
     * @param timeout_ms Time to wait for a response from ODrive
     * 
     * @return T Data from the endpoint, or 0 on timeout
     *
     * Blocks until the response is received or the timeout is reached.
     *
     */
    template <typename T>
    T getEndpoint(uint16_t endpoint_id, uint16_t timeout_ms = 10) {
        uint8_t data[8] = {};
        data[0] = 0; // Opcode read

        // Little-endian endpoint
        data[1] = (uint8_t)(endpoint_id);
        data[2] = (uint8_t)(endpoint_id >> 8);

        requested_msg_id_ = 0x005; // Await TxSdo message
        can_intf_.sendMsg((node_id_ << ODriveCAN::kNodeIdShift) | 0x004, 8, data);
        if (!awaitMsg(timeout_ms)) return T{};

        T ret{};
        memcpy(&ret, &buffer_[4], sizeof(T));
        return ret;
    }

    /**
     * @brief Set endpoint to value
     * 
     * @tparam T Type of the value from flat_endpoints.json
     * @param endpoint_id Unique ID of endpoint from flat_endpoints.json
     * @param value value to write to the endpoint
     *
     * This function returns immediately and does not check if the ODrive
     * received the CAN message.
     */
    template <typename T>
    bool setEndpoint(uint16_t endpoint_id, T value) {
        uint8_t data[8] = {};
        data[0] = 1; // Opcode write

        // Endpoint
        data[1] = endpoint_id & 0xFF;
        data[2] = (endpoint_id >> 8) & 0xFF;

        // Value to write
        memcpy(&data[4], &value, sizeof(T));

        can_intf_.sendMsg((node_id_ << ODriveCAN::kNodeIdShift) | 0x004, 8, data);
        return true;
    }

private:
    bool awaitMsg(uint16_t timeout_ms);

    ODriveCanIntfWrapper can_intf_;
    uint32_t node_id_;

    volatile uint8_t requested_msg_id_ = REQUEST_PENDING;

    uint8_t buffer_[8];

    static const uint8_t kNodeIdShift = 5;
    static const uint8_t kCmdIdBits = 0x1F;

    void* axis_state_user_data_;
    void* feedback_user_data_;
    void* torques_user_data_;
    
    void (*axis_state_callback_)(Heartbeat_msg_t& feedback, void* user_data) = nullptr;
    void (*feedback_callback_)(Get_Encoder_Estimates_msg_t& feedback, void* user_data) = nullptr;
    void (*torques_callback_)(Get_Torques_msg_t& feedback, void* user_data) = nullptr;
};
