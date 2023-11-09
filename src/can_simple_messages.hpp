#pragma once

#include <stdint.h>

#include "can_helpers.hpp"

struct Get_Version_msg_t final {
    uint8_t Protocol_Version      = 0;
    uint8_t Hw_Version_Major      = 0;
    uint8_t Hw_Version_Minor      = 0;
    uint8_t Hw_Version_Variant    = 0;
    uint8_t Fw_Version_Major      = 0;
    uint8_t Fw_Version_Minor      = 0;
    uint8_t Fw_Version_Revision   = 0;
    uint8_t Fw_Version_Unreleased = 0;

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<uint8_t>(data, Protocol_Version, 0, 8, true);
        can_setSignal<uint8_t>(data, Hw_Version_Major, 8, 8, true);
        can_setSignal<uint8_t>(data, Hw_Version_Minor, 16, 8, true);
        can_setSignal<uint8_t>(data, Hw_Version_Variant, 24, 8, true);
        can_setSignal<uint8_t>(data, Fw_Version_Major, 32, 8, true);
        can_setSignal<uint8_t>(data, Fw_Version_Minor, 40, 8, true);
        can_setSignal<uint8_t>(data, Fw_Version_Revision, 48, 8, true);
        can_setSignal<uint8_t>(data, Fw_Version_Unreleased, 56, 8, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Protocol_Version      = can_getSignal<uint8_t>(data, 0, 8, true);
        Hw_Version_Major      = can_getSignal<uint8_t>(data, 8, 8, true);
        Hw_Version_Minor      = can_getSignal<uint8_t>(data, 16, 8, true);
        Hw_Version_Variant    = can_getSignal<uint8_t>(data, 24, 8, true);
        Fw_Version_Major      = can_getSignal<uint8_t>(data, 32, 8, true);
        Fw_Version_Minor      = can_getSignal<uint8_t>(data, 40, 8, true);
        Fw_Version_Revision   = can_getSignal<uint8_t>(data, 48, 8, true);
        Fw_Version_Unreleased = can_getSignal<uint8_t>(data, 56, 8, true);
    }

    Get_Version_msg_t() = default;

    Get_Version_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x000;
    const uint8_t msg_length = 8;
};

struct Heartbeat_msg_t final {
    uint32_t Axis_Error           = 0;
    uint8_t  Axis_State           = 0;
    uint8_t  Procedure_Result     = 0;
    uint8_t  Trajectory_Done_Flag = 0;

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<uint32_t>(data, Axis_Error, 0, 32, true);
        can_setSignal<uint8_t>(data, Axis_State, 32, 8, true);
        can_setSignal<uint8_t>(data, Procedure_Result, 40, 8, true);
        can_setSignal<uint8_t>(data, Trajectory_Done_Flag, 48, 1, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Axis_Error           = can_getSignal<uint32_t>(data, 0, 32, true);
        Axis_State           = can_getSignal<uint8_t>(data, 32, 8, true);
        Procedure_Result     = can_getSignal<uint8_t>(data, 40, 8, true);
        Trajectory_Done_Flag = can_getSignal<uint8_t>(data, 48, 1, true);
    }

    Heartbeat_msg_t() = default;

    Heartbeat_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x001;
    const uint8_t msg_length = 8;
};

struct Estop_msg_t final {
    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
    }

    Estop_msg_t() = default;

    Estop_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x002;
    const uint8_t msg_length = 0;
};

struct Get_Error_msg_t final {
    uint32_t Active_Errors = 0;
    uint32_t Disarm_Reason = 0;

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<uint32_t>(data, Active_Errors, 0, 32, true);
        can_setSignal<uint32_t>(data, Disarm_Reason, 32, 32, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Active_Errors = can_getSignal<uint32_t>(data, 0, 32, true);
        Disarm_Reason = can_getSignal<uint32_t>(data, 32, 32, true);
    }

    Get_Error_msg_t() = default;

    Get_Error_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x003;
    const uint8_t msg_length = 8;
};

struct Set_Axis_Node_ID_msg_t final {
    uint32_t Axis_Node_ID = 0;

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<uint32_t>(data, Axis_Node_ID, 0, 32, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Axis_Node_ID = can_getSignal<uint32_t>(data, 0, 32, true);
    }

    Set_Axis_Node_ID_msg_t() = default;

    Set_Axis_Node_ID_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x006;
    const uint8_t msg_length = 8;
};

struct Set_Axis_State_msg_t final {
    uint32_t Axis_Requested_State = 0;

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<uint32_t>(data, Axis_Requested_State, 0, 32, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Axis_Requested_State = can_getSignal<uint32_t>(data, 0, 32, true);
    }

    Set_Axis_State_msg_t() = default;

    Set_Axis_State_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x007;
    const uint8_t msg_length = 4;
};

struct Get_Encoder_Estimates_msg_t final {
    float Pos_Estimate = 0.0f;  // [rev]
    float Vel_Estimate = 0.0f;  // [rev/s]

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<float>(data, Pos_Estimate, 0, 32, true);
        can_setSignal<float>(data, Vel_Estimate, 32, 32, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Pos_Estimate = can_getSignal<float>(data, 0, 32, true, 1.0f, 0.0f);
        Vel_Estimate = can_getSignal<float>(data, 32, 32, true, 1.0f, 0.0f);
    }

    Get_Encoder_Estimates_msg_t() = default;

    Get_Encoder_Estimates_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id = 0x009;
    const uint8_t msg_length = 8;
};

struct Set_Controller_Mode_msg_t final {
    uint32_t Control_Mode = 0;
    uint32_t Input_Mode   = 0;

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<uint32_t>(data, Control_Mode, 0, 32, true);
        can_setSignal<uint32_t>(data, Input_Mode, 32, 32, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Control_Mode = can_getSignal<uint32_t>(data, 0, 32, true);
        Input_Mode   = can_getSignal<uint32_t>(data, 32, 32, true);
    }

    Set_Controller_Mode_msg_t() = default;

    Set_Controller_Mode_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id = 0x00B;
    const uint8_t msg_length = 8;
};

struct Set_Input_Pos_msg_t final {
    float Input_Pos = 0.0f;  // [rev]
    float Vel_FF    = 0.0f;  // [rev/s]
    float Torque_FF = 0.0f;  // [Nm]

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<float>(data, Input_Pos, 0, 32, true);
        can_setSignal<int16_t>(data, Vel_FF, 32, 16, true, 0.001f, 0.0f);
        can_setSignal<int16_t>(data, Torque_FF, 48, 16, true, 0.001f, 0.0f);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Input_Pos = can_getSignal<float>(data, 0, 32, true, 1.0f, 0.0f);
        Vel_FF    = can_getSignal<int16_t>(data, 32, 16, true, 0.001f, 0.0f);
        Torque_FF = can_getSignal<int16_t>(data, 48, 16, true, 0.001f, 0.0f);
    }

    Set_Input_Pos_msg_t() = default;

    Set_Input_Pos_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x00C;
    const uint8_t msg_length = 8;
};

struct Set_Input_Vel_msg_t final {
    float Input_Vel       = 0.0f;  // [rev/s]
    float Input_Torque_FF = 0.0f;  // [Nm]

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<float>(data, Input_Vel, 0, 32, true);
        can_setSignal<float>(data, Input_Torque_FF, 32, 32, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Input_Vel       = can_getSignal<float>(data, 0, 32, true, 1.0f, 0.0f);
        Input_Torque_FF = can_getSignal<float>(data, 32, 32, true, 1.0f, 0.0f);
    }

    Set_Input_Vel_msg_t() = default;

    Set_Input_Vel_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x00D;
    const uint8_t msg_length = 8;
};

struct Set_Input_Torque_msg_t final {
    float Input_Torque = 0.0f;  // [Nm]

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<float>(data, Input_Torque, 0, 32, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Input_Torque = can_getSignal<float>(data, 0, 32, true, 1.0f, 0.0f);
    }

    Set_Input_Torque_msg_t() = default;

    Set_Input_Torque_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x00E;
    const uint8_t msg_length = 8;
};

struct Set_Limits_msg_t final {
    float Velocity_Limit = 0.0f;  // [rev/s]
    float Current_Limit  = 0.0f;  // [A]

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<float>(data, Velocity_Limit, 0, 32, true);
        can_setSignal<float>(data, Current_Limit, 32, 32, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Velocity_Limit = can_getSignal<float>(data, 0, 32, true, 1.0f, 0.0f);
        Current_Limit  = can_getSignal<float>(data, 32, 32, true, 1.0f, 0.0f);
    }

    Set_Limits_msg_t() = default;

    Set_Limits_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x00F;
    const uint8_t msg_length = 8;
};

struct Set_Traj_Vel_Limit_msg_t final {
    float Traj_Vel_Limit = 0.0f;  // [rev/s]

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<float>(data, Traj_Vel_Limit, 0, 32, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Traj_Vel_Limit = can_getSignal<float>(data, 0, 32, true, 1.0f, 0.0f);
    }

    Set_Traj_Vel_Limit_msg_t() = default;

    Set_Traj_Vel_Limit_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x011;
    const uint8_t msg_length = 8;
};

struct Set_Traj_Accel_Limits_msg_t final {
    float Traj_Accel_Limit = 0.0f;  // [rev/s^2]
    float Traj_Decel_Limit = 0.0f;  // [rev/s^2]

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<float>(data, Traj_Accel_Limit, 0, 32, true);
        can_setSignal<float>(data, Traj_Decel_Limit, 32, 32, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Traj_Accel_Limit = can_getSignal<float>(data, 0, 32, true, 1.0f, 0.0f);
        Traj_Decel_Limit = can_getSignal<float>(data, 32, 32, true, 1.0f, 0.0f);
    }

    Set_Traj_Accel_Limits_msg_t() = default;

    Set_Traj_Accel_Limits_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x012;
    const uint8_t msg_length = 8;
};

struct Set_Traj_Inertia_msg_t final {
    float Traj_Inertia = 0.0f;  // [Nm/(rev/s^2)]

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<float>(data, Traj_Inertia, 0, 32, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Traj_Inertia = can_getSignal<float>(data, 0, 32, true, 1.0f, 0.0f);
    }

    Set_Traj_Inertia_msg_t() = default;

    Set_Traj_Inertia_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x013;
    const uint8_t msg_length = 8;
};

struct Get_Iq_msg_t final {
    float Iq_Setpoint = 0.0f;  // [A]
    float Iq_Measured = 0.0f;  // [A]

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<float>(data, Iq_Setpoint, 0, 32, true);
        can_setSignal<float>(data, Iq_Measured, 32, 32, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Iq_Setpoint = can_getSignal<float>(data, 0, 32, true, 1.0f, 0.0f);
        Iq_Measured = can_getSignal<float>(data, 32, 32, true, 1.0f, 0.0f);
    }

    Get_Iq_msg_t() = default;

    Get_Iq_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x014;
    const uint8_t msg_length = 8;
};

struct Get_Temperature_msg_t final {
    float FET_Temperature   = 0.0f;  // [deg C]
    float Motor_Temperature = 0.0f;  // [deg C]

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<float>(data, FET_Temperature, 0, 32, true);
        can_setSignal<float>(data, Motor_Temperature, 32, 32, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        FET_Temperature   = can_getSignal<float>(data, 0, 32, true, 1.0f, 0.0f);
        Motor_Temperature = can_getSignal<float>(data, 32, 32, true, 1.0f, 0.0f);
    }

    Get_Temperature_msg_t() = default;

    Get_Temperature_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x015;
    const uint8_t msg_length = 8;
};

struct Reboot_msg_t final {
    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
    }

    Reboot_msg_t() = default;

    Reboot_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x016;
    const uint8_t msg_length = 0;
};

struct Get_Bus_Voltage_Current_msg_t final {
    float Bus_Voltage = 0.0f;  // [V]
    float Bus_Current = 0.0f;  // [A]

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<float>(data, Bus_Voltage, 0, 32, true);
        can_setSignal<float>(data, Bus_Current, 32, 32, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Bus_Voltage = can_getSignal<float>(data, 0, 32, true, 1.0f, 0.0f);
        Bus_Current = can_getSignal<float>(data, 32, 32, true, 1.0f, 0.0f);
    }

    Get_Bus_Voltage_Current_msg_t() = default;

    Get_Bus_Voltage_Current_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x017;
    const uint8_t msg_length = 8;
};

struct Clear_Errors_msg_t final {
    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
    }

    Clear_Errors_msg_t() = default;

    Clear_Errors_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x018;
    const uint8_t msg_length = 0;
};

struct Set_Absolute_Position_msg_t final {
    float Position = 0.0f;  // [rev]

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<float>(data, Position, 0, 32, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Position = can_getSignal<float>(data, 0, 32, true, 1.0f, 0.0f);
    }

    Set_Absolute_Position_msg_t() = default;

    Set_Absolute_Position_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x019;
    const uint8_t msg_length = 8;
};

struct Set_Pos_Gain_msg_t final {
    float Pos_Gain = 0.0f;  // [(rev/s) / rev]

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<float>(data, Pos_Gain, 0, 32, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Pos_Gain = can_getSignal<float>(data, 0, 32, true, 1.0f, 0.0f);
    }

    Set_Pos_Gain_msg_t() = default;

    Set_Pos_Gain_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x01A;
    const uint8_t msg_length = 8;
};

struct Set_Vel_Gains_msg_t final {
    float Vel_Gain            = 0.0f;  // [Nm / (rev/s)]
    float Vel_Integrator_Gain = 0.0f;  // [Nm / rev]

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<float>(data, Vel_Gain, 0, 32, true);
        can_setSignal<float>(data, Vel_Integrator_Gain, 32, 32, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Vel_Gain            = can_getSignal<float>(data, 0, 32, true, 1.0f, 0.0f);
        Vel_Integrator_Gain = can_getSignal<float>(data, 32, 32, true, 1.0f, 0.0f);
    }

    Set_Vel_Gains_msg_t() = default;

    Set_Vel_Gains_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x01B;
    const uint8_t msg_length = 8;
};

struct Get_Torques_msg_t final {
    float Torque_Target   = 0.0f;  // [Nm]
    float Torque_Estimate = 0.0f;  // [Nm]

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<float>(data, Torque_Target, 0, 32, true);
        can_setSignal<float>(data, Torque_Estimate, 32, 32, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Torque_Target   = can_getSignal<float>(data, 0, 32, true, 1.0f, 0.0f);
        Torque_Estimate = can_getSignal<float>(data, 32, 32, true, 1.0f, 0.0f);
    }

    Get_Torques_msg_t() = default;

    Get_Torques_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x01C;
    const uint8_t msg_length = 8;
};

struct Get_Controller_Error_msg_t final {
    uint32_t Controller_Error = 0;

    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
        can_setSignal<uint32_t>(data, Controller_Error, 0, 32, true);
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
        Controller_Error = can_getSignal<uint32_t>(data, 0, 32, true);
    }

    Get_Controller_Error_msg_t() = default;

    Get_Controller_Error_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x01D;
    const uint8_t msg_length = 8;
};

struct Enter_DFU_Mode_msg_t final {
    template <size_t N>
    void encode(uint8_t (&data)[N]) const {
    }

    template <size_t N>
    void decode(const uint8_t (&data)[N]) {
    }

    Enter_DFU_Mode_msg_t() = default;

    Enter_DFU_Mode_msg_t(const can_Message_t& msg) {
        decode(msg.data);
    }

    void encode(can_Message_t& msg) const {
        encode(msg.data);
    }

    void decode(const can_Message_t& msg) {
        decode(msg.data);
    }

    static const uint8_t cmd_id     = 0x01F;
    const uint8_t msg_length = 0;
};