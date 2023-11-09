#pragma once

#ifdef DEBUG
#include <Arduino.h>
#endif
#include <type_traits>
#include "FlexCAN_T4.h"
#include "ODriveCAN.hpp"

template <typename FlexCANTypes>
bool ODriveCAN<FlexCANTypes>::sendMsg(can_Message_t msg) {
#ifdef DEBUG
    int byte_index = msg.len - 1;
    Serial.print("sending: 0x");
    Serial.print(msg.id, HEX);
    Serial.print(" - 0x");
    while (byte_index >= 0) Serial.print(msg.data[byte_index--], HEX);
    Serial.println("");
#endif // DEBUG
    CAN_message_t teensy_msg;
    teensy_msg.id = msg.id;
    teensy_msg.len = msg.len;
    teensy_msg.flags = {.extended = is_extended_};
    memcpy(teensy_msg.buf, msg.data, msg.len);
    can_intf_.events();
    return (can_intf_.write(teensy_msg) > 0);
}

template <typename FlexCANTypes>
template <class CANMsgType>
can_Message_t ODriveCAN<FlexCANTypes>::formatPacket(CANMsgType& data) {
    static_assert(std::is_same<decltype(data), const CAN_message_t&>::value, "CANMsgType must be: const CAN_message_t");
    can_Message_t msg;
    msg.id = data.id;
    msg.len = data.len;
    msg.is_extended_id = data.flags.extended;
    memcpy(msg.data, data.buf, data.len);
    return msg;
}

template <typename FlexCANTypes>
uint64_t ODriveCAN<FlexCANTypes>::events() {
    return can_intf_.events();
}
