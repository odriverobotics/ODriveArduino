#pragma once
#include "MCP2515.h"
#include "ODriveCAN.hpp"

#ifdef DEBUG
#include <Arduino.h>
#endif

template <>
bool ODriveCAN<MCP2515Class>::sendMsg(can_Message_t msg) {
#ifdef DEBUG
    int byte_index = msg.len - 1;
    Serial.print("sending: 0x");
    Serial.print(msg.id, HEX);
    Serial.print(" - 0x");
    while (byte_index >= 0) Serial.print(msg.data[byte_index--], HEX);
    Serial.println("");
    byte_index = 0;
#else
    int byte_index;
#endif // DEBUG
    byte_index = 0;
    if (is_extended_) can_intf_.beginExtendedPacket(msg.id);
    else can_intf_.beginPacket(msg.id);
    while (byte_index < msg.len) can_intf_.write(msg.data[byte_index++]);
    return can_intf_.endPacket();
}
template <>
can_Message_t ODriveCAN<MCP2515Class>::formatPacket(int packet_size, MCP2515Class& can_intf) {
    can_Message_t output;
    output.len = packet_size;
    output.id = can_intf.packetId();
    can_intf.readBytes(output.data, packet_size);
    return output;
}

template <>
uint64_t ODriveCAN<MCP2515Class>::events() {
    delay(10); // not sure why this resulted in less dropped messages, could have been a twisted coincidence
    return 1;
}