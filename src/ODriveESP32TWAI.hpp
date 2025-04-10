#pragma once

#include "TWAI_CAN_Class.h" // Include your custom TWAIClass header
#include "ODriveCAN.h"

// This is a convenience struct because the TWAIClass doesn't have a
// native message type.
struct CanMsg {
    uint32_t id;
    uint8_t len;
    uint8_t buffer[8];
};

// Must be defined by the application if you want to use defaultCanReceiveCallback().
void onCanMessage(const CanMsg& msg);


static inline bool sendMsg(TWAIClass& can_intf, uint32_t id, uint8_t length, const uint8_t* data) {
    // Send CAN message
    can_intf.prepareMessage(id, length, !data);
    if (data) {
        for (int i = 0; i < length; ++i) {
            can_intf.write(data[i], i);
        }
    }
    return can_intf.endPacket();
}

static inline void onReceive(const CanMsg& msg, ODriveCAN& odrive) {
    odrive.onReceive(msg.id, msg.len, msg.buffer);
}

static inline void pumpEvents(TWAIClass& intf) {
    CanMsg msg;
    int length = intf.parsePacket(); // Check if a packet is available

    if (length > 0) {
        msg.id = intf.packetId(); // Retrieve the ID of the received message
        // Debug print to check if ID is read correctly
        //Serial.print("Received Raw Frame ID: 0x"); // DEBUG PRINT uncomment to print all messages to serial console **********************************
        //Serial.println(msg.id, HEX); // DEBUG PRINT uncomment to print all messages to serial console **********************************
        msg.len = length;
        intf.readBytes(msg.buffer, length); // Retrieve the data from the message
        onCanMessage(msg); // Call the user-defined callback
    }
}

CREATE_CAN_INTF_WRAPPER(TWAIClass)
