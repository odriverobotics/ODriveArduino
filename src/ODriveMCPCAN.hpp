#pragma once

#include "MCP2515.h"
#include "ODriveCAN.h"

// This is a convenience struct because the MCP2515 library doesn't have a
// native message type.
struct CanMsg {
    uint32_t id;
    uint8_t len;
    uint8_t buffer[8];
};

// Must be defined by the application if you want to use defaultCanReceiveCallback().
void onCanMessage(const CanMsg& msg);

static bool sendMsg(MCP2515Class& can_intf, uint32_t id, uint8_t length, const uint8_t* data) {
    if (id & 0x80000000) {
        can_intf.beginExtendedPacket(id & 0x1fffffff, length, !data);
    } else {
        can_intf.beginPacket(id, length, !data);
    }
    if (data) {
        for (int i = 0; i < length; ++i) {
            can_intf.write(data[i]);
        }
    }
    return can_intf.endPacket();
}

static void onReceive(const CanMsg& msg, ODriveCAN& odrive) {
    odrive.onReceive(msg.id, msg.len, msg.buffer);
}

static void pumpEvents(MCP2515Class& intf) {
    // On other platforms, this polls and processes incoming CAN messages.
    // However, this is not possible on MCP2515-based platforms.
    //
    // A 10ms delay was found to reduce the number of dropped messages, however a
    // specific root cause has not been identified, and may be a quirk in the MCP2515.
    delay(10);
}

CREATE_CAN_INTF_WRAPPER(MCP2515Class)
