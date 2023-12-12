#pragma once

#include "ODriveCAN.h"
#include <api/HardwareCAN.h>

// Must be defined by the application
void onCanMessage(const CanMsg& msg);

static bool sendMsg(HardwareCAN& can_intf, uint32_t id, uint8_t length, const uint8_t* data) {
    // Note: Arduino_CAN does not support the RTR bit. The ODrive interprets
    // zero-length packets the same as RTR=1, but it creates the possibility of
    // collisions.
    CanMsg msg(
        (id & 0x80000000) ? CanExtendedId(id) : CanStandardId(id),
        length,
        data
    );
    return can_intf.write(msg) >= 0;
}

static void onReceive(const CanMsg& msg, ODriveCAN& odrive) {
    odrive.onReceive(msg.id, msg.data_length, msg.data);
}

static void pumpEvents(HardwareCAN& intf, int max_events = 100) {
    // max_events prevents an infinite loop if messages come at a high rate
    while (intf.available() && max_events--) {
        onCanMessage(intf.read());
    }
}

CREATE_CAN_INTF_WRAPPER(HardwareCAN)
