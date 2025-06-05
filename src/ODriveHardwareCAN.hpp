// Glue layer for platforms that implement the Arduino's official HardwareCAN API.

#pragma once

#include "ODriveCAN.h"

#include <api/HardwareCAN.h>

// Must be defined by the application
void onCanMessage(const CanMsg& msg);

/**
 * @brief Sends a CAN message over the specified platform-specific interface.
 *
 * @param can_intf A platform-specific reference to the CAN interface to use.
 * @param id: The CAN message ID to send.
 *        Bit 31 indicates if the ID is extended (29-bit) or standard (11-bit).
 *        Bits 30 and 29 are reserved.
 * @param length: The length of the data in bytes (0-8). For RTR messages, this
 *        should be 0.
 * @param data: A pointer to the data to send. If null, a remote transmission
 *        request (RTR=1) is sent, if supported by the interface.
 * @return: True if the message was sent successfully, false otherwise.
 */
static bool sendMsg(HardwareCAN& can_intf, uint32_t id, uint8_t length, const uint8_t* data) {
    // Note: Arduino_CAN does not support the RTR bit. The ODrive interprets
    // zero-length packets the same as RTR=1, but it creates the possibility of
    // collisions.
    CanMsg msg{
        (id & 0x80000000) ? CanExtendedId(id) : CanStandardId(id),
        length,
        data,
    };
    return can_intf.write(msg) >= 0;
}

/**
 * @brief Receives a CAN message from the platform-specific interface and passes
 * it to the ODriveCAN instance.
 *
 * @param msg: The received CAN message in a platform-specific format.
 * @param odrive: The ODriveCAN instance to pass the message to.
 */
static void onReceive(const CanMsg& msg, ODriveCAN& odrive) {
    odrive.onReceive(msg.id, msg.data_length, msg.data);
}

/**
 * @brief Processes the CAN interface's RX buffer and calls onCanMessage for
 * each pending message.
 *
 * On hardware interfaces where onCanMessage() is already called from the
 * interrupt handler, this function is a no-op.
 *
 * @param intf: The platform-specific CAN interface to process.
 * @param max_events: The maximum number of events to process. This prevents
 *        an infinite loop if messages come at a high rate.
 */
static void pumpEvents(HardwareCAN& intf, int max_events = 100) {
    // max_events prevents an infinite loop if messages come at a high rate
    while (intf.available() && max_events--) {
        onCanMessage(intf.read());
    }
}

CREATE_CAN_INTF_WRAPPER(HardwareCAN)
