#pragma once

#include <Arduino.h>
#include <string.h>
#include "ODriveCAN.h"
#include "TCAN4x5x.h"

// Classical CAN frame container used by ODriveArduino sketches
struct CanMsg {
    uint32_t id;        // Bit 31 (0x80000000) indicates 29-bit Extended ID
    uint8_t  len;       // 0..8 bytes
    uint8_t  buffer[8]; // Classical CAN payload
};

// Implemented in the main .ino sketch to dispatch frames to ODriveCAN instances
void onCanMessage(const CanMsg& msg);

// 1. Transmit function called by ODriveCAN using your library's exact method names
static bool sendMsg(TCAN4x5x& can_intf, uint32_t id, uint8_t length, const uint8_t* data) {
    if (length > 8) {
        return false; // ODrive uses Classical CAN (max 8 bytes)
    }

    TCAN4x5x_MCAN_TX_Header txHdr = {0};

    // ODriveCAN sets bit 31 (0x80000000) if the frame uses a 29-bit Extended ID
    if (id & 0x80000000UL) {
        txHdr.XTD = 1;
        txHdr.ID  = id & 0x1FFFFFFFUL;
    } else {
        txHdr.XTD = 0;
        txHdr.ID  = id & 0x000007FFUL; // MCAN_WriteTXBuffer shifts standard IDs << 18 internally
    }

    // ODriveCAN passes data == nullptr when sending an RTR request (e.g. odrv0.request())
    txHdr.RTR = (data == nullptr) ? 1 : 0;
    txHdr.DLC = length;
    txHdr.FDF = 0; // Must be 0 (Classical CAN) for ODrive
    txHdr.BRS = 0; // Must be 0 (No Bit Rate Switching)
    txHdr.ESI = 0;
    txHdr.EFC = 0;
    txHdr.MM  = 0;

    // Provide a safe zeroed buffer when data == nullptr so MCAN_WriteTXBuffer doesn't fault
    uint8_t payload[8] = {0};
    if (data != nullptr && length > 0) {
        memcpy(payload, data, length);
    }

    // Use your library's exact methods: MCAN_WriteTXBuffer & MCAN_TransmitBufferContents
    if (can_intf.MCAN_WriteTXBuffer(0, &txHdr, payload) == 0) {
        return false;
    }
    return can_intf.MCAN_TransmitBufferContents(0);
}

// 2. Pass a received CanMsg into an ODriveCAN instance
static void onReceive(const CanMsg& msg, ODriveCAN& odrive) {
    odrive.onReceive(msg.id, msg.len, msg.buffer);
}

// 3. Event pump calling your library's exact method: MCAN_ReadNextFIFO
static void pumpEvents(TCAN4x5x& can_intf) {
    while (true) {
        TCAN4x5x_MCAN_RX_Header rxHdr = {0};
        uint8_t rxData[64] = {0}; // Safety buffer matching MRAM configuration

        // Sentinel value to detect empty FIFO
        rxHdr.DLC = 0x0F;
        uint8_t numBytes = can_intf.MCAN_ReadNextFIFO(RXFIFO0, &rxHdr, rxData);

        if (numBytes == 0 && rxHdr.DLC == 0x0F) {
            break; // FIFO is completely empty
        }

        // Ignore CAN-FD frames, Remote frames, or payloads > 8 bytes
        if (rxHdr.FDF || rxHdr.RTR || numBytes > 8) {
            continue;
        }

        CanMsg msg = {0};
        msg.id  = rxHdr.ID;
        if (rxHdr.XTD) {
            msg.id |= 0x80000000UL;
        }
        msg.len = numBytes;
        if (numBytes > 0) {
            memcpy(msg.buffer, rxData, numBytes);
        }

        onCanMessage(msg);
    }
}

// 4. Generate wrap_can_intf(TCAN4x5x&) for ODriveCAN
CREATE_CAN_INTF_WRAPPER(TCAN4x5x)