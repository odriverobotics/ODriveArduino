#pragma once

#include <Arduino.h>
#include <string.h>
#include "ODriveCAN.h"
#include "TCAN4x5x.h"

struct CanMsg {
    uint32_t id; 
    uint8_t  len;
    uint8_t  buffer[8]; 
};

// Must be defined by the application
void onCanMessage(const CanMsg& msg);

static bool sendMsg(TCAN4x5x& can_intf, uint32_t id, uint8_t length, const uint8_t* data) {
    if (length > 8) {
        return false; 
    }
    TCAN4x5x_MCAN_TX_Header txHdr = {0};
    if (id & 0x80000000UL) {
        txHdr.XTD = 1;
        txHdr.ID  = id & 0x1FFFFFFFUL;
    } else {
        txHdr.XTD = 0;
        txHdr.ID  = id & 0x000007FFUL;
    }

    txHdr.RTR = (data == nullptr) ? 1 : 0;
    txHdr.DLC = length;
    txHdr.FDF = 0;
    txHdr.BRS = 0; 
    txHdr.ESI = 0;
    txHdr.EFC = 0;
    txHdr.MM  = 0;

    uint8_t payload[8] = {0};
    if (data != nullptr && length > 0) {
        memcpy(payload, data, length);
    }
    if (can_intf.MCAN_WriteTXBuffer(0, &txHdr, payload) == 0) {
        return false;
    }
    return can_intf.MCAN_TransmitBufferContents(0);
}

static void onReceive(const CanMsg& msg, ODriveCAN& odrive) {
    odrive.onReceive(msg.id, msg.len, msg.buffer);
}

static void pumpEvents(TCAN4x5x& can_intf) {
    while (true) {
        TCAN4x5x_MCAN_RX_Header rxHdr = {0};
        uint8_t rxData[64] = {0};
        rxHdr.DLC = 0x0F;
        uint8_t numBytes = can_intf.MCAN_ReadNextFIFO(RXFIFO0, &rxHdr, rxData);

        if (numBytes == 0 && rxHdr.DLC == 0x0F) {
            break; 
        }

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

CREATE_CAN_INTF_WRAPPER(TCAN4x5x)
