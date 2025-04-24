#pragma once

#include "ODriveCAN.h"
#include <STM32_CAN.h>

using CanMsg = CAN_message_t;

// Must be defined by the application
void onCanMessage(const CanMsg& msg);

static bool sendMsg(STM32_CAN& can_intf, uint32_t id, uint8_t length, const uint8_t* data) {
  CanMsg msg;
  msg.id = id & 0x1ffffff;
  msg.flags.extended = id & 0x80000000;
  msg.flags.remote = (data == nullptr);
  msg.len = length;
  if (data) {
    for (int i = 0; i < length; ++i) {
      msg.buf[i] = data[i];
    }
  }
  return can_intf.write(msg) >= 0;
}

static void onReceive(const CanMsg& msg, ODriveCAN& odrive) {
    odrive.onReceive(msg.id, msg.len, msg.buf);
}

static void pumpEvents(STM32_CAN& intf, int max_events = 100) {
    // max_events prevents an infinite loop if messages come at a high rate
    CanMsg msg;
    while (intf.read(msg) && max_events--) {
        onCanMessage(msg);
    }
}

CREATE_CAN_INTF_WRAPPER(STM32_CAN)
