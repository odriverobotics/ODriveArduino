
#pragma once

#include <stdint.h>
#include <string.h>

struct can_Message_t {
    uint32_t id         = 0x000;
    uint8_t len         = 0;
    bool is_extended_id = false;
    bool rtr            = false;
    uint8_t data[8]      = {0};
};

template <typename T, size_t N>
T can_getSignal(const uint8_t (&buf)[N], const size_t startBit, const size_t length, const bool isIntel) {
    union {
        uint64_t tempVal;
        uint8_t tempBuf[N];  // This is used because memcpy into tempVal generates less optimal code
        T retVal;
    };

    const uint64_t mask = length < 64 ? (1ULL << length) - 1ULL : -1ULL;
    const uint8_t shift = isIntel ? startBit : (64 - startBit) - length;

    memcpy(tempBuf, buf, N);
    if (isIntel) {
        tempVal = (tempVal >> shift) & mask;
    } else {
        tempVal = __builtin_bswap64(tempVal);
        tempVal = (tempVal >> shift) & mask;
    }

    return retVal;
}

template <typename T, size_t N>
void can_setSignal(uint8_t (&buf)[N], const T& val, const size_t startBit, const size_t length, const bool isIntel) {
    const uint64_t mask = length < 64 ? (1ULL << length) - 1ULL : -1ULL;
    const uint8_t shift = isIntel ? startBit : (64 - startBit) - length;

    union {
        uint64_t valAsBits;
        T tempVal;
    };

    tempVal = val;

    union {
        uint64_t data;
        uint8_t tempBuf[N];
    };

    memcpy(tempBuf, buf, N);
    if (isIntel) {
        data &= ~(mask << shift);
        data |= valAsBits << shift;
    } else {
        data = __builtin_bswap64(data);
        data &= ~(mask << shift);
        data |= valAsBits << shift;
        data = __builtin_bswap64(data);
    }

    memcpy(buf, tempBuf, N);
}

template <typename T, size_t N>
float can_getSignal(const uint8_t (&buf)[N], const size_t startBit, const size_t length, const bool isIntel, const float factor, const float offset) {
    T retVal = can_getSignal<T>(buf, startBit, length, isIntel);
    return (retVal * factor) + offset;
}

template <typename T, size_t N>
void can_setSignal(uint8_t (&buf)[N], const float& val, const size_t startBit, const size_t length, const bool isIntel, const float factor, const float offset) {
    T scaledVal = static_cast<T>((val - offset) / factor);
    can_setSignal<T>(buf, scaledVal, startBit, length, isIntel);
}
