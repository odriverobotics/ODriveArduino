#ifndef TWAI_CAN_CLASS_H
#define TWAI_CAN_CLASS_H

//#include "driver/twai.h"

class TWAIClass {
public:
    TWAIClass() : is_initialized(false) {}
    ~TWAIClass() {
        if (is_initialized) {
            end();
        }
    }
	
bool begin(long baudRate) {
    if (is_initialized) return false;  // Already initialized
    
    // Configure TWAI (CAN) settings
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
    //TODO ADD SWITCH CASE FOR DIFFERENT BAUDRATE****************************************************************************************************************
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();  // Ensure it matches ODrive
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install the TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("❌ TWAI Driver install failed!");
        return false;
    }

    // Start TWAI driver
    if (twai_start() != ESP_OK) {
        Serial.println("❌ TWAI Driver start failed!");
        twai_driver_uninstall();
        return false;
    }

    // Configure TWAI Alerts (Monitor TX and Bus Errors)
    uint32_t alerts_to_enable = TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | 
                                TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR;
    if (twai_reconfigure_alerts(alerts_to_enable, NULL) != ESP_OK) {
        Serial.println("❌ Failed to configure CAN Alerts!");
        return false;
    }

    Serial.println("✅ TWAI CAN Bus Started Successfully!");
    is_initialized = true;
    return true;
}

    // End CAN communication
    void end() {
        if (is_initialized) {
            twai_stop();
            twai_driver_uninstall();
            is_initialized = false;
        }
    }

    // Send a CAN message
    int endPacket() {
        if (!is_initialized || !message_ready) {
            return 0; // Not initialized or no message ready
        }

        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK) {
            message_ready = false;
            return 1; // Successfully sent message
        }

        return 0; // Failed to send message
    }

    // Prepare a CAN messagewrite
    void prepareMessage(uint32_t id, uint8_t length, bool rtr = false) {
        tx_message.identifier = id;
        tx_message.data_length_code = length;
        //tx_message.rtr = rtr ? TWAI_RTR : TWAI_NO_RTR; //TODO this line doesnt compile. verify if required for odrive*******************************************

		tx_message.rtr = 0;
        tx_message.extd = (id > 0x7FF); // Extended frame if ID > 11 bits
        message_ready = true;
    }

    // Write data to the CAN message
    void write(uint8_t byte, int index) {
        if (index >= 0 && index < 8) {
            tx_message.data[index] = byte;
        }
    }

    // Parse incoming CAN packet
    int parsePacket() {
        if (!is_initialized) {
            return 0; // Not initialized
        }

        if (twai_receive(&rx_message, pdMS_TO_TICKS(10)) == ESP_OK) {
            return rx_message.data_length_code; // Number of bytes received
        }

        return 0; // No packet received
    }

    // Access packet ID
    uint32_t packetId() {
        return rx_message.identifier;
    }

    // Access received data
    void readBytes(uint8_t* buffer, uint8_t length) {
      if (length > 8) length = 8; // Maximum data length
      for (uint8_t i = 0; i < length; ++i) {
          buffer[i] = rx_message.data[i];
          //Serial.printf(" %d = %02x,", i, rx_message.data[i]); // DEBUG PRINT uncomment to print all messages to serial console **********************************
      }
      //Serial.println(""); //uncomment to print all messages to serial console **********************************
    }

    // Callback for received messages
    void onReceive(void (*callback)(int)) {
        receive_callback = callback;
    }

private:
    bool is_initialized;
    bool message_ready = false;
    twai_message_t tx_message;
    twai_message_t rx_message;
    void (*receive_callback)(int) = nullptr;
};

extern TWAIClass CAN;

#endif // TWAI_CAN_CLASS_H
