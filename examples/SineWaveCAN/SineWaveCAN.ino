#include <Arduino.h>
#include "ODriveCAN.h"

// Documentation for this example can be found here:
// https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html

/* Configuration of example sketch -------------------------------------------*/

// CAN bus baudrate. Make sure this matches for every device on the bus
#define CAN_BAUDRATE 250000

// ODrive node_id for odrv0
#define ODRV0_NODE_ID 0

// Uncomment below the line that corresponds to your hardware.
// See also "Board-specific settings" to adapt the details for your hardware setup.

// #define IS_TEENSY_BUILTIN // Teensy boards with built-in CAN interface (e.g. Teensy 4.1). See below to select which interface to use.
// #define IS_ARDUINO_BUILTIN // Arduino boards with built-in CAN interface (e.g. Arduino Uno R4 Minima)
// #define IS_MCP2515 // Any board with external MCP2515 based extension module. See below to configure the module.
// #define IS_STM32_BUILTIN // STM32 boards with built-in CAN interface (e.g. STM32F4 Discovery).
// #define IS_ESP32_TWAI // ESP32 boards with built-in TWAI (CAN) interface. Directly uses the ESP-IDF TWAI driver.
// #define IS_TCAN4X5X // RP2350 / Arduino boards with external TCAN4551 SPI controller.

/* Board-specific includes ---------------------------------------------------*/

#if defined(IS_TEENSY_BUILTIN) + defined(IS_ARDUINO_BUILTIN) + defined(IS_MCP2515) + defined(IS_STM32_BUILTIN) + defined(IS_ESP32_TWAI) + defined(IS_TCAN4X5X) != 1
#warning "Select exactly one hardware option at the top of this file."

#if CAN_HOWMANY > 0 || CANFD_HOWMANY > 0
#define IS_ARDUINO_BUILTIN
#warning "guessing that this uses HardwareCAN"
#else
#error "cannot guess hardware version"
#endif

#endif

#ifdef IS_ARDUINO_BUILTIN
// See https://github.com/arduino/ArduinoCore-API/blob/master/api/HardwareCAN.h
// and https://github.com/arduino/ArduinoCore-renesas/tree/main/libraries/Arduino_CAN

#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>
#endif // IS_ARDUINO_BUILTIN

#ifdef IS_MCP2515
// See https://github.com/sandeepmistry/arduino-CAN/
#include "MCP2515.h"
#include "ODriveMCPCAN.hpp"
#endif // IS_MCP2515

#ifdef IS_TEENSY_BUILTIN
// See https://github.com/tonton81/FlexCAN_T4
// clone https://github.com/tonton81/FlexCAN_T4.git into /src
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // hack to prevent teensy compile error
#endif // IS_TEENSY_BUILTIN

#ifdef IS_STM32_BUILTIN
// See https://github.com/pazi88/STM32_CAN
#include <STM32_CAN.h>
#include "ODriveSTM32CAN.hpp"
#endif // IS_STM32_BUILTIN

#ifdef IS_ESP32_TWAI
// See https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/twai.html
#include "driver/twai.h"
#include "ODriveESP32TWAI.hpp"
#endif // IS_ESP32_TWAI

#ifdef IS_TCAN4X5X
#include <SPI.h>
#include <TCAN4x5x.h>
#include "ODriveTCAN4x5x.hpp"
#endif // IS_TCAN4X5X

/* Board-specific settings ---------------------------------------------------*/

/* Teensy */

#ifdef IS_TEENSY_BUILTIN

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

bool setupCan() {
can_intf.begin();
can_intf.setBaudRate(CAN_BAUDRATE);
can_intf.setMaxMB(16);
can_intf.enableFIFO();
can_intf.enableFIFOInterrupt();
can_intf.onReceive(onCanMessage);
return true;
}

#endif // IS_TEENSY_BUILTIN

/* MCP2515-based extension modules -*/

#ifdef IS_MCP2515

MCP2515Class& can_intf = CAN;

// chip select pin used for the MCP2515
#define MCP2515_CS 10

// interrupt pin used for the MCP2515
// NOTE: not all Arduino pins are interruptable, check the documentation for your board!
#define MCP2515_INT 2

// freqeuncy of the crystal oscillator on the MCP2515 breakout board.
// common values are: 16 MHz, 12 MHz, 8 MHz
#define MCP2515_CLK_HZ 8000000

static inline void receiveCallback(int packet_size) {
if (packet_size > 8) {
return; // not supported
}
CanMsg msg = {.id = (unsigned int)CAN.packetId(), .len = (uint8_t)packet_size};
CAN.readBytes(msg.buffer, packet_size);
onCanMessage(msg);
}

bool setupCan() {
// configure and initialize the CAN bus interface
CAN.setPins(MCP2515_CS, MCP2515_INT);
CAN.setClockFrequency(MCP2515_CLK_HZ);
if (!CAN.begin(CAN_BAUDRATE)) {
return false;
}

CAN.onReceive(receiveCallback);
return true;
}

#endif // IS_MCP2515

/* Arduinos with built-in CAN */

#ifdef IS_ARDUINO_BUILTIN

HardwareCAN& can_intf = CAN;

bool setupCan() {
return can_intf.begin((CanBitRate)CAN_BAUDRATE);
}

#endif

/* STM32 boards with built-in CAN */

#ifdef IS_STM32_BUILTIN

STM32_CAN Can1( CAN1 );
STM32_CAN& can_intf = Can1;

bool setupCan() {
can_intf.begin();
can_intf.setBaudRate(CAN_BAUDRATE);
return true;
}

#endif // IS_STM32_BUILTIN

/* ESP32 boards with built-in TWAI (CAN) */

#ifdef IS_ESP32_TWAI

// Pins used to connect to CAN bus transceiver
#define ESP32_TWAI_TX_PIN 26
#define ESP32_TWAI_RX_PIN 25

ESP32TWAIIntf can_intf;

bool setupCan() {
twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
(gpio_num_t)ESP32_TWAI_TX_PIN,
(gpio_num_t)ESP32_TWAI_RX_PIN,
TWAI_MODE_NORMAL
);

twai_timing_config_t t_config;
switch (CAN_BAUDRATE) {
    case 1000000: t_config = TWAI_TIMING_CONFIG_1MBITS(); break;
    case 800000:  t_config = TWAI_TIMING_CONFIG_800KBITS(); break;
    case 500000:  t_config = TWAI_TIMING_CONFIG_500KBITS(); break;
    case 250000:  t_config = TWAI_TIMING_CONFIG_250KBITS(); break;
    case 125000:  t_config = TWAI_TIMING_CONFIG_125KBITS(); break;
    case 100000:  t_config = TWAI_TIMING_CONFIG_100KBITS(); break;
    case 50000:   t_config = TWAI_TIMING_CONFIG_50KBITS(); break;
    case 25000:   t_config = TWAI_TIMING_CONFIG_25KBITS(); break;
    default:      t_config = TWAI_TIMING_CONFIG_250KBITS(); break;
}

twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    return false;
}

if (twai_start() != ESP_OK) {
    twai_driver_uninstall();
    return false;
}

return true;


}

#endif // IS_ESP32_TWAI

/* TCAN4551 SPI Controller */

#ifdef IS_TCAN4X5X

const uint8_t MISO_PIN = 16;
const uint8_t CS_PIN   = 17;
const uint8_t SCK_PIN  = 18;
const uint8_t MOSI_PIN = 19;
const uint8_t INT1_PIN = 20;
const uint8_t INT2_PIN = 21;
const uint8_t RST_PIN  = 0xFF;

TCAN4x5x can_intf(CS_PIN, MISO_PIN, MOSI_PIN, SCK_PIN, RST_PIN);

volatile bool tcanIrqAsserted = false;
void IRAM_ATTR onTcanPinFall() {
tcanIrqAsserted = true;
}

bool setupCan() {
uint16_t prescaler = (uint16_t)(1000000UL / CAN_BAUDRATE);
if (prescaler < 1) prescaler = 1;
can_intf.setNominalBitRate(prescaler, 32, 8);

TCAN4x5x_MRAM_Config customMRAM = {0};
customMRAM.SIDNumElements      = 1;
customMRAM.XIDNumElements      = 1;
customMRAM.Rx0NumElements      = 16;
customMRAM.Rx0ElementSize      = MRAM_8_Byte_Data;
customMRAM.TxBufferNumElements = 4;
customMRAM.TxBufferElementSize = MRAM_8_Byte_Data;
can_intf.setMRAMConfig(&customMRAM);

TCAN4x5x_MCAN_Global_Filter_Configuration gfc = {0};
gfc.RRFE = 1;
gfc.RRFS = 1;
gfc.ANFE = TCAN4x5x_GFC_ACCEPT_INTO_RXFIFO0;
gfc.ANFS = TCAN4x5x_GFC_ACCEPT_INTO_RXFIFO0;
can_intf.setGlobalFilterConfig(&gfc);

TCAN4x5x_DEV_CONFIG devCfg = {0};
devCfg.SWE_DIS          = 1;
devCfg.CLK_REF          = 1;
devCfg.GPIO1_CONFIG     = TCAN4x5x_DEV_CONFIG_GPIO1_CONFIG_GPO;
devCfg.GPIO1_GPO_CONFIG = TCAN4x5x_DEV_CONFIG_GPO1_MCAN_INT1;
devCfg.GPO2_CONFIG      = TCAN4x5x_DEV_CONFIG_GPO2_MCAN_INT0;
devCfg.WD_ACTION        = TCAN4x5x_DEV_CONFIG_WDT_ACTION_nINT;
devCfg.WAKE_CONFIG      = TCAN4x5x_DEV_CONFIG_WAKE_BOTH_EDGES;
can_intf.setDeviceConfig(&devCfg);

can_intf.attachInterruptPin(INT1_PIN, onTcanPinFall);
can_intf.attachInterruptPin(INT2_PIN, onTcanPinFall);

can_intf.begin(&SPI);

TCAN4x5x_MCAN_CCCR_Config cccr = {0};
can_intf.MCAN_EnableProtectedRegisters();
can_intf.MCAN_ReadCCCRRegister(&cccr);
cccr.FDOE = 0;
cccr.BRSE = 0;
can_intf.MCAN_ConfigureCCCRRegister(&cccr);
can_intf.MCAN_DisableProtectedRegisters();

can_intf.WDT_Disable();
can_intf.Device_SetMode(TCAN4x5x_DEVICE_MODE_NORMAL);

return (can_intf.Device_ReadDeviceVersion() != 0xFFFF && can_intf.Device_ReadMode() == TCAN4x5x_DEVICE_MODE_NORMAL);
}

#endif // IS_TCAN4X5X

/* Example sketch ------------------------------------------------------------*/

// Instantiate ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
ODriveCAN* odrives[] = {&odrv0}; // Make sure all ODriveCAN instances are accounted for here

struct ODriveUserData {
Heartbeat_msg_t last_heartbeat;
bool received_heartbeat = false;
Get_Encoder_Estimates_msg_t last_feedback;
bool received_feedback = false;
};

// Keep some application-specific user data for every ODrive.
ODriveUserData odrv0_user_data;

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
odrv_user_data->last_heartbeat = msg;
odrv_user_data->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
odrv_user_data->last_feedback = msg;
odrv_user_data->received_feedback = true;
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
for (auto odrive: odrives) {
onReceive(msg, *odrive);
}
}

void setup() {
Serial.begin(115200);

// Wait for up to 3 seconds for the serial port to be opened on the PC side.
// If no PC connects, continue anyway.
for (int i = 0; i < 30 && !Serial; ++i) {
delay(100);
}
delay(200);

Serial.println("Starting ODriveCAN demo");

// Register callbacks for the heartbeat and encoder feedback messages
odrv0.onFeedback(onFeedback, &odrv0_user_data);
odrv0.onStatus(onHeartbeat, &odrv0_user_data);

// Configure and initialize the CAN bus interface. This function depends on
// your hardware and the CAN stack that you're using.
if (!setupCan()) {
Serial.println("CAN failed to initialize: reset required");
while (true); // spin indefinitely
}

Serial.println("Waiting for ODrive...");
while (!odrv0_user_data.received_heartbeat) {
pumpEvents(can_intf);
}

Serial.println("found ODrive");

// request bus voltage and current (1sec timeout)
Serial.println("attempting to read bus voltage and current");
Get_Bus_Voltage_Current_msg_t vbus;
if (!odrv0.request(vbus, 1000)) {
Serial.println("vbus request failed!");
while (true); // spin indefinitely
}

Serial.print("DC voltage [V]: ");
Serial.println(vbus.Bus_Voltage);
Serial.print("DC current [A]: ");
Serial.println(vbus.Bus_Current);

Serial.println("Enabling closed loop control...");
while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
odrv0.clearErrors();
delay(1);
odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

// Pump events for 150ms. This delay is needed for two reasons;
// 1. If there is an error condition, such as missing DC power, the ODrive might
//    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
//    on the first heartbeat response, so we want to receive at least two
//    heartbeats (100ms default interval).
// 2. If the bus is congested, the setState command won't get through
//    immediately but can be delayed.
for (int i = 0; i < 15; ++i) {
  delay(10);
  pumpEvents(can_intf);
}


}

Serial.println("ODrive running!");
}

void loop() {
pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages
// Note that on MCP2515-based platforms, this will delay for a fixed 10ms.
//
// This has been found to reduce the number of dropped messages, however it can be removed
// for applications requiring loop times over 100Hz.

float SINE_PERIOD = 2.0f; // Period of the position command sine wave in seconds

float t = 0.001 * millis();

float phase = t * (TWO_PI / SINE_PERIOD);

odrv0.setPosition(
sin(phase), // position
cos(phase) * (TWO_PI / SINE_PERIOD) // velocity feedforward (optional)
);

// print position and velocity for Serial Plotter
if (odrv0_user_data.received_feedback) {
Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
odrv0_user_data.received_feedback = false;
Serial.print("odrv0-pos:");
Serial.print(feedback.Pos_Estimate);
Serial.print(",");
Serial.print("odrv0-vel:");
Serial.println(feedback.Vel_Estimate);
}
}
