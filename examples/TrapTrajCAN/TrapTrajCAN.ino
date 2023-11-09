// clone https://github.com/tonton81/FlexCAN_T4.git into /src and uncomment below
// #define IS_TEENSY

#include <Arduino.h>
#include "ODriveEnums.h"

#ifdef IS_TEENSY

#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"

typedef FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CANInft;
CANInft can0;

#else

#include "MCP2515.h"
#include "ODriveMCPCAN.hpp"

typedef MCP2515Class CANInft;
CANInft can0 = CAN;

// chip select pin used for the MCP2515
#define MCP2515_CS 10

// interrupt pin used for the MCP2515
// NOTE: not all Aruino pins are interruptable, check the documentation for your board!
#define MCP2515_INT 2

// freqeuncy of the crystal oscillator on the MCP2515 breakout board. 
// common values are: 16E6, 12E6, 8E6
#define MCP2515_CLK_HZ 8000000

#endif

//========================================================================
// Macro configuration for ODriveCAN Trapazoidal Trajectory Example

// CAN bus baudrate. Make sure this matches for every device on the bus
#define CAN_BAUDRATE 250000

// rate of vbus message serial prints
#define ODRV0_VBUS_RATE_MS 1700

// ODrive node_id for odrv0
#define ODRV0_NODE_ID 0
#define ODRV0_IS_EXTENDED false

//========================================================================


//========================================================================
// required ODrive Configuration:
// odrv.config.enable_can_a = True
// odrv.can.config.baud_rate = <CAN_BAUDRATE>
// odrv.axis0.config.can.node_id = <ODRV{i}_NODE_ID>
// odrv.axis0.config.can.is_extended = <ODRV{i}_IS_EXTENDED>
// odrv.axis0.config.can.encoder_msg_rate_ms != 0, suggest 25
// odrv.axis0.config.can.heartbeat_msg_rate_ms != 0, suggest 250
//========================================================================


//========================================================================
// ODriveCAN object(s) initialization

ODriveCAN<CANInft> odrv0(can0, ODRV0_NODE_ID, ODRV0_IS_EXTENDED); // Standard CAN message ID
// ODriveCAN odrv1(0xabc, true); // Extended CAN message ID
//========================================================================


//========================================================================
// boiler plate CAN Controller callback

ODriveCAN<CANInft>* odrives[] = {&odrv0}; //, &odrv1}; // Make sure all ODriveCAN instances are accounted for here!

#ifdef IS_TEENSY
void receiveCallback(const CAN_message_t& flex_msg) {
    can_Message_t msg = ODriveCAN<CANInft>::formatPacket(flex_msg);
#else
void receiveCallback(int packet_size) {
    can_Message_t msg = ODriveCAN<CANInft>::formatPacket(packet_size, can0);
#endif // IS_TEENSY
    for (auto odrive: odrives) {
        odrive->onReceive(msg);
    }
    // insert any additional CA N message handling here
}
//========================================================================


//========================================================================
struct ODriveEstimates {
    Get_Encoder_Estimates_msg_t estimates;
    bool _fresh = false;
    bool fresh() {
        if (_fresh) {
            _fresh = false;
            return true;
        }
        return false;
    }
};

ODriveEstimates odrv0_estimates;

void processEncoderFeedback(Get_Encoder_Estimates_msg_t& feedback, void* feeback_user_data) {
    ODriveEstimates* odrv_estimates = static_cast<ODriveEstimates*>(feeback_user_data);
    // process encoder estimates 
    memcpy(&odrv_estimates->estimates, &feedback, sizeof(Get_Encoder_Estimates_msg_t));
    odrv_estimates->_fresh = true;
}
//========================================================================


//========================================================================
// demonstration for axis status callback.
struct ODriveStatus {
    Heartbeat_msg_t heartbeat;
    bool _fresh = false;
    bool fresh() {
        if (_fresh) {
            _fresh = false;
            return true;
        }
        return false;
    }
};

ODriveStatus odrv0_status;

void processAxisStatus(Heartbeat_msg_t& feedback, void* axis_status_user_data) {
    ODriveStatus* odrv_status = static_cast<ODriveStatus*>(axis_status_user_data);
    // process axis state
    memcpy(&odrv_status->heartbeat, &feedback, sizeof(Heartbeat_msg_t));
    odrv_status->_fresh = true;
}
//========================================================================

void odrive_startup(ODriveCAN<CANInft>& odrv) {

    Serial.println("attempting to read bus voltage and current");

    Get_Bus_Voltage_Current_msg_t vbus;
    
    // request bus voltage and current (1sec timeout)
    if (odrv0.request(vbus, 1)) {
        Serial.println("Bus V / C");
        Serial.println(vbus.Bus_Voltage);
        Serial.println(vbus.Bus_Current);
    }
    else Serial.println("vbus request failed!");
    
    // make sure odrv0 is configured for trajectory commands
    if (!odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_TRAP_TRAJ)) {
        Serial.println("failed to set Controller Mode");
    }
}

void enter_closed_loop_control(ODriveCAN<CANInft>& odrv, ODriveStatus& odrv_status) {
    while (odrv_status.heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        Serial.println("Attempting to enter CLOSED LOOP CONTROL");

        while (!odrv_status.fresh()) odrv.events();

        // dump errors
        Serial.print("  Axis Error - "); Serial.println(odrv_status.heartbeat.Axis_Error);
        Serial.print("  Axis State - "); Serial.println(odrv_status.heartbeat.Axis_State);
        Serial.print("  Procedure Result - "); Serial.println(odrv_status.heartbeat.Procedure_Result);
        Serial.print("  Trajectory Done Flag - "); Serial.println(odrv_status.heartbeat.Trajectory_Done_Flag);

        if (!odrv.clearErrors()) Serial.println("failed to clear errors");
        if (!odrv.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)) Serial.println("failed to set Axis State");
    }
    Serial.println("Success!");
}

void setup() {

    Serial.begin(115200);
    while (!Serial);

    Serial.println("Starting ODriveCAN demo");

    // set ODriveCAN callbacks for the encoder feedback and the axis status
    odrv0.onFeedback(processEncoderFeedback, &odrv0_estimates);
    odrv0.onStatus(processAxisStatus, &odrv0_status);

#ifdef IS_TEENSY
 // configure and initialize the CAN bus interface
    can0.begin();
    can0.setBaudRate(CAN_BAUDRATE);
    can0.setMaxMB(16);
    can0.enableFIFO();
    can0.enableFIFOInterrupt();
#else
    // configure and initialize the CAN bus interface
    can0.setPins(MCP2515_CS, MCP2515_INT);
    can0.setClockFrequency(MCP2515_CLK_HZ);
    if (!can0.begin(CAN_BAUDRATE)) {
        Serial.println("CAN failed to initialize: reset required");
        while (true); // spin indefinitely
    }
#endif //IS_TEENSY

    can0.onReceive(receiveCallback); // attach boiler plate CAN callback
    Serial.println("CAN initialized");

    odrive_startup(odrv0);
}

bool fwd_rev = false;

void loop() {

    if (!Serial) { // exit closed loop control if the terminal closes
        odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
        return; // skip below if serial communication is lost
    }

    odrv0.events();

    if (odrv0_status.heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        enter_closed_loop_control(odrv0, odrv0_status);
        return; // skipp below if not in closed loop control
    }

    // print position and velocity for Serial Plotter
    if (odrv0_estimates.fresh()) {
        Serial.print("odrv0-pos:");
        Serial.print(odrv0_estimates.estimates.Pos_Estimate);
        Serial.print(",");
        Serial.print("odrv0-vel:");
        Serial.println(odrv0_estimates.estimates.Vel_Estimate);
    }
    
    if (!odrv0_status.fresh()) return; // skip below if heartbeat is old

    if (!odrv0_status.heartbeat.Trajectory_Done_Flag) return; // skip below if trajectory is in progress

    // set new target pos for the trajectory planner
    float target_pos = (fwd_rev) ? 5.0f : -5.0f;
    fwd_rev ^= true;
    if (!odrv0.trapezoidalMove(target_pos)) Serial.println("trapezoidalMove command failed!");

}
