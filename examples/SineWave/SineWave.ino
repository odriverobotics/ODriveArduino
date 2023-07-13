 
#include <ODriveArduino.h>
#include <SoftwareSerial.h>

SoftwareSerial odrive_serial(8, 9); // RX (ODrive TX), TX (ODrive RX)
ODriveArduino odrive(odrive_serial);

void setup() {
  // This baudrate must match the baudrate configured on the ODrive.
  // Note that the default ODrive baudrate is 115200, so this example requires
  // to change the baudrate on the ODrive accordingly (see docs for explanation),
  odrive_serial.begin(19200);

  Serial.begin(115200);
  
  delay(10);

  Serial.println("Waiting for ODrive...");
  while (odrive.getState() == AXIS_STATE_UNDEFINED) {
    delay(100);
  }
  
  Serial.print("DC voltage: ");
  Serial.println(odrive.getParameterAsFloat("vbus_voltage"));
  
  Serial.println("Enabling closed loop control...");
  while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrive.clearErrors();
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(10);
  }
  
  Serial.println("ODrive running!");
}

void loop() {
  float SINE_PERIOD = 2.0f; // Period of the position command sine wave in seconds

  float t = 0.001 * millis();
  
  float phase = t * (TWO_PI / SINE_PERIOD);
  
  odrive.setPosition(
    sin(phase), // position
    cos(phase) * (TWO_PI / SINE_PERIOD) // velocity feedforward (optional)
  );

  ODriveFeedback feedback = odrive.getFeedback();
  Serial.print("pos:");
  Serial.print(feedback.pos);
  Serial.print(", ");
  Serial.print("vel:");
  Serial.print(feedback.vel);
  Serial.println();
}
