#include <BluetoothSerial.h>
// Define the serial port and baud rate
BluetoothSerial SerialBT;
#define Serial1_TX 17
#define Serial1_RX 16
#define BAUD_RATE 9600

void setup() {
  SerialBT.begin("ESP32_BT_Serial");
  Serial1.begin(BAUD_RATE, SERIAL_8N1, Serial1_RX, Serial1_TX);
}

void loop() {
  // Get the current values of Kp, Ki, Kd, and setpoint
  if (SerialBT.available()) {
  String data = SerialBT.readStringUntil('\n');
  Serial1.println(data);
  }

}

