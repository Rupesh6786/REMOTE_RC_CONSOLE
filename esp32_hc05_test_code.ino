// HC-05 with ESP32 using Serial2

#define RXD2 16  // ESP32 RX pin
#define TXD2 17  // ESP32 TX pin

void setup() {
  Serial.begin(115200);                         // Serial Monitor
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  // HC-05 default baud rate

  Serial.println("ESP32 Bluetooth HC-05 Ready...");
}

void loop() {
  // Check if data is coming from HC-05
  if (Serial2.available()) {
    char receivedChar = Serial2.read();
    Serial.println(receivedChar);  // Print data to Serial Monitor
  }
}
