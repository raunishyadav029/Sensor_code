#include <ModbusMaster.h>

#define RE_DE_PIN 25  // RS485 control pin
#define RX_PIN 27   // RS485 RO -> ESP32 RX
#define TX_PIN 26    // RS485 DI -> ESP32 TX

ModbusMaster node;

void preTransmission() {
  digitalWrite(RE_DE_PIN, HIGH);  // Enable Transmit Mode
}

void postTransmission() {
  digitalWrite(RE_DE_PIN, LOW);   // Enable Receive Mode
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW); // Start in receive mode

  node.begin(4, Serial2); // Set Modbus Slave ID to 4
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

void loop() {
  uint8_t result;
  uint16_t illuminationValue;

  result = node.readHoldingRegisters(0x0000, 1); // Reading register 0x0000

  if (result == node.ku8MBSuccess) {
    illuminationValue = node.getResponseBuffer(0);
    float lux = illuminationValue / 10.0;  // Convert as per datasheet
    Serial.print("Illumination Level: ");
    Serial.print(lux);
    Serial.println(" Lux");
  } else {
    Serial.println("Failed to read from sensor!");
  }

  delay(1000); // Read every second
}
