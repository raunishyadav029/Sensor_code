#include <Arduino.h>
#include <ModbusMaster.h>

// Define RS485 Control Pins
#define RE_DE 25
#define TX_PIN 26
#define RX_PIN 27

// Define Soil Moisture Sensor Modbus Address
#define SENSOR_ADDRESS 0x05

// Initialize Modbus
ModbusMaster node;

void preTransmission() {
  digitalWrite(RE_DE, HIGH);  // Enable Transmit mode
}

void postTransmission() {
  digitalWrite(RE_DE, LOW);   // Enable Receive mode
}

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing...");

  // RS485 Setup
  pinMode(RE_DE, OUTPUT);
  digitalWrite(RE_DE, LOW);  // Set to Receive mode
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Attach Modbus
  node.begin(SENSOR_ADDRESS, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

void loop() {
  uint8_t result;
  uint16_t moisture = 0, conductivity = 0, temperature = 0;

  // Read Soil Moisture (% VWC) from Register 0x0000
  result = node.readHoldingRegisters(0x0000, 1);
  if (result == node.ku8MBSuccess) {
    moisture = node.getResponseBuffer(0);
  } else {
    Serial.println("⚠️ Failed to read Moisture!");
  }

  // Read Electrical Conductivity (µS/cm) from Register 0x0001
  result = node.readHoldingRegisters(0x0001, 1);
  if (result == node.ku8MBSuccess) {
    conductivity = node.getResponseBuffer(0);
  } else {
    Serial.println("⚠️ Failed to read Electrical Conductivity!");
  }

  // Read Soil Temperature (°C) from Register 0x0002
  result = node.readHoldingRegisters(0x0002, 1);
  if (result == node.ku8MBSuccess) {
    temperature = node.getResponseBuffer(0);
  } else {
    Serial.println("⚠️ Failed to read Temperature!");
  }

  // Display readings
  Serial.print("Soil Moisture: ");
  Serial.print(moisture);
  Serial.println(" %");

  Serial.print("Electrical Conductivity: ");
  Serial.print(conductivity);
  Serial.println(" µS/cm");

  Serial.print("Soil Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.println("----------------------------");
  
  delay(5000);  // Read every 5 seconds
}
