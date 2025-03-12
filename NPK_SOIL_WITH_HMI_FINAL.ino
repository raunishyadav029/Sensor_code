#include <ModbusMaster.h>
#include <Wire.h>
#include <HardwareSerial.h>

#define MAX485_DE  25  // RS485 Direction Control Pin (DE)
#define MAX485_RE  25  // RS485 Direction Control Pin (RE)
#define RS485_RX   16  // ESP32 RX (Receive Data from RS485)
#define RS485_TX   17  // ESP32 TX (Send Data to RS485)

#define HMI_TX 26  // HMI Display TX Pin
#define HMI_RX 27  // HMI Display RX Pin

// Create Modbus object for ESP32 (Master)
ModbusMaster node;
HardwareSerial HMI(1);  // Using Serial1 for HMI communication

// Function to manage RS485 transmission
void preTransmission() {
  digitalWrite(MAX485_DE, HIGH);
  digitalWrite(MAX485_RE, HIGH);
}

void postTransmission() {
  digitalWrite(MAX485_DE, LOW);
  digitalWrite(MAX485_RE, LOW);
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);  // Initialize RS485 at 9600 baud
  HMI.begin(9600, SERIAL_8N1, HMI_RX, HMI_TX);  // Initialize HMI display

  // RS485 Control Pins
  pinMode(MAX485_DE, OUTPUT);
  pinMode(MAX485_RE, OUTPUT);
  digitalWrite(MAX485_DE, LOW);
  digitalWrite(MAX485_RE, LOW);

  // Set ESP32 as Modbus Master
  node.begin(2, Serial2);  // Slave ID = 0x01 (default)
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  
  Serial.println("HMI Ready");
}

void sendToHMI(const char* label, int value) {
  String command = String(label) + ".txt=\"" + String(value) + "\"\xFF\xFF\xFF";
  HMI.print(command);
  Serial.print("Sent to HMI: ");
  Serial.println(command);
}

void loop() {
  uint8_t result;

  Serial.println("🔄 Requesting NPK values...");
  result = node.readHoldingRegisters(0x00, 3);  // Read 3 registers (N, P, K)

  if (result == node.ku8MBSuccess) {
    int nitrogen   = node.getResponseBuffer(0);
    int phosphorus = node.getResponseBuffer(1);
    int potassium  = node.getResponseBuffer(2);

    Serial.println("✅ Data Received Successfully!");
    Serial.print("🌱 Nitrogen: "); Serial.println(nitrogen);
    Serial.print("🌱 Phosphorus: "); Serial.println(phosphorus);
    Serial.print("🌱 Potassium: "); Serial.println(potassium);

    // Sending Data to Nextion HMI
    sendToHMI("t3", nitrogen);
    sendToHMI("t5", phosphorus);
    sendToHMI("t56", potassium);
  } else {
    Serial.print("❌ Failed to read Modbus data. Error Code: ");
    Serial.println(result);
  }

  delay(2000);  // Read every 2 seconds
}
