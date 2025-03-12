#include <ModbusMaster.h>

#define MAX485_DE  25  // RS485 Direction Control Pin (DE)
#define MAX485_RE  25   // RS485 Direction Control Pin (RE)
#define RS485_RX   27  // ESP32 RX (Receive Data from RS485)
#define RS485_TX   26  // ESP32 TX (Send Data to RS485)

// Create Modbus object for ESP32 (Master)
ModbusMaster node;

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

  // RS485 Control Pins
  pinMode(MAX485_DE, OUTPUT);
  pinMode(MAX485_RE, OUTPUT);
  digitalWrite(MAX485_DE, LOW);
  digitalWrite(MAX485_RE, LOW);

  // Set ESP32 as Modbus Master
  node.begin(2, Serial2);  // Slave ID = 0x01 (default)
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
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
    Serial.print("🌱 Nitrogen: "); Serial.print(nitrogen); Serial.println(" mg/kg");
    Serial.print("🌱 Phosphorus: "); Serial.print(phosphorus); Serial.println(" mg/kg");
    Serial.print("🌱 Potassium: "); Serial.print(potassium); Serial.println(" mg/kg");

  } else {
    Serial.print("❌ Failed to read Modbus data. Error Code: ");
    Serial.println(result);
  }

  delay(2000);  // Read every 2 seconds
}
