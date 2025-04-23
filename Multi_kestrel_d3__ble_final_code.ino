#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEScan.h>

// Kestrel D3 UUIDs
#define KESTREL_D3_SERVICE_UUID       "12630000-cc25-497d-9854-9b6c02c77054"
#define TEMPERATURE_CHAR_UUID        "12630001-cc25-497d-9854-9b6c02c77054"
#define HEAT_INDEX_CHAR_UUID         "12630003-cc25-497d-9854-9b6c02c77054"
#define HUMIDITY_CHAR_UUID           "12630002-cc25-497d-9854-9b6c02c77054"
#define STATION_PRESSURE_CHAR_UUID   "12630007-cc25-497d-9854-9b6c02c77054"
#define DEW_POINT_CHAR_UUID          "12630004-cc25-497d-9854-9b6c02c77054"

const int NUM_DEVICES = 9;
const char* targetMACs[NUM_DEVICES] = {
  "60:a4:23:1c:f0:19", "90:35:ea:da:06:39", "dc:8e:95:dc:1c:c1",
  "b4:e3:f9:97:41:24", "88:6b:0f:c0:ab:0d", "dc:8e:95:dc:1b:75",
  "00:3c:84:6d:74:fc", "ec:1b:bd:bf:e9:1a", "e0:79:8d:86:47:86"
};

BLEClient* client;

float parseKestrelData(String rawData) {
  if (rawData.length() < 3) return 0.0; // Minimum 3 bytes (1 header + 2 value)
  
  uint8_t header = static_cast<uint8_t>(rawData[0]);
  if (header != 0x07) return 0.0; // Validate header byte
  
  // Extract 16-bit little-endian value from bytes 1-2
  uint16_t rawValue = static_cast<uint8_t>(rawData[1]) | (static_cast<uint8_t>(rawData[2]) << 8);
  return static_cast<float>(rawValue) / 100.0; // Divide by 100 for scaling
}

bool scanForDevice(const char* targetMAC, BLEAddress &foundAddr) {
  BLEScan* scanner = BLEDevice::getScan();
  scanner->setActiveScan(true);
  BLEScanResults results = *scanner->start(5, false);

  for (int i = 0; i < results.getCount(); i++) {
    BLEAdvertisedDevice device = results.getDevice(i);
    if (device.getAddress().toString() == targetMAC) {
      foundAddr = device.getAddress();
      return true;
    }
  }
  return false;
}

void readKestrelSensor(BLEAddress bleAddress, int index) {
  Serial.printf("\nReading from Device %d - [%s]\n", index + 1, bleAddress.toString().c_str());

  if (!client->connect(bleAddress)) {
    Serial.println(" Failed to connect");
    return;
  }

  BLERemoteService* service = client->getService(KESTREL_D3_SERVICE_UUID);
  if (!service) {
    Serial.println(" Service not found!");
    client->disconnect();
    
    return;
  }

  auto readChar = [&](const char* uuid, const char* label) {
    BLERemoteCharacteristic* c = service->getCharacteristic(uuid);
    if (c && c->canRead()) {
      String valueStr = c->readValue();
      // Serial.printf("Raw data for %s: [%s]\n", label, valueStr.c_str());  // Debugging output
      float value = parseKestrelData(valueStr);
      Serial.printf("%s: %.2f\n", label, value);
    } else {
      Serial.printf("%s: Not found or cannot read\n", label);
    }
  };

  readChar(TEMPERATURE_CHAR_UUID,      "Temperature(°C)");
  readChar(HUMIDITY_CHAR_UUID,         "Humidity(%)");
  readChar(HEAT_INDEX_CHAR_UUID ,       "Heat Index(°C)");
  readChar(STATION_PRESSURE_CHAR_UUID, "Station Pressure(kPa)");
  readChar(DEW_POINT_CHAR_UUID,        "Dew Point(°C)");

  client->disconnect();
  Serial.println("Disconnected from device.");
  delay(500);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE Kestrel Multi-Device Reader");
  BLEDevice::init("");
  client = BLEDevice::createClient();
}

void loop() {
  for (int i = 0; i < NUM_DEVICES; i++) {
    BLEAddress foundAddr("");
    Serial.printf(" \nScanning for device [%s]...", targetMACs[i]);

    if (scanForDevice(targetMACs[i], foundAddr)) {
      readKestrelSensor(foundAddr, i);
    } else {
      Serial.println("Device not found in scan.");
    }

    delay(1000);
  }

  Serial.println("\nFinished reading all devices. Waiting 10 seconds...\n");
  delay(10000);
} 