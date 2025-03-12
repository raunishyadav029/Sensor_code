#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define KESTREL_D3_SERVICE_UUID       "12630000-cc25-497d-9854-9b6c02c77054"
#define TEMPERATURE_CHAR_UUID        "12630001-cc25-497d-9854-9b6c02c77054"
#define HEAT_INDEX_CHAR_UUID         "12630003-cc25-497d-9854-9b6c02c77054"
#define HUMIDITY_CHAR_UUID           "12630002-cc25-497d-9854-9b6c02c77054"
#define STATION_PRESSURE_CHAR_UUID   "12630007-cc25-497d-9854-9b6c02c77054"
#define DEW_POINT_CHAR_UUID          "12630004-cc25-497d-9854-9b6c02c77054"

BLEScan* pBLEScan;
BLEAdvertisedDevice* myDevice;

const char* targetDeviceAddress = "60:a4:23:1c:f0:19"; // Confirm MAC

BLEClient* pClient;
BLERemoteService* pRemoteService;
BLERemoteCharacteristic* pTempCharacteristic;
BLERemoteCharacteristic* pHeatIndexCharacteristic;
BLERemoteCharacteristic* pHumidityCharacteristic;
BLERemoteCharacteristic* pPressureCharacteristic;
BLERemoteCharacteristic* pDewPointCharacteristic;

float parseKestrelData(String data) {
  if (data.length() < 3) return 0.0; // Minimum 3 bytes (1 header + 2 value)
  
  uint8_t header = static_cast<uint8_t>(data[0]);
  if (header != 0x07) return 0.0; // Validate header byte
  
  // Extract 16-bit little-endian value from bytes 1-2
  uint16_t rawValue = static_cast<uint8_t>(data[1]) | (static_cast<uint8_t>(data[2]) << 8);
  return static_cast<float>(rawValue) / 100.0; // Divide by 100 for scaling
}

void printCharacteristic(BLERemoteCharacteristic* characteristic, const char* name) {
  if (characteristic != nullptr && characteristic->canRead()) {
    String data = characteristic->readValue();
    Serial.print("Raw " + String(name) + " Bytes (Hex): ");
    for (int i = 0; i < data.length(); i++) {
      Serial.printf("%02X ", static_cast<uint8_t>(data[i]));
    }
    Serial.println();
    
    float value = parseKestrelData(data);
    if (value != 0.0) {
      Serial.println(String(name) + ": " + String(value, 2));
    } else {
      Serial.println("Invalid " + String(name) + " data!");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  BLEDevice::init("Kestrel_D3_Client");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  Serial.println("Starting BLE scan...");
}

void loop() {
  BLEScanResults foundDevices = *pBLEScan->start(10, false);
  bool deviceFound = false;

  for (int i = 0; i < foundDevices.getCount(); i++) {
    BLEAdvertisedDevice device = foundDevices.getDevice(i);
    if (device.getAddress().toString().equals(targetDeviceAddress)) {
      deviceFound = true;
      myDevice = new BLEAdvertisedDevice(device);
      break;
    }
  }

  if (deviceFound) {
    Serial.println("Found Kestrel D3! Connecting...");
    pClient = BLEDevice::createClient();
    
    if (pClient->connect(myDevice)) {
      Serial.println("Connected! Accessing services...");
      pRemoteService = pClient->getService(KESTREL_D3_SERVICE_UUID);
      
      if (pRemoteService == nullptr) {
        Serial.println("Service not found!");
        pClient->disconnect();
        return;
      }

      pTempCharacteristic = pRemoteService->getCharacteristic(TEMPERATURE_CHAR_UUID);
      pHeatIndexCharacteristic = pRemoteService->getCharacteristic(HEAT_INDEX_CHAR_UUID);
      pHumidityCharacteristic = pRemoteService->getCharacteristic(HUMIDITY_CHAR_UUID);
      pPressureCharacteristic = pRemoteService->getCharacteristic(STATION_PRESSURE_CHAR_UUID);
      pDewPointCharacteristic = pRemoteService->getCharacteristic(DEW_POINT_CHAR_UUID);

      printCharacteristic(pTempCharacteristic, "Temperature (°C)");
      printCharacteristic(pHeatIndexCharacteristic, "Heat Index (°C)");
      printCharacteristic(pHumidityCharacteristic, "Humidity (%)");
      printCharacteristic(pPressureCharacteristic, "Pressure (kPa)");
      printCharacteristic(pDewPointCharacteristic, "Dew Point (°C)");

      pClient->disconnect();
      Serial.println("Disconnected");
    } else {
      Serial.println("Connection failed!");
    }
  } else {
    Serial.println("Device not found");
  }

  delay(5000);
}