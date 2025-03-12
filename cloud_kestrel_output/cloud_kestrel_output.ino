#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <WiFi.h>
#include <HTTPClient.h>

#define WIFI_SSID "Raunish_PC"
#define WIFI_PASSWORD "Csio@12345"
#define THINGSPEAK_API_KEY "JOM9EF2X01I06SDS"
#define THINGSPEAK_URL "http://api.thingspeak.com/update"

#define KESTREL_D3_SERVICE_UUID       "12630000-cc25-497d-9854-9b6c02c77054"
#define TEMPERATURE_CHAR_UUID        "12630001-cc25-497d-9854-9b6c02c77054"
#define HEAT_INDEX_CHAR_UUID         "12630003-cc25-497d-9854-9b6c02c77054"
#define HUMIDITY_CHAR_UUID           "12630002-cc25-497d-9854-9b6c02c77054"
#define STATION_PRESSURE_CHAR_UUID   "12630007-cc25-497d-9854-9b6c02c77054"
#define DEW_POINT_CHAR_UUID          "12630004-cc25-497d-9854-9b6c02c77054"

BLEScan* pBLEScan;
BLEAdvertisedDevice* myDevice;
const char* targetDeviceAddress = "60:a4:23:1c:f0:19";
BLEClient* pClient;
BLERemoteService* pRemoteService;
BLERemoteCharacteristic* pTempCharacteristic;
BLERemoteCharacteristic* pHeatIndexCharacteristic;
BLERemoteCharacteristic* pHumidityCharacteristic;
BLERemoteCharacteristic* pPressureCharacteristic;
BLERemoteCharacteristic* pDewPointCharacteristic;

float parseKestrelData(String data) {
  if (data.length() < 3) return 0.0;
  uint8_t header = static_cast<uint8_t>(data[0]);
  if (header != 0x07) return 0.0;
  uint16_t rawValue = static_cast<uint8_t>(data[1]) | (static_cast<uint8_t>(data[2]) << 8);
  return static_cast<float>(rawValue) / 100.0;
}

void sendToThingSpeak(float temp, float heatIndex, float humidity, float pressure, float dewPoint) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String(THINGSPEAK_URL) + "?api_key=" + THINGSPEAK_API_KEY +
                 "&field1=" + String(temp) + "&field2=" + String(heatIndex) +
                 "&field3=" + String(humidity) + "&field4=" + String(pressure) +
                 "&field5=" + String(dewPoint);
    http.begin(url);
    int httpResponseCode = http.GET();
    http.end();
    Serial.println("ThingSpeak response: " + String(httpResponseCode));
  } else {
    Serial.println("WiFi Disconnected!");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
  BLEDevice::init("Kestrel_D3_Client");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  
  Serial.println("Scanning for device...");
  BLEScanResults* foundDevices = pBLEScan->start(10, false);
  for (int i = 0; i < foundDevices->getCount(); i++) {
    BLEAdvertisedDevice device = foundDevices->getDevice(i);
    if (device.getAddress().toString() == targetDeviceAddress) {
      myDevice = new BLEAdvertisedDevice(device);
      break;
    }
  }
  
  if (myDevice) {
    Serial.println("Connecting to BLE device...");
    pClient = BLEDevice::createClient();
    if (pClient->connect(myDevice)) {
      Serial.println("Connected to BLE!");
      pRemoteService = pClient->getService(KESTREL_D3_SERVICE_UUID);
      if (pRemoteService) {
        pTempCharacteristic = pRemoteService->getCharacteristic(TEMPERATURE_CHAR_UUID);
        pHeatIndexCharacteristic = pRemoteService->getCharacteristic(HEAT_INDEX_CHAR_UUID);
        pHumidityCharacteristic = pRemoteService->getCharacteristic(HUMIDITY_CHAR_UUID);
        pPressureCharacteristic = pRemoteService->getCharacteristic(STATION_PRESSURE_CHAR_UUID);
        pDewPointCharacteristic = pRemoteService->getCharacteristic(DEW_POINT_CHAR_UUID);
      }
    }
  }
}

void loop() {
  if (pClient && pClient->isConnected()) {
    float temp = parseKestrelData(String(pTempCharacteristic->readValue().c_str()));
    float heatIndex = parseKestrelData(String(pHeatIndexCharacteristic->readValue().c_str()));
    float humidity = parseKestrelData(String(pHumidityCharacteristic->readValue().c_str()));
    float pressure = parseKestrelData(String(pPressureCharacteristic->readValue().c_str()));
    float dewPoint = parseKestrelData(String(pDewPointCharacteristic->readValue().c_str()));

    Serial.println("Temperature: " + String(temp, 2) + " °C");
    Serial.println("Heat Index: " + String(heatIndex, 2) + " °C");
    Serial.println("Humidity: " + String(humidity, 2) + " %");
    Serial.println("Pressure: " + String(pressure, 2) + " kPa");
    Serial.println("Dew Point: " + String(dewPoint, 2) + " °C");

    sendToThingSpeak(temp, heatIndex, humidity, pressure, dewPoint);
  } else {
    Serial.println("BLE Device disconnected! Reconnecting...");
    pClient->connect(myDevice);
  }
  delay(60000);
}
