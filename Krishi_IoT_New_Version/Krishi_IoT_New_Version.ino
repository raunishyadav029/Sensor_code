// Krishi IoT Smart Agriculture Project Installed at CSIO Rose Garden
// WiFi enabled 
// SIM function: N/A
// Soil NPK,kestrel, Soil pH, Soil Moisture, Leaf Wetness, Light Sensor, Weather Sensor: Avl


#include <SoftwareSerial.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include <ModbusMaster.h>

#include <SPI.h>
#include <SD.h>

#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>


#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>


#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  // OLED I2C address (usually 0x3C or 0x3D)

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// BLE Kestrel D3 UUIDs
#define KESTREL_D3_SERVICE_UUID "12630000-cc25-497d-9854-9b6c02c77054"
#define TEMPERATURE_CHAR_UUID "12630001-cc25-497d-9854-9b6c02c77054"
#define HUMIDITY_CHAR_UUID "12630002-cc25-497d-9854-9b6c02c77054"
#define HEAT_INDEX_CHAR_UUID "12630003-cc25-497d-9854-9b6c02c77054"
#define STATION_PRESSURE_CHAR_UUID "12630007-cc25-497d-9854-9b6c02c77054"
#define DEW_POINT_CHAR_UUID "12630004-cc25-497d-9854-9b6c02c77054"



BLEScan* pBLEScan;
BLEAdvertisedDevice* myDevice;

const char* targetDeviceAddress = "60:a4:23:1c:f0:19";  // Confirm MAC

BLEClient* pClient;
BLERemoteService* pRemoteService;
BLERemoteCharacteristic* pTempCharacteristic;
BLERemoteCharacteristic* pHeatIndexCharacteristic;
BLERemoteCharacteristic* pHumidityCharacteristic;
BLERemoteCharacteristic* pPressureCharacteristic;
BLERemoteCharacteristic* pDewPointCharacteristic;

#define LDO_EN 4

// SD Pin Definitions
#define SD_MISO 19
#define SD_MOSI 23
#define SD_CLK 18
#define SD_CS 5  //  SD module CS

// RS485 Control Pins
#define RE_DE 25
#define TX_PIN 26
#define RX_PIN 27
#define BUF_SIZE 1024
// SoftwareSerial RS485(RX,TX);  // Software Serial for RS485
#define Response_Timeout 1000
#define Response_Size 9

#define Soil_pH_Sensor_Slave 1
#define Soil_NPK_Sensor_Slave 2
#define Light_Sensor_Slave 4
#define Soil_Moisture_Temp_EC_Sensor_Slave 6
#define Leaf_Temp_Humd_Sensor_Slave 7
#define Weather_Sensor_Slave 20

#define W_Temperature_Address 0x0000
#define W_Humidity_Address 0x0002
#define W_Pressure_Address 0x0004
#define W_Light_Intensity_Address 0x0006
#define W_Wind_Speed_Address 0x0012
#define W_Wind_Direction_Address 0x000C
#define W_Rainfall_Address 0x0014

RTC_DS3231 rtc;
// File path for logging data
const char* logFilePath = "/KRISHI_IOT_Data_Log.txt";

// File for logging
File data_File;
RTC_DATA_ATTR int Reading_ID = 0;

float Air_Temperature = 0.0;
float Air_Humidity = 0.0;
float Barometric_Pressure_Pa = 0.0;
float Barometric_Pressure_atm = 0.0;
float W_Light_Intensity = 0.0;
float Wind_Speed = 0.0;
float Wind_Direction = 0.0;
float Rainfall = 0.0;
float Soil_pH = 0.0;
float Soil_Moisture = 0.0;
float Soil_Temperature = 0.0;
float Soil_EC = 0.0;
float Soil_Nitrogen = 0.0;
float Soil_Phosphorus = 0.0;
float Soil_Potassium = 0.0;
float Leaf_Humidity = 0.0;
float Leaf_Temperature = 0.0;
float Light_Intensity = 0.0;
float Kestrel_Air_Temperature = 0.0;
float Kestrel_Air_Humidity = 0.0;
float Kestrel_Dew_Point_Temperature = 0.0;
float Kestrel_Heat_Stress_Index = 0.0;
float Kestrel_Barometric_Pressure = 0.0;


// Initialize Modbus
ModbusMaster pHSensor, npkSensor, soilSensor, lightSensor, leafSensor, weatherSensor;

// WiFi Credentials
const char* ssid = "Krishi-IoT_CSIO";
const char* password = "CSIOkrishiIoT";

// ThingsBoard Cloud server settings
const char* thinksBoardToken = "JMPfGn1LWaNZLMNXcTlZ";  // Replace with your ThingsBoard device access token
const char* thinksBoardServer = "demo.thingsboard.io";  // ThingsBoard server
const int thinksBoardPort = 1883;                       // MQTT port

// Declare a WiFiClient object
WiFiClient espClient;
PubSubClient client(espClient);


void preTransmission() {
  digitalWrite(RE_DE, HIGH);
}

void postTransmission() {
  digitalWrite(RE_DE, LOW);
}

void OLED_Display_Title() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 1);
  display.println("KRISHI-IoT Dashboard");
  display.setCursor(0, 8);
  display.println("---------------------");
  display.setTextSize(1);
}

void OLED_Display_Start() {
  display.clearDisplay();
  OLED_Display_Title();
  display.setCursor(0, 20);
  display.println("Soil and Meteorology");
  display.setCursor(0, 36);
  display.println("Data Fetching...");
  display.display();
  delay(3000);
}

void OLED_Display_Soil() {
  display.clearDisplay();
  OLED_Display_Title();
  display.setCursor(0, 16);
  display.println("Soil Moisture:" + String(Soil_Moisture/10) + "%");
  display.setCursor(0, 32);
  display.println("Soil Temp:" + String(Soil_Temperature/10) + "C");
  display.setCursor(0, 48);
  display.println("Soil EC:" + String(Soil_EC) + "dS/m");
  display.display();
}


void OLED_Display_NPK_pH() {
  display.clearDisplay();
  OLED_Display_Title();
  display.setCursor(0, 16);
  display.println("Soil pH:" + String(Soil_pH));
  display.setCursor(0, 28);
  display.println("Nitrogen:" + String(Soil_Nitrogen) + "mg/kg");
  display.setCursor(0, 42);
  display.println("Phosphorus:" + String(Soil_Phosphorus) + "mg/kg");
  display.display();
  display.setCursor(0, 54);
  display.println("Potassium:" + String(Soil_Potassium) + "mg/kg");
  display.display();
}

void OLED_Display_Leaf_Light() {
  display.clearDisplay();
  OLED_Display_Title();
  display.setCursor(0, 16);
  display.println("Leaf Humid:" + String(Leaf_Humidity/10) + " %");
  display.setCursor(0, 32);
  display.println("Leaf Temp:" + String(Leaf_Temperature) + " C");
  display.setCursor(0, 48);
  display.println("Light:" + String(Light_Intensity) + " lux");
  display.display();
}

void OLED_Display_Weather() {
  display.clearDisplay();
  OLED_Display_Title();
  display.setCursor(0, 12);
  display.println("Wind Speed:" + String(Wind_Speed) + "m/s");
  display.setCursor(0, 20);
  display.println("Wind Dir:" + String(Wind_Direction) + "deg");
  display.setCursor(0, 26);
  display.println("Rainfall:" + String(Rainfall) + "mm");
  display.setCursor(0, 36);
  display.println("Air temp:" + String(Air_Temperature) + "C");
  display.setCursor(0, 46);
  display.println("Air humd:" + String(Air_Humidity) + "%RH");
  display.setCursor(0, 56);
  display.println("Pressure:" + String(Barometric_Pressure_atm) + "atm");
  display.display();
}

void OLED_Display_Kestral() {
  display.clearDisplay();
  OLED_Display_Title();
  display.setCursor(0, 16);
  display.println("Kestrel Temp:" + String(Kestrel_Air_Temperature) + " C");
  display.setCursor(0, 26);
  display.println("Kestrel Humid:" + String(Kestrel_Air_Humidity) + " %");
  display.setCursor(0, 36);
  display.println("Dew Point:" + String(Kestrel_Dew_Point_Temperature) + " C");
  display.setCursor(0, 44);
  display.println("Heat stress:" + String(Kestrel_Heat_Stress_Index) + " C");
  display.setCursor(0, 52);
  display.println("Kestrel pres:" + String(Kestrel_Barometric_Pressure) + "atm");
  display.display();
}

void OLED_Display_Uploading_Data() {
  display.clearDisplay();
  OLED_Display_Title();
  display.setCursor(0, 20);
  display.println("Sensors Data Uploading");
  display.display();
  delay(3000);
}

void OLED_Display_Uploaded_Data() {
  display.clearDisplay();
  OLED_Display_Title();
  display.setCursor(0, 20);
  display.println("Sensors Data Uploaded");
  display.display();
  delay(3000);
}

void OLED_Display_Sleep() {
  display.clearDisplay();
  OLED_Display_Title();
  display.setCursor(0, 20);
  display.println("Device is going into ");
  display.setCursor(0, 36);
  display.println("sleep mode for 15 min...");
  display.display();
  delay(3000);
}

void readSoilSensor() {
  Serial.println("Reading Soil Sensor...");
  uint8_t result;
  int attempts = 3;
  while (attempts > 0) {
    result = soilSensor.readHoldingRegisters(0x00, 3);
    if (result == 0) break;
    Serial.println("Retrying Moisture Sensor...");
    delay(500);
    attempts--;
  }
  if (result == 0) {
    Soil_Moisture = soilSensor.getResponseBuffer(0);
    Soil_Temperature = soilSensor.getResponseBuffer(1);
    Soil_EC = soilSensor.getResponseBuffer(2);
  } else {
    Serial.println("Soil Sensor Read Error");
  }
  OLED_Display_Soil();
  delay(7000);
}


void readpHSensor() {
  Serial.println("Reading pH Sensor...");
  // Read 1 register from address 0x0000 (check sensor manual)
  uint8_t result = pHSensor.readHoldingRegisters(0x0000, 1);
  if (result == pHSensor.ku8MBSuccess) {  // Ensure successful read
    uint16_t raw_pH = pHSensor.getResponseBuffer(0);
    Soil_pH = raw_pH / 100.0;  // Convert if sensor gives pH * 100
    Serial.print("Soil pH: ");
    Serial.println(Soil_pH, 2);  // Print with 2 decimal places
  } else {
    Serial.print("pH Sensor Read Error! Modbus Error Code: ");
    Serial.println(result, HEX);  // Debugging: print error code
  }
}


void readNPKSensor() {
  uint8_t result = npkSensor.readHoldingRegisters(0x00, 3);
  if (result == 0) {
    Soil_Nitrogen = npkSensor.getResponseBuffer(0);
    Soil_Phosphorus = npkSensor.getResponseBuffer(1);
    Soil_Potassium = npkSensor.getResponseBuffer(2);
  } else {
    Serial.println("NPK Read Error");
  }
  OLED_Display_NPK_pH();
  delay(7000);
}


void readLightSensor() {
  Serial.println("Reading Light Sensor...");
  uint8_t result = lightSensor.readHoldingRegisters(0x00, 1);
  if (result == 0) {
    Light_Intensity = lightSensor.getResponseBuffer(0);
  } else {
    Serial.println("Light Sensor Read Error");
  }
}

void readLeafSensor() {
  Serial.println("Reading Leaf Sensor...");
  uint8_t result;
  int attempts = 3;
  while (attempts > 0) {
    result = leafSensor.readHoldingRegisters(0x00, 2);
    if (result == 0) break;
    Serial.println("Retrying Leaf Sensor...");
    delay(500);
    attempts--;
  }
  if (result == 0) {
    Leaf_Temperature = leafSensor.getResponseBuffer(0);
    Leaf_Humidity = leafSensor.getResponseBuffer(1);
  } else {
    Serial.println("Leaf Sensor Read Error");
  }
  OLED_Display_Leaf_Light();
  delay(7000);
}


// Request data from S700 sensor
float Read_Sensor_Data(uint16_t registerAddress) {
  uint8_t result = weatherSensor.readHoldingRegisters(registerAddress, 2);  // Read 2 registers (32-bit value)
  if (result == weatherSensor.ku8MBSuccess) {
    uint32_t rawData = (weatherSensor.getResponseBuffer(0) << 16) | weatherSensor.getResponseBuffer(1);
    return rawData / 1000.0;  // Convert to float
  } else {
    Serial.print("Error reading register 0x");
    Serial.println(registerAddress, HEX);
    return -999.0;  // Error value
  }
}

void readWeatherSensor() {
  Serial.println("Reading Weather Data...");
  Air_Temperature = Read_Sensor_Data(W_Temperature_Address);
  Air_Humidity = Read_Sensor_Data(W_Humidity_Address);
  Barometric_Pressure_Pa = Read_Sensor_Data(W_Pressure_Address);
  W_Light_Intensity = Read_Sensor_Data(W_Light_Intensity_Address);
  Wind_Speed = Read_Sensor_Data(W_Wind_Speed_Address);
  Wind_Direction = Read_Sensor_Data(W_Wind_Direction_Address);
  Rainfall = Read_Sensor_Data(W_Rainfall_Address);
  OLED_Display_Weather();
  delay(7000);
}



float parseKestrelData(String data) {
  if (data.length() < 3) return 0.0;  // Minimum 3 bytes (1 header + 2 value)

  uint8_t header = static_cast<uint8_t>(data[0]);
  if (header != 0x07) return 0.0;  // Validate header byte

  // Extract 16-bit little-endian value from bytes 1-2
  uint16_t rawValue = static_cast<uint8_t>(data[1]) | (static_cast<uint8_t>(data[2]) << 8);
  return static_cast<float>(rawValue) / 100.0;  // Divide by 100 for scaling
}

void printCharacteristic(BLERemoteCharacteristic* characteristic, const char* name) {
  if (characteristic != nullptr && characteristic->canRead()) {
    String data = characteristic->readValue();
    // Serial.print("Raw " + String(name) + " Bytes (Hex): ");
    for (int i = 0; i < data.length(); i++) {
      // Serial.printf("%02X ", static_cast<uint8_t>(data[i]));
    }
    Serial.println();

    float value = parseKestrelData(data);
    if (value != 0.0) {
      // Serial.println(String(name) + ": " + String(value, 2));

      if (characteristic == pTempCharacteristic) {
        Kestrel_Air_Temperature = value;
      }

      else if (name == "Humidity (%)") {
        Kestrel_Air_Humidity = value;
      }

      else if (characteristic == pPressureCharacteristic) {
        Kestrel_Barometric_Pressure = value;
      }

      else if (characteristic == pHeatIndexCharacteristic) {
        Kestrel_Heat_Stress_Index = value;
      }

      else if (characteristic == pDewPointCharacteristic) {
        Kestrel_Dew_Point_Temperature = value;
      }

    }

    else {
      Serial.println("Invalid " + String(name) + " data!");
    }
  }
}



void readBLEData() {
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
        Serial.println("Kestrel D3 Service not found!");
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

      // Kestrel_Air_Temperature = pRemoteService->getCharacteristic(TEMPERATURE_CHAR_UUID);
      // Kestrel_Air_Humidity = pHumidityCharacteristic;
      // Kestrel_Dew_Point_Temperature = pDewPointCharacteristi;
      // Kestrel_Heat_Stress_Index = pHeatIndexCharacteristic;
      // Kestrel_Barometric_Pressure = pPressureCharacteristic;

      pClient->disconnect();
      Serial.println("BLE Disconnected");
    } else {
      Serial.println("BLE Connection failed!");
    }
  } else {
    Serial.println("Kestrel D3 Device not found");
  }
  OLED_Display_Kestral();
  delay(7000);
}

void showSensorsData() {

  Serial.println();

  Serial.println();
  Serial.print("Soil pH: ");
  Serial.print("Soil_pH");
  Serial.println(" ");

  Serial.println();
  Serial.print("Air_Temperature: ");
  Serial.print(Air_Temperature);
  Serial.println(" °C");

  Serial.print("Air_Humidity: ");
  Serial.print(Air_Humidity);
  Serial.println(" %RH");

  Serial.print("Barometric_Pressure_Pa: ");
  Serial.print(Barometric_Pressure_Pa);
  Serial.println(" Pa");

  Barometric_Pressure_atm = Barometric_Pressure_Pa / 101325.0;
  Serial.print("Barometric_Pressure_atm: ");
  Serial.print(Barometric_Pressure_atm);
  Serial.println(" atm");

  Serial.print("Wind Speed: ");
  Serial.print(Wind_Speed);
  Serial.println(" m/s");

  Serial.print("Wind Direction: ");
  Serial.print(Wind_Direction);
  Serial.println(" °");

  Serial.print("W_Light Intensity: ");
  Serial.print(W_Light_Intensity);
  Serial.println(" Lux");

  Serial.print("Rainfall: ");
  Serial.print(Rainfall);
  Serial.println(" mm");


  Serial.println();
  Soil_Temperature = Soil_Temperature / 10;
  Serial.print("Soil Temperature: ");
  Serial.print(Soil_Temperature);
  Serial.println(" °C");

  Soil_Moisture = Soil_Moisture / 10;
  Serial.print("Soil Moisture: ");
  Serial.print(Soil_Moisture);
  Serial.println(" %RH");

  Serial.print("Soil EC: ");
  Serial.print(Soil_EC);
  Serial.println(" mV");

  Serial.println();
  Serial.print("Soil Nitogen: ");
  Serial.print(Soil_Nitrogen);
  Serial.println(" mg/Kg");

  Serial.print("Soil Phosphorus: ");
  Serial.print(Soil_Phosphorus);
  Serial.println(" mg/Kg");

  Serial.print("Soil Potassium: ");
  Serial.print(Soil_Potassium);
  Serial.println(" mg/Kg");

  Serial.println();
  Leaf_Humidity = Leaf_Humidity / 10;
  Serial.print("Leaf Humidity: ");
  Serial.print(Leaf_Humidity);
  Serial.println(" %");

  Leaf_Temperature = Leaf_Temperature / 10;
  Serial.print("Leaf Temperature: ");
  Serial.print(Leaf_Temperature);
  Serial.println(" °C");

  Serial.println();
  Serial.print("Kestrel Air Temperature: ");
  Serial.print(Kestrel_Air_Temperature);
  Serial.println(" °C");

  Serial.print("Kestrel Air Humidity: ");
  Serial.print(Kestrel_Air_Humidity);
  Serial.println(" %");

  Serial.print("Kestrel Dew Point Temperature: ");
  Serial.print(Kestrel_Dew_Point_Temperature);
  Serial.println(" °C");

  Serial.print("Kestrel Heat Stress Index: ");
  Serial.print(Kestrel_Heat_Stress_Index);
  Serial.println(" °C");

  Kestrel_Barometric_Pressure = Kestrel_Barometric_Pressure / 98.7;
  Serial.print("Kestrel Barometric Pressure: ");
  Serial.print(Kestrel_Barometric_Pressure);
  Serial.println(" atm");

  delay(5000);
  Serial.println();
}


// Data logging function
void Data_Logging() {
  String rtcTime = String(rtc.now().year()) + "-" + String(rtc.now().month()) + "-" + String(rtc.now().day()) + " " + String(rtc.now().hour()) + ":" + String(rtc.now().minute()) + ":" + String(rtc.now().second());

  String Data_String = "ID: " + String(Reading_ID) + ", RTC Time: " + rtcTime + ", CSIO_Air Temperature: " + String(Air_Temperature, 3) + "°C" + ", CSIO_Air Humidity: " + String(Air_Humidity, 3) + " %" + ", CSIO_Barometric Pressure (Pa): " + String(Barometric_Pressure_Pa, 3) + " Pa" + ", CSIO_Barometric Pressure (atm): " + String(Barometric_Pressure_atm, 3) + " atm" + ", CSIO_Light Intensity: " + String(W_Light_Intensity, 3) + " lux" + ", CSIO_Wind Speed: " + String(Wind_Speed, 3) + " m/s" + ", CSIO_Wind Direction: " + String(Wind_Direction, 3) + "°" + ", CSIO_Rainfall: " + String(Rainfall, 3) + " mm" + ", CSIO_Soil pH: " + String(Soil_pH, 3) + ", CSIO_Soil Moisture: " + String(Soil_Moisture, 3) + " %" + ", CSIO_Soil Temperature: " + String(Soil_Temperature, 3) + "°C" + ", CSIO_Soil EC: " + String(Soil_EC, 3) + " dS/m" + ", CSIO_Soil Nitrogen: " + String(Soil_Nitrogen, 3) + " mg/kg" + ", CSIO_Soil Phosphorus: " + String(Soil_Phosphorus, 3) + " mg/kg" + ", CSIO_Soil Potassium: " + String(Soil_Potassium, 3) + " mg/kg" + ", Leaf Humidity: " + String(Leaf_Humidity, 3) + " %" + ", CSIO_Leaf Temperature: " + String(Leaf_Temperature, 3) + "°C" + ", CSIO_Light Intensity: " + String(Light_Intensity, 3) + " lux" + ", Kestrel Air Temperature: " + String(Kestrel_Air_Temperature, 3) + "°C" + ", Kestrel Air Humidity: " + String(Kestrel_Air_Humidity, 3) + " %" + ", Kestrel Dew Point Temperature: " + String(Kestrel_Dew_Point_Temperature, 3) + "°C" + ", Kestrel Heat Stress Index: " + String(Kestrel_Heat_Stress_Index, 3) + "°C" + ", Kestrel Barometric Pressure: " + String(Kestrel_Barometric_Pressure, 3) + " atm";

  Serial.println(Data_String);

  data_File = SD.open(logFilePath, FILE_APPEND);
  if (data_File) {
    data_File.println(Data_String);
    data_File.close();
    Serial.println("Data logged successfully.");
  } else {
    Serial.println("Failed to open file for writing");
  }
};


// void setup_wifi() {
//   delay(10);
//   Serial.print("Connecting to ");
//   Serial.println(ssid);
//   WiFi.begin(ssid, password);
//   Serial.print("Connecting to WiFi...");
//   unsigned long startAttemptTime = millis();
//   while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
//     Serial.print(".");
//     delay(500);
//   }
//   Serial.println(WiFi.status() == WL_CONNECTED ? "Connected!" : "Failed!");
// };


void Upload_Data() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected. Cannot upload data.");
    return;
  }

  // Set the server for the PubSubClient
  client.setServer(thinksBoardServer, thinksBoardPort);

  if (!client.connected()) {
    if (client.connect("ESP32Client", thinksBoardToken, NULL)) {
      Serial.println("Connected to ThingsBoard");
    } else {
      Serial.print("Failed to connect, rc=");
      Serial.println(client.state());
      return;
    }
  }


  //  Send data in multiple parts
  sendTelemetry_Part1();
  delay(1000);
  sendTelemetry_Part2();
  delay(1000);
  sendTelemetry_Part3();
  delay(1000);
  sendTelemetry_Part4();
}

void sendTelemetry_Part1() {
  DynamicJsonDocument doc(BUF_SIZE);

  doc["CSIO_air_temperature"] = Air_Temperature;
  doc["CSIO_air_humidity"] = Air_Humidity;
  doc["CSIO_air_pressure"] = Barometric_Pressure_atm;
  doc["CSIO_wind_light_intensity"] = Light_Intensity;
  doc["CSIO_wind_speed"] = Wind_Speed;
  doc["CSIO_wind_direction"] = Wind_Direction;
  doc["CSIO_rainfall"] = Rainfall;

  sendJson(doc);
}

void sendTelemetry_Part2() {
  DynamicJsonDocument doc(BUF_SIZE);

  doc["CSIO_soil_pH"] = Soil_pH;
  doc["CSIO_soil_moisture"] = Soil_Moisture;
  doc["CSIO_soil_temperature"] = Soil_Temperature;
  doc["CSIO_soil_EC"] = Soil_EC;
  doc["CSIO_soil_nitrogen"] = Soil_Nitrogen;
  doc["CSIO_soil_phosphorus"] = Soil_Phosphorus;
  doc["CSIO_soil_potassium"] = Soil_Potassium;

  sendJson(doc);
}

void sendTelemetry_Part3() {
  DynamicJsonDocument doc(BUF_SIZE);

  doc["CSIO_leaf_humidity"] = Leaf_Humidity;
  doc["CSIO_leaf_temperature"] = Leaf_Temperature;
  doc["CSIO_light_intensity"] = Light_Intensity;

  sendJson(doc);
}

void sendTelemetry_Part4() {
  DynamicJsonDocument doc(BUF_SIZE);

  doc["CSIO_kestrel_air_temperature"] = Kestrel_Air_Temperature;
  doc["CSIO_kestrel_air_humidity"] = Kestrel_Air_Humidity;
  doc["CSIO_kestrel_dew_point_temperature"] = Kestrel_Dew_Point_Temperature;
  doc["CSIO_kestrel_heat_stress_index"] = Kestrel_Heat_Stress_Index;
  doc["CSIO_kestrel_barometric_pressure"] = Kestrel_Barometric_Pressure;

  sendJson(doc);
}

void sendJson(DynamicJsonDocument &doc) {
  char payload[BUF_SIZE];
  serializeJson(doc, payload);

  Serial.print("Sending Payload: ");
  Serial.println(payload);

  if (client.publish("v1/devices/me/telemetry", payload)) {
    Serial.println(" Data sent successfully!");
  } else {
    Serial.println(" Failed to send data! Retrying...");
    delay(2000);
    client.publish("v1/devices/me/telemetry", payload);  // Retry once
  }
}




// void Upload_Data() {
//   if (WiFi.status() == WL_CONNECTED) {
//     // Set the server for the PubSubClient
//     client.setServer(thinksBoardServer, thinksBoardPort);
//     if (!client.connected()) {
//       // Attempt to connect
//       if (client.connect("ESP32Client", thinksBoardToken, NULL)) {
//         Serial.println("Connected to ThingsBoard");
//       } else {
//         Serial.print("Failed to connect, rc=");
//         Serial.print(client.state());
//         return;
//       }
//     }

   

void setup() {
  Serial.begin(115200);
  Serial.println("Serial Monitor Enabled");
  delay(1000);

  pinMode(LDO_EN, OUTPUT);
  digitalWrite(LDO_EN, HIGH);

  Wire.begin();
  Serial.println("I2C Communication Enabled");

  // Initialize SD Card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card initialization failed!");
    return;
  }
  Serial.println("SD Card initialized.");

  if (!display.begin(SCREEN_ADDRESS, OLED_RESET)) {
    Serial.println("SH1106 allocation failed");
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  delay(500);

  OLED_Display_Start();

  // RS485 Setup
  pinMode(RE_DE, OUTPUT);
  digitalWrite(RE_DE, LOW);
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);


  // Modbus setup
  npkSensor.begin(Soil_NPK_Sensor_Slave, Serial2);
  soilSensor.begin(Soil_Moisture_Temp_EC_Sensor_Slave, Serial2);
  lightSensor.begin(Light_Sensor_Slave, Serial2);
  leafSensor.begin(Leaf_Temp_Humd_Sensor_Slave, Serial2);
  weatherSensor.begin(Weather_Sensor_Slave, Serial2);
  pHSensor.begin(Soil_pH_Sensor_Slave, Serial2);


  npkSensor.preTransmission(preTransmission);
  npkSensor.postTransmission(postTransmission);
  soilSensor.preTransmission(preTransmission);
  soilSensor.postTransmission(postTransmission);
  lightSensor.preTransmission(preTransmission);
  lightSensor.postTransmission(postTransmission);
  leafSensor.preTransmission(preTransmission);
  leafSensor.postTransmission(postTransmission);
  weatherSensor.preTransmission(preTransmission);
  weatherSensor.postTransmission(postTransmission);
  pHSensor.preTransmission(preTransmission);
  pHSensor.postTransmission(postTransmission);




  BLEDevice::init("Kestrel_D3_Client");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);



  // Initialize RTC
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1)
      ;
  }
  Serial.println("RTC Enabled");
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, resetting time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  // Set RTC to compile time
  }
  Reading_ID++;
  // Serial.printf("Reading ID: %d\n", Reading_ID);

  // esp_sleep_enable_timer_wakeup(15*60* 1000000);   // 15 mins
  esp_sleep_enable_timer_wakeup(2 * 60 * 1000000);  // 60 secs


  readSoilSensor();
  delay(1000);
  readpHSensor();
  delay(1000);
  readNPKSensor();
  delay(1000);
  readLightSensor();
  delay(1000);
  readLeafSensor();
  delay(1000);
  readWeatherSensor();
  delay(1000);
  readBLEData();
  delay(5000);
  showSensorsData();
  delay(1000);

  
  BLEDevice::deinit(true);  // true = erase BLE client cache
  delay(1000);              // allow memory cleanup
  


  Data_Logging();
  delay(1000);
  Serial.println("All Parameters data logged..");

  
  WiFi.begin(ssid, password);
  int retry = 20;
  while (WiFi.status() != WL_CONNECTED && retry-- > 0) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected.");
  } else {
    Serial.println("WiFi connection failed.");
    }
  
  client.setServer(thinksBoardServer, thinksBoardPort);

   // Check if WiFi is connected before uploading
  if (WiFi.status() == WL_CONNECTED) {
    Upload_Data();
  } else {
    Serial.println("WiFi not connected. Data will be uploaded later.");
  }
  

  OLED_Display_Uploading_Data();
  delay(1000);

  OLED_Display_Uploaded_Data();
  delay(500);

  OLED_Display_Sleep();

  Serial.println("Device went into deep sleep. will be wake up in 15 mins");
  esp_deep_sleep_start();
}
void loop() {
  // Empty - everything handled in setup
}
