#include <MQUnifiedsensor.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "version.h"

#define PACKET_SEND_FREQUENCY     120000
#define GAS_DELAY        5000
#define GAS_DATA_POINTS        5

/************************Hardware Related Macros************************************/
#define         Board                   ("ESP-32")
#define         Pin                     (A0)  //Analog input 4 of your arduino
/***********************Software Related Macros************************************/
#define         Type                    ("MQ-4") //MQ4
#define         Voltage_Resolution      (5)
#define         ADC_Bit_Resolution      (12) // For arduino UNO/MEGA/NANO
#define         RatioMQ4CleanAir        (4.4) //RS / R0 = 60 ppm 
/*****************************Globals***********************************************/

//Tautuk Specific calls
const uint32_t FLASH_ADDRESS_4MB = 0x3F0000;
const uint32_t FLASH_ADDRESS_16MB = 0xFE0000;

const char* serverUrl = "http://device-api.tautuk.com/events";

//Declare Sensor
MQUnifiedsensor MQ4(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

// WiFi Credentials
char ssid[32] = {0};
char password[64] = {0};
char secretKey[64] = {0};

String lastSendResponse = "";

// Packet states
enum DataPacketState {
  DATA_PACKET_A,
  DATA_PACKET_B
};
DataPacketState dataState = DATA_PACKET_A;

enum DataReadyState {
  DATA_PACKET_READY,
  DATA_PACKET_NOT_READY
};
DataReadyState dataAReady = DATA_PACKET_NOT_READY;
DataReadyState dataBReady = DATA_PACKET_NOT_READY;

String gasDataA = "";
String gasDataB = "";
int readingCount = 0;


// SemaphoreHandle_t mutex;
// Task handles
TaskHandle_t Task1;
TaskHandle_t Task2;
void connectToWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Establishing connection to WiFi with SSID " + String(ssid));
  }

  Serial.println("Connected to WiFi!");
}

void readGasSensor(){
  MQ4.update();

  // xSemaphoreTake(mutex, portMAX_DELAY);
  if (readingCount == GAS_DATA_POINTS) {
    switch (dataState) {
      case DATA_PACKET_A:
        dataAReady = DATA_PACKET_READY;
        dataBReady = DATA_PACKET_NOT_READY;
        dataState = DATA_PACKET_B;
        break;
      case DATA_PACKET_B:
        dataAReady = DATA_PACKET_NOT_READY;
        dataBReady = DATA_PACKET_READY;
        dataState = DATA_PACKET_A;
        break;
    }
    readingCount = 0;
  }

   MQ4.setA(3811.9); MQ4.setB(-3.113); // Configure the equation to to calculate CH4 concentration
  float LPG = MQ4.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  
  MQ4.setA(1012.7); MQ4.setB(-2.786); // Configure the equation to to calculate CH4 concentration
  float CH4 = MQ4.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  
  MQ4.setA(200000000000000); MQ4.setB(-19.05); // Configure the equation to to calculate CH4 concentration
  float CO = MQ4.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  
    switch (dataState) {
    case DATA_PACKET_A:
      gasDataA += String(LPG,4) + "," + String(CH4,4) + "," + String(CO,4) + ";";
      break;
    case DATA_PACKET_B:
      gasDataB += String(LPG,4) + "," + String(CH4,4) + "," + String(CO,4) + ";";
      break;
  }
  readingCount++;
  // xSemaphoreGive(mutex);
}



void readGasTask(void * parameter) {
  for(;;) {
      readGasSensor();
      vTaskDelay(pdMS_TO_TICKS(GAS_DELAY));
  }
}

void transmitData(String eventName) {
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Preparing transmit data...");
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");

    String authHeader = String("Bearer ") + String(secretKey);
    http.addHeader("Authorization", authHeader.c_str());

    int dataLength = 0;
    if (dataAReady == DATA_PACKET_READY) {
      dataLength = gasDataA.length();
    } else if (dataBReady == DATA_PACKET_READY) {
      dataLength = gasDataB.length();
    }

    // Constructing the JSON
    const size_t capacity = JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(1) + dataLength + 512;
    DynamicJsonDocument doc(capacity);

    doc["name"] = eventName;

    Serial.printf("Reading count: %d\n", readingCount);

    // xSemaphoreTake(mutex, portMAX_DELAY);
    JsonObject data = doc.createNestedObject("data");
    if (dataAReady == DATA_PACKET_READY) {
      data["telemetry"] = gasDataA;
      dataAReady = DATA_PACKET_NOT_READY;
      gasDataA = ""; 
    } else if (dataBReady == DATA_PACKET_READY) {
      data["telemetry"] = gasDataB;
      dataBReady = DATA_PACKET_NOT_READY;
      gasDataB = ""; 
    }
    data["version"] = VERSION_STR;

    String jsonPayload;
    serializeJson(doc, jsonPayload);

    Serial.println("Sending POST request...");
    // Sending POST request
    int httpResponseCode = http.POST(jsonPayload);

    if (httpResponseCode > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String response = http.getString();
        lastSendResponse = response;
        Serial.println(response);
    } else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
    }

    Serial.println("Ending HTTP object...");
    http.end();
    Serial.println("Exiting transmit data...");
  }
}

void transmitDataTask(void * parameter) {
  for(;;) {
      vTaskDelay(pdMS_TO_TICKS(PACKET_SEND_FREQUENCY));
      if (dataAReady == DATA_PACKET_READY || dataBReady == DATA_PACKET_READY) {
        transmitData("mag_data");
      }
  }
}

void readFlashConfig() {
  // Get the flash size in bytes
  uint32_t flashSize = ESP.getFlashChipSize();

  float flashSizeMB = float(flashSize) / (1024 * 1024);
  Serial.print("Flash size (MB): ");
  Serial.println(flashSizeMB, 2);  // Display with 2 decimal places

  uint32_t credentialsAddress;

  if (flashSize == 4 * 1024 * 1024) {
    Serial.println("This is a 4MB flash chip.");
    credentialsAddress = FLASH_ADDRESS_4MB;
  } else if (flashSize == 16 * 1024 * 1024) {
    Serial.println("This is a 16MB flash chip.");
    credentialsAddress = FLASH_ADDRESS_16MB;
  } else {
    Serial.println("This flash chip size is neither 4MB nor 16MB.");
    credentialsAddress = FLASH_ADDRESS_4MB;
  }

  ESP.flashRead(credentialsAddress, (uint32_t*) ssid, sizeof(ssid));
  ESP.flashRead(credentialsAddress + sizeof(ssid), (uint32_t*) password, sizeof(password));
  ESP.flashRead(credentialsAddress + sizeof(ssid) + sizeof(password), (uint32_t*) secretKey, sizeof(secretKey));
}

void sendBootMessage() {
  transmitData("startup");
}

void setup() {
  //Init the serial port communication - to debug the library
  Serial.begin(9600); //Init serial port

  //Set math model to calculate the PPM concentration and the value of constants
  MQ4.setRegressionMethod(1); //_PPM =  a*ratio^b

  MQ4.init(); 
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ4.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ4.calibrate(RatioMQ4CleanAir);
    Serial.print(".");
  }
  MQ4.setR0(calcR0/10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
  /*****************************  MQ CAlibration ********************************************/ 
  //WIA STIFF 
  bool isDualCore = true;
  #ifdef CONFIG_IDF_TARGET_ESP32S2
    // Code specific for ESP32-S2
    Serial.println("Running on ESP32-S2! Not a dual core!");
    isDualCore = false;
  #else
    // Code for other ESP32 variants
    Serial.println("Running on a different ESP32 variant!");
  #endif

  readFlashConfig();

  connectToWiFi();
    
  // mutex = xSemaphoreCreateMutex();

  sendBootMessage();
  
    // If device has more than one core
  if (isDualCore) {
    Serial.println("Is dual core. Using xTaskCreatePinnedToCore");
    xTaskCreatePinnedToCore(
      readGasTask,   /* Task function */
      "ReadGasTask", /* Task name */
      10000,       /* Stack size */
      NULL,        /* Task input */
      1,           /* Priority */
      &Task1,        /* Task handle */
      0);          /* Core */

   xTaskCreatePinnedToCore(
      transmitDataTask,   /* Task function */
      "TransmitDataTask", /* Task name */
      10000,       /* Stack size */
      NULL,        /* Task input */
      1,           /* Priority */
      &Task2,        /* Task handle */
      1);          /* Core */
  } else {
    Serial.println("Has one core. Using xTaskCreate");
    // If device has one core
    xTaskCreate(readGasTask, "ReadGasTask", 2048, NULL, 1, NULL);
    xTaskCreate(transmitDataTask, "TransmitDataTask", 4096, NULL, 1, NULL);
  }

  Serial.println("Finished with setup!");


}

void loop() {
  MQ4.update(); // Update data, the arduino will read the voltage from the analog pin
  vTaskDelay(pdMS_TO_TICKS(1000));
}
