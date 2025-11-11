#include <WiFi.h>
#include <esp_now.h>

#define WINDOW_SIZE 30

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x88, 0x13, 0xBF, 0x01, 0x2E, 0x20};

//88:13:bf:01:2e:20


// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  float potValues[6];
} struct_message;

// ==== Potentiometer setup ====
int potPins[6] = {36, 39, 34, 35, 32, 33};
int rawValues[6];
int calibration[6] = {25, 28, 41, 10, 0, 50};

struct_message potValuesMsg;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Rolling average data
int readings[6][WINDOW_SIZE];  // buffer of recent readings
long totals[6];                // running totals
int indices[6];                // circular buffer indices

// Send interval (ms)
unsigned long lastSend = 0;
const unsigned long sendInterval = 5;  // every 5 ms (~200 Hz)

// ==== Mapping range ====
const float ADC_MIN = 0.0;
const float ADC_MAX = 4095.0;
const float MM_MIN = 402.0;
const float MM_MAX = 662.0;

// ---- Float mapping helper ----
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//==================================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("time_ms,pot1,pot2,pot3,pot4,pot5,pot6"); // CSV header

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));

   // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  for (int i = 0; i < 6; i++) {
    totals[i] = 0;
    indices[i] = 0;
    for (int j = 0; j < WINDOW_SIZE; j++) {
      readings[i][j] = 0;
    }
  }
}

void loop() {
  // ---- Read and filter potentiometers ----
  for (int i = 0; i < 6; i++) {
    int newReading = max((analogRead(potPins[i]) - calibration[i]), 0);

    totals[i] -= readings[i][indices[i]];      // remove old value
    readings[i][indices[i]] = newReading;      // add new one
    totals[i] += newReading;                   // update total
    indices[i] = (indices[i] + 1) % WINDOW_SIZE;  // advance circular index

    // Compute average and map to mm
    float avgADC = (float)totals[i] / WINDOW_SIZE;
    rawValues[i] = avgADC;
    potValuesMsg.potValues[i] = mapFloat(avgADC, ADC_MIN, ADC_MAX, MM_MIN, MM_MAX);
  }

  // ---- Send data every sendInterval ms ----
  if (millis() - lastSend >= sendInterval) {
    lastSend = millis();

    // Serial output (CSV)
    Serial.print(millis());
    Serial.print(",");
    for (int i = 0; i < 6; i++) {
      Serial.print(potValuesMsg.potValues[i], 2); // print in mm with 2 decimals
      if (i < 5) Serial.print(",");
    }
    Serial.println();

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &potValuesMsg, sizeof(potValuesMsg));
    
    if (result == ESP_OK) {
        // Serial.println("Sent with success");
    }
    else {
        Serial.println("Error sending the data");
    }
  }
}
