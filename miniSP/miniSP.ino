#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#define WINDOW_SIZE 30

// ==== Wi-Fi credentials (can still be used if needed) ====
const char* ssid = "ESP32_IMU_Server";
const char* password = "12345678";

// ==== Receiver endpoint ====
const char* serverURL = "http://192.168.4.1/miniSP";  // optional, can be disabled

// ==== Potentiometer setup ====
int potPins[6] = {36, 39, 34, 35, 32, 33};
int rawValues[6];
float potValues[6]; // now in mm after mapping
int calibration[6] = {25, 28, 41, 10, 0, 50};

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

void setup() {
  Serial.begin(115200);
  Serial.println("time_ms,pot1,pot2,pot3,pot4,pot5,pot6"); // CSV header

  WiFi.begin(ssid, password);
  Serial.println("\nConnecting to receiver Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());

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
    // potValues[i] = mapFloat(avgADC, ADC_MIN, ADC_MAX, MM_MIN, MM_MAX);
    potValues[i] = avgADC;
  }

  // ---- Send data every sendInterval ms ----
  if (millis() - lastSend >= sendInterval) {
    lastSend = millis();

    // Serial output (CSV)
    Serial.print(millis());
    Serial.print(",");
    for (int i = 0; i < 6; i++) {
      Serial.print(potValues[i], 2); // print in mm with 2 decimals
      if (i < 5) Serial.print(",");
    }
    Serial.println();

    // Optional: send to server
    // if (WiFi.status() == WL_CONNECTED) {
    //   HTTPClient http;
    //   http.begin(serverURL);
    //   http.addHeader("Content-Type", "text/plain");

    //   String payload = "";
    //   for (int i = 0; i < 6; i++) {
    //     payload += String(potValues[i], 2);
    //     if (i < 5) payload += ",";
    //   }
    //   payload += ";";
    //   http.POST(payload);
    //   http.end();
    // }
  }
}
