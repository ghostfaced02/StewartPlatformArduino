#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

const char* ssid = "ESP32_IMU_Server";
const char* password = "12345678";

WebServer server(80);

#define I2C_SLAVE_ADDR 3

bool shadowmode = false;

String latestData = "0.0,0.0,480.0,0.0,0.0,0.0;";  // Default IMU string

// Called when Arduino (I2C master) requests data
void onRequest() {
  Serial.println("sending to csp:");
  Serial.println(latestData);
  Wire.write((const uint8_t*)latestData.c_str(), latestData.length()); // Send stored string
}

// Handle incoming IMU data via POST /imu
void handleIMUData() {
  String data = server.arg("plain");  // Read POST data
  Serial.println("Received from phone: " + data);
  latestData = data;
  server.send(200, "text/plain", "Data Received");
}

void setup() {
  Serial.begin(115200);
  
  // Start I2C as slave
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onRequest(onRequest);

  // Create Wi-Fi AP
  WiFi.softAP(ssid, password);
  Serial.println("ESP32 AP Created!");
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Setup HTTP endpoint
  server.on("/imu", HTTP_POST, handleIMUData);
  server.begin();
}

void loop() {
  server.handleClient();  // Handle incoming HTTP requests
}
