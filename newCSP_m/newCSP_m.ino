#include <PID_v1.h>
#include <Encoder.h>
#include <Wire.h>

#define ERROR_MARGIN 2
#define PID_INTERVAL 5  // Run PID every 5ms
unsigned long lastUpdate = 0;

//-----------------------------------------------------------------------------
//{22, 23, 24, 26, 28, 45};
//{12, 10, 8, 6, 4, 45};

int LPWM_Output[] = {5, 7};
int RPWM_Output[] = {4, 6};

//-----------------------------------------------------------------------------


int clk_pin[] = {18, 19};
int dt_pin[] = {3,2};

long oldPosition[] = {-999, -999}; 
double initialPos = 402;
double maxDist = initialPos + 260;
double increment = 78.44/80;

//-----------------------------------------------------------------------------

double result[6]; //output vector of leg lenghts
double prevResult[6] = {0,0,0,0,0,0};   // previous leg lengths

double input[] = {0,0};              
double output[] = {0,0};

double Kp = 20.0, Ki = 0.01, Kd = 0.1;


PID pid[] = {
    PID(&input[0], &output[0], &result[0], Kp, Ki, Kd, DIRECT),
    PID(&input[1], &output[1], &result[1], Kp, Ki, Kd, DIRECT)
};

long newPosition[2] = {0, 0}; //encoder readings

//-----------------------------------------------------------------------------

void setup() {
  Serial.begin(115000);
  Wire.begin();
  delay(500);

  Serial.println("Configuring Pins...");
  for (int i = 0; i < 2; i++) {
    pinMode(RPWM_Output[i], OUTPUT);
    pinMode(LPWM_Output[i], OUTPUT);

    pid[i].SetMode(AUTOMATIC);
    pid[i].SetOutputLimits(-120, 120);
  }
  
  Serial.println("Retracting actuators...");
  for (int i = 0; i < 2; i++) {
    analogWrite(RPWM_Output[i], 200);
    analogWrite(LPWM_Output[i], 0);
  }
  delay(6000);
  for (int i = 0; i < 2; i++) {
    analogWrite(RPWM_Output[i], 0);
    analogWrite(LPWM_Output[i], 0);
  }

  Serial.println("Retraction done");
}

//454 mm - minimum leg length 

String data;



void loop() {
  char buffer[128];
  int bytesReceived = Wire.requestFrom(3, 128);  // Request up to 128 bytes from ESP32

  int i = 0;
  unsigned long start = millis();
  while (Wire.available()) {
    if (millis() - start > 100) break; // timeout safety
    char c = Wire.read();
    if (i < sizeof(buffer) - 1) {
        buffer[i++] = c;
    }
  }
  buffer[i] = '\0';  // Null-terminate the string

    String data = String(buffer);

  if (data.length() == 0) {
    Serial.println("No data received from ESP32 revert to home");
    data = "454,454,454,454,454,454;";
  }

  //Serial.println(data);

  // Parse incoming data string into the result[] array (up to 6 values)
  int parsed = parseDataString(data, result, 6);


  if (parsed != 6) {
    for(int i = 0; i<6; i++){
      result[i] = prevResult[i];
    }
  }else{
    for(int i = 0; i<6; i++){
      prevResult[i] = result[i];
    }
  }

  // Serial.print(millis());
  // Serial.print(",");
  // for(int i = 0; i<5 ; i++){
  //   Serial.print(result[i]);
  //   Serial.print(",");
  // }
  // Serial.print(result[5]);
  // Serial.println(";");

  //PID control loop
  if (millis() - lastUpdate >= PID_INTERVAL) {
      lastUpdate = millis();

      sendToSlave(1, result[2], result[3]);
      sendToSlave(2, result[4], result[5]);

      controlLoop();
  }
}

Encoder encoder[] = {
        Encoder(dt_pin[0], clk_pin[0]),
        Encoder(dt_pin[1], clk_pin[1])
};



void controlLoop(){
  for(int i = 0; i<2; i++){
    newPosition[i] = encoder[i].read();
    if (newPosition[i] != oldPosition[i]) {
      oldPosition[i] = newPosition[i];
    }
  }

  for(int i = 0; i<2; i++){
    input[i] = -newPosition[i]*increment+initialPos; 

    double error = abs(result[i] - input[i]);
    if (error > ERROR_MARGIN) {
        pid[i].Compute();

        // Combine PID + feedforward
        double motorCmd = output[i];

        // Apply to motor
        motorControl(motorCmd, i);
    } else {
        motorControl(0, i);
    }

  } 
}

void motorControl(double speed, int motorNum) {
    if (speed > 0) {
        analogWrite(LPWM_Output[motorNum], speed);
        analogWrite(RPWM_Output[motorNum], 0);
    } else {
        analogWrite(LPWM_Output[motorNum], 0);
        analogWrite(RPWM_Output[motorNum], abs(speed));
    }
}


int parseDataString(const String &data, double out[], int maxItems) {
  if (maxItems <= 0) return 0;

  const int BUF_SIZE = 128;
  char buf[BUF_SIZE];
  data.toCharArray(buf, BUF_SIZE);

  // Truncate at first semicolon if present
  for (int i = 0; i < BUF_SIZE && buf[i] != '\0'; ++i) {
    if (buf[i] == ';') { buf[i] = '\0'; break; }
  }

  int count = 0;
  char *tok = strtok(buf, ",");
  while (tok != NULL && count < maxItems) {
    // skip empty tokens
    if (tok[0] != '\0') {
      out[count++] = atof(tok);
    }
    tok = strtok(NULL, ",");
  }

  return count;
}


void sendToSlave(byte slaveAddress, double val1, double val2) {
  Wire.beginTransmission(slaveAddress);

  Wire.write((uint8_t*)&val1, sizeof(val1));
  Wire.write((uint8_t*)&val2, sizeof(val2));
  Wire.endTransmission();
}


