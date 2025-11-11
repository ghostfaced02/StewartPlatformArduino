#include <Wire.h>
#include <PID_v1.h>
#include <Encoder.h>

#define ERROR_MARGIN 2
unsigned long lastUpdate = 0;

int address = 2; // I2C address of the slave device

float val1, val2;
double result[2]; //output vector of leg lenghts

int LPWM_Output[] = {4, 6};
int RPWM_Output[] = {5, 7};

//-----------------------------------------------------------------------------

int clk_pin[] = {18,19};
int dt_pin[] = {3,2};

long oldPosition[] = {-999, -999}; 
double initialPos = 402;
double maxDist = initialPos + 260;
double increment = 78.44/80;

Encoder encoder[] = {
        Encoder(dt_pin[0], clk_pin[0]),
        Encoder(dt_pin[1], clk_pin[1])
};

double input[] = {0,0};              
double output[] = {0,0};

double Kp = 20.0, Ki = 0.01, Kd = 0.1;

PID pid[] = {
    PID(&input[0], &output[0], &result[0], Kp, Ki, Kd, DIRECT),
    PID(&input[1], &output[1], &result[1], Kp, Ki, Kd, DIRECT)
};

void setup() {
  Serial.begin(9600);
  Wire.begin(address); // Slave address
  Wire.onReceive(receiveEvent);
  
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

  Serial.println(input[0]);
  Serial.println(input[1]);

}

void loop(){
    // if (millis() - lastUpdate >= 20) {
    //   lastUpdate = millis();
    //   Serial.print("Position1:");
    //   Serial.print(input[0]);
    //   Serial.print(",");
    //   Serial.print("Position2:");
    //   Serial.println(input[1]);
    // }
}

void receiveEvent(int bytes) {
  if (bytes >= 2 * sizeof(float)) {
    Wire.readBytes((char*)&val1, sizeof(float));
    Wire.readBytes((char*)&val2, sizeof(float));

    result[0] = val1;
    result[1] = val2;

    controlLoop();
  }
}

long newPosition[2];

void controlLoop(){
  for(int i = 0; i<2; i++){
    newPosition[i] = encoder[i].read();
    if (newPosition != oldPosition[i]) {
      oldPosition[i] = newPosition[i];
    }
  }

  for(int i = 0; i<2; i++){
    input[i] = -newPosition[i]*increment+initialPos; 

    double error = abs(result[i] - input[i]);
    if (error > ERROR_MARGIN) {
        pid[i].Compute();

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
