#include <PID_v1.h>
#include <Encoder.h>
#include <Wire.h>

#define ERROR_MARGIN 2
#define PID_INTERVAL 5  // Run PID every 5ms
unsigned long lastUpdate = 0;


//-----------------------------------------------------------------------------
//{22, 23, 24, 26, 28, 45};
//{12, 

int LPWM_Output[] = {5, 7};
int RPWM_Output[] = {4, 6};

//-----------------------------------------------------------------------------


int clk_pin[] = {18, 19};
int dt_pin[] = {3,2};

long oldPosition[] = {-999, -999}; 
double initialPos = 402;
double maxDist = initialPos + 260;
double increment = 78.44/80;

double maxDeg = 14.10;

//-----------------------------------------------------------------------------

double result[6]; //output vector of leg lenghts

double input[] = {0,0};              
double output[] = {0,0};

double Kp = 20.0, Ki = 0.01, Kd = 0.1;



double feedforwardVel[6] = {0,0,0,0,0,0};
double feedforwardAcc[6] = {0,0,0,0,0,0};
double prevResult[6] = {0,0,0,0,0,0};   // previous leg lengths
unsigned long prevFFTime = 0;


PID pid[] = {
    PID(&input[0], &output[0], &result[0], Kp, Ki, Kd, DIRECT),
    PID(&input[1], &output[1], &result[1], Kp, Ki, Kd, DIRECT)
};

long newPosition[2] = {0, 0}; //encoder readings

class coordinate {
  public:
    double x;
    double y;
    double z;
  
    double len() {
      return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    }
};
  
class position {
  public:
    double x;
    double y;
    double z;
    double psi, theta, phi; //roll, pitch, yaw
  
    position(double a, double b, double c, double d, double e, double f) {
      x = a;
      y = b;
      z = c;
      psi = d;
      theta = e;
      phi = f;
    }

    void toArray(double out[6]) {
      out[0] = x;
      out[1] = y;
      out[2] = z;
      out[3] = psi;
      out[4] = theta;
      out[5] = phi;
    }

};

bool operator==(const position& lhs, const position& rhs) {
    return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z &&
            lhs.psi == rhs.psi && lhs.theta == rhs.theta && lhs.phi == rhs.phi);
}

position oldTarget(-999,-999,-999,-999,-999,-999);

class platform {
  public:
    coordinate p[6]; // coordinates of the joints of the upper platform
    coordinate b[6]; // coordinates of the base

    platform() {
      double bOuterR = 461.88/2;
      double bInnerR = 461.88/2;
      double pOuterR = 150;
      double pInnerR = 150;
      for (int i = 0; i < 3; i++) {
        b[i*2].x = bOuterR * cos((120 * i + 90+18.95) * PI / 180); 
        b[i*2].y = bOuterR * sin((120 * i + 90+18.95) * PI / 180);
        b[i*2].z = 0;

        b[i*2+1].x = bInnerR * cos((120 * i + 90+101.05) * PI / 180);
        b[i*2+1].y = bInnerR * sin((120 * i + 90+101.05) * PI / 180);
        b[i*2+1].z = 0;

        p[i*2].x = pOuterR * cos((120 * i + 90 + 40.53) * PI / 180);
        p[i*2].y = pOuterR * sin((120 * i + 90 + 40.53) * PI / 180);
        p[i*2].z = 0; 

        p[i*2+1].x = pInnerR * cos((120 * i + 90 + 79.47) * PI / 180);
        p[i*2+1].y = pInnerR * sin((120 * i + 90 + 79.47) * PI / 180);
        p[i*2+1].z = 0;

      }
    }

    void calcTargetLegLength(position targetPos) {    
      if(targetPos == oldTarget){
        return;
      }

      printPosition(targetPos);

      oldTarget = targetPos;    

      coordinate l; //effective leg position
      //precalculated values
      const double cosphi = cos(targetPos.phi);
      const double costheta = cos(targetPos.theta);
      const double cospsi = cos(targetPos.psi);
      const double sinphi = sin(targetPos.phi);
      const double sintheta = sin(targetPos.theta);
      const double sinpsi = sin(targetPos.psi);

      //rotational matrix
      double Rb[6][6];
      for(int i = 0; i < 6; i++){
        for(int j = 0; j < 6; j++){
          Rb[i][j] = 0;
        }
      }
      
      Rb[0][0] = cospsi * costheta;
      Rb[0][1] = -sinpsi * cosphi + cospsi * sintheta * sinpsi;
      Rb[0][2] = sinpsi * sinphi + cospsi * sintheta * cosphi;
      Rb[1][0] = sinpsi * costheta;
      Rb[1][1] = cospsi * cosphi + sinpsi * sintheta * sinphi;
      Rb[1][2] = -cospsi * sinphi + sinpsi * sintheta * cosphi;
      Rb[2][0] = -sintheta;
      Rb[2][1] = costheta * sinphi;
      Rb[2][2] = costheta * cosphi;
      
      //main calcs
      for (int i = 0; i < 6; i++) {
        l.x = targetPos.x + Rb[0][0] * p[i].x + Rb[0][1] * p[i].y + Rb[0][2] * p[i].z - b[i].x;
        l.y = targetPos.y + Rb[1][0] * p[i].x + Rb[1][1] * p[i].y + Rb[1][2] * p[i].z - b[i].y;
        l.z = targetPos.z + Rb[2][0] * p[i].x + Rb[2][1] * p[i].y + Rb[2][2] * p[i].z - b[i].z;

        double res = sqrt(l.x * l.x + l.y * l.y + l.z * l.z);

        Serial.print(i);
        Serial.println(res);
        
        if(res >= maxDist){
          Serial.println("Target out of range, return to initial pos");
          for (int j = 0; j < 6; j++){
            result[i] = 402;
          }
          return;
        }

        result[i] = res; //leg lengths

        unsigned long now = millis();
        double dt = (now - prevFFTime) / 1000.0; // seconds

        if (dt > 0) {
          for (int i = 0; i < 6; i++) {
            double vel = (result[i] - prevResult[i]) / dt;   // mm/s
            double acc = (vel - ((prevResult[i] - result[i]) / dt)) / dt; // rough acc estimate

            // Save for later use by controlLoop
            feedforwardVel[i] = vel;
            feedforwardAcc[i] = acc;

            prevResult[i] = result[i];
          }
          prevFFTime = now;
        }
      }
      
    }
};

//-----------------------------------------------------------------------------

// Function to convert string to position object
position parsePosition(String input) {
    input.trim();  // Remove leading/trailing whitespace

    double values[6];  // Expected: x, y, z, theta, phi, psi
    int index = 0;
    
    char* str = strdup(input.c_str());  // Convert to C-string
    char* token = strtok(str, ",");     // Tokenize using comma
    
    while (token != nullptr && index < 6) {
        values[index++] = atof(token);  // Convert to double
        token = strtok(nullptr, ",");   // Get next token
    }

    free(str);  // Free memory allocated by strdup

    // Limit angles to ±20 degrees
    if (values[3] > maxDeg) values[3] = maxDeg;
    if (values[3] < -maxDeg) values[3] = -maxDeg;

    if (values[4] > maxDeg) values[4] = maxDeg;
    if (values[4] < -maxDeg) values[4] = -maxDeg;

    // Convert angles to radians
    double theta_rad = values[3] * PI / 180;
    double phi_rad = values[4] * PI / 180;
    double psi_rad = values[5] * PI / 180;

    // Return position object with the 6 values (x, y, z, theta, phi, psi)
    return position(values[0], values[1], values[2], psi_rad, phi_rad, theta_rad);
}

position parseLegLength(String input) {
  Serial.print("INput to parseleglength:");
  Serial.println(input);

  input.trim();  // Remove leading/trailing whitespace

  
  double values[6] = {0, 0, 0, 0, 0, 0};  // Initialize all to 0
  int index = 0;

  // Create a modifiable local copy
  char buffer[64];
  input.toCharArray(buffer, sizeof(buffer));

  char* token = strtok(buffer, ",");
  while (token != nullptr && index < 6) {
    values[index++] = atof(token);
    token = strtok(nullptr, ",");
  }

  // Construct and return a position object
  return position(values[0], values[1], values[2],
                  values[3], values[4], values[5]);
}




//-----------------------------------------------------------------------------


platform p;

void setup() {
  Serial.begin(9600);
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

position target(0,0,500,0,0, 0);
String data;

bool retracked = false;
bool wavemode = false;
bool shadowmode = false;

//Circular wave motion
unsigned long waveStartTime = 0;
double waveRadius = 40;   // mm radius of circular wave
double waveHeight = 50;   // mm amplitude on Z
double waveSpeed = 0.001; // radians/ms (controls speed)


void loop() {
  Serial.println("loop");
  char buffer[64];
  Wire.requestFrom(3, 64);  // Request up to 64 bytes from ESP32

  int i = 0;
  while (Wire.available()) {
    char c = Wire.read();
    if (i < sizeof(buffer) - 1) {
      buffer[i++] = c;
    }
  }
  buffer[i] = '\0';  // Null-terminate the string

  data = String(buffer);

  Serial.println(data);

  if (data.length() == 0) {
    Serial.println("No data received from ESP32 revert to home");
    data = "0.0,0.0,480.0,0.0,0.0,0.0";
  }

  if (data.startsWith("retract")){
    if(retracked){
      return;
    }
    Serial.println("Retracting actuators...");
    sendToSlave(1, 0, 0, 0, 0, 0, 0);
    sendToSlave(2, 0, 0, 0, 0, 0, 0);
    for (int i = 0; i < 2; i++) {
      analogWrite(RPWM_Output[i], 200);
      analogWrite(LPWM_Output[i], 0);
    }
    delay(6000);
    for (int i = 0; i < 2; i++) {
      analogWrite(RPWM_Output[i], 0);
      analogWrite(LPWM_Output[i], 0);
    }
    retracked = true;
    wavemode = false;
    shadowmode = false;
    return;
  } else if (data.startsWith("wave")) {
    if (!wavemode) {
        wavemode = true;
        retracked = false;
        waveStartTime = millis();
        Serial.println("Wave mode started");
    }
  } else if (data.startsWith("shadow")) {
      wavemode = false;
      retracked = false;
      shadowmode = true;
      Serial.println("Shadow mode started");
      target = parseLegLength(data.substring(7));
      printPosition(target);
  } else {
      shadowmode = false;
      wavemode = false;
      target = parsePosition(data);
  }

  if (wavemode) {
    unsigned long t = millis() - waveStartTime;
    double angle = t * waveSpeed;

    // Circular XY trajectory
    //double x = waveRadius * cos(angle);
    //double y = waveRadius * sin(angle);
    
    double x = 10 * cos(angle) *PI/180;
    double y = 10 * sin(angle)*PI/180;

    target = position(0, 0, 500, 0, x, y);
  }
    

  //DEBUG
  //Serial.println(data);  
  // printPosition(target);

  //PID control loop
  if (millis() - lastUpdate >= PID_INTERVAL) {
      lastUpdate = millis();
      if(shadowmode){
        target.toArray(result);
      }else{
        p.calcTargetLegLength(target);
      }


      Serial.println("Printing current target:");
      for(int i=0; i<6; i++){
        Serial.print(result[i]);
        Serial.print(",");
      }
      Serial.println("");

      sendToSlave(1, result[2], result[3], feedforwardVel[2], feedforwardVel[3], feedforwardAcc[2], feedforwardAcc[3]);
      sendToSlave(2, result[4], result[5], feedforwardVel[4], feedforwardVel[5], feedforwardAcc[4], feedforwardAcc[5]);

      controlLoop();

      //DEBUG
      // Serial.print("Position1:");
      // Serial.print(input[0]);
      // Serial.print(",");
      // Serial.print("Position2:");
      // Serial.println(input[1]);

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

        // Feedforward (tune these gains!)
        double Kff_v = 0.0*255/90;   // velocity feedforward gain
        double Kff_a = 0.00*255/90;   // acceleration feedforward gain        
        double ff = Kff_v * feedforwardVel[i] + Kff_a * feedforwardAcc[i];

        // Combine PID + feedforward
        double motorCmd = output[i] + ff;

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


void sendToSlave(byte slaveAddress, double val1, double val2, double vel1, double vel2, double acc1, double acc2) {
  Wire.beginTransmission(slaveAddress);

  Wire.write((uint8_t*)&val1, sizeof(val1));
  Wire.write((uint8_t*)&val2, sizeof(val2));
  Wire.write((uint8_t*)&val1, sizeof(vel1));
  Wire.write((uint8_t*)&val2, sizeof(vel2));
  Wire.write((uint8_t*)&val1, sizeof(acc1));
  Wire.write((uint8_t*)&val2, sizeof(acc2));

  Wire.endTransmission();
}

void printPosition(const position& pos) {
  Serial.print("Position: ");
  Serial.print(pos.x); Serial.print(", ");
  Serial.print(pos.y); Serial.print(", ");
  Serial.print(pos.z); Serial.print(", ");
  Serial.print(pos.psi); Serial.print(" , ");
  Serial.print(pos.theta); Serial.print(", ");
  Serial.println(pos.phi);
}


