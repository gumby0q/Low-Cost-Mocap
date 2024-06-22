// #include <esp_now.h>
// #include <esp_wifi.h>
// #include <WiFi.h>
// #include <ArduinoJson.h>
// #include <PID_v1.h>
// #include <stdint.h>
// #include <EEPROM.h>
// #include "sbus.h"





// // https://github.com/kkbin505/Arduino-Transmitter-for-ELRS/blob/main/PPMtoCRSF/PPMtoCRSF.ino

// // https://github.com/bolderflight/sbus/blob/main/src/sbus.cpp
// /* -------------------------------------------------------------------------------------- */
// /* -------------------------------------------------------------------------------------- */
// /* -------------------------------------------------------------------------------------- */


// #include <esp_now.h>
// #include <esp_wifi.h>
// #include <WiFi.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include <stdint.h>
#include <EEPROM.h>



// M5-Stick-C Version
// Original ESP32 CRSF Packet Gen Code by :
//
//    https://github.com/AlessandroAU/ESP32-CRSF-Packet-Gen
//
//  Modified by Nicecrash for Flowshutter and betaflight Blackbox logging purposes, GYROflow/FLOWshutter
//
//    Betaflight AETR1234 16 Channel
//    AUX 1  =   ARM HIGH
//    Roll/Pitch/Yaw = MID
//    Thrrottle      = LOW
//
//  LED      =   pin10
//  Tx       =   pin33
//  Rx       =   pin32
//  ButtonA  =   pin37

#include "CRSF.h"

#define LOOP_STATE_STOPPED 0  //  CRSF DISARMED
#define LOOP_STATE_STARTED 1  //  CRSF ARMED

int loopState = LOOP_STATE_STOPPED; // start CRSFDisarmed

const int ledPin1 = 10;       // internal M5Stick-C Red LED pin

// #define CRSFinterval 5000 //in ms
#define CRSFinterval 5000 //in us
#define uartCRSFinverted true

CRSF crsf;

#define CRSF_CHANNEL_VALUE_MIN 172
#define CRSF_CHANNEL_VALUE_MID 992
#define CRSF_CHANNEL_VALUE_MAX 1811

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  crsf.sendRCFrameToFC();
  portEXIT_CRITICAL_ISR(&timerMux);

}

void StartTimer() {
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, CRSFinterval, true);
  timerAlarmEnable(timer);
}

// ----
// uart buffer
char buffer[1024];
// ----
// -------------------------------------------------------------------------------
// -------------------------------------------------------------------------------
// -------------------------------------------------------------------------------

#define ALARM_PIN 34

#define MAX_VEL 100
// #define ROTOR_RADIUS 0.0225
#define ROTOR_RADIUS 0.02

// #define Z_GAIN 0.7
#define Z_GAIN 0.8


#define DRONE_INDEX 0

#define EEPROM_SIZE 4

unsigned long lastPing;
uint64_t millisFromStart = 0;
uint64_t millisFromArm = 0;

#define DELAY_TO_ARM (uint64_t)(1500)

// bfs::SbusTx sbus_tx(&Serial1, 33, 32, true, false);
// bfs::SbusData data;

uint16_t zBase = 173;

bool armed = false;
unsigned long timeArmed = 0;

StaticJsonDocument<1024> json;

int xTrim = 0, yTrim = 0, zTrim = 0, yawTrim = 0;

double groundEffectCoef = 28, groundEffectOffset = -0.035;

// nested pid loops
// outer: position pid loop
// inner: velocity pid loop
// velocity pid loop sends accel setpoint to flight controller
// double xPosSetpoint = -0.421, xPos = 0;
// double yPosSetpoint = 0.154, yPos = 0;
// double zPosSetpoint = 0.296, zPos = 0;

double xPosSetpoint = 0.0, xPos = 0;
double yPosSetpoint = 0.0, yPos = 0;
double zPosSetpoint = 0.0, zPos = 0;

// double yawPosSetpoint = -0.1415, yawPos, yawPosOutput;
double yawPosSetpoint = 0.0, yawPos, yawPosOutput;

double xyPosKp = 1, xyPosKi = 0, xyPosKd = 0;
double zPosKp = 1.5, zPosKi = 0, zPosKd = 0;
double yawPosKp = 0.3, yawPosKi = 0.1, yawPosKd = 0.05;

double xVelSetpoint, xVel, xVelOutput;
double yVelSetpoint, yVel, yVelOutput;
double zVelSetpoint, zVel, zVelOutput;

// double xyVelKp = 0.2, xyVelKi = 0.03, xyVelKd = 0.05;
double xyVelKp = 0.1, xyVelKi = 0.01, xyVelKd = 0.05;
double zVelKp = 0.3, zVelKi = 0.1, zVelKd = 0.05;

// PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection)

// PID::PID(double* Input, double* Output, double* Setpoint,
//         double Kp, double Ki, double Kd, int ControllerDirection)

PID xPosPID(&xPos, &xVelSetpoint, &xPosSetpoint, xyPosKp, xyPosKi, xyPosKd, DIRECT);
PID yPosPID(&yPos, &yVelSetpoint, &yPosSetpoint, xyPosKp, xyPosKi, xyPosKd, DIRECT);
PID zPosPID(&zPos, &zVelSetpoint, &zPosSetpoint, zPosKp, zPosKi, zPosKd, DIRECT);
PID yawPosPID(&yawPos, &yawPosOutput, &yawPosSetpoint, yawPosKp, yawPosKi, yawPosKd, DIRECT);

PID xVelPID(&xVel, &xVelOutput, &xVelSetpoint, xyVelKp, xyVelKi, xyVelKd, DIRECT);
PID yVelPID(&yVel, &yVelOutput, &yVelSetpoint, xyVelKp, xyVelKi, xyVelKd, DIRECT);
PID zVelPID(&zVel, &zVelOutput, &zVelSetpoint, zVelKp, zVelKi, zVelKd, DIRECT);


unsigned long lastLoopTime = micros();
unsigned long lastSbusSend = micros();
unsigned long lastPrint = micros();

float loopFrequency = 2000.0;
float sbusFrequency = 50.0;


// volatile uint16_t xPWM = 992;
// volatile uint16_t yPWM = 992;
// volatile uint16_t zPWM = 992;
// volatile uint16_t yawPWM = 992;

// -------------------------------------------------------------------------------


/* -------------------------------------------------------------- incoming data >>> */


// // callback function that will be executed when data is received
// // void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
// void OnDataRecv(const uint8_t *incomingData, int len) {
//   // Serial.println((char*)incomingData);
//   DeserializationError err = deserializeJson(json, (char *)incomingData);

//   // Serial.println("\n incoming packet \n");

//   if (err) {
//     Serial.print("failed to parse json");
//     return;
//   }

//   if (json.containsKey("pos") && json.containsKey("vel")) {
//     xPos = json["pos"][0];
//     yPos = json["pos"][1];
//     zPos = json["pos"][2];
//     yawPos = json["pos"][3];

//     xVel = json["vel"][0];
//     yVel = json["vel"][1];
//     zVel = json["vel"][2];
//     // xVel = 0;
//     // yVel = 0;
//     // zVel = json["vel"][2];
//     // Serial.println("\n json pos vel!!! \n");
//   } else if (json.containsKey("armed")) {
//     if (json["armed"] != armed && json["armed"]) {
//       timeArmed = millis();
//     }
//     // Serial.printf("\narmed 1 %d \n", (uint8_t)armed);
//     armed = json["armed"];
//     // Serial.printf("\narmed 2 %d \n", (uint8_t)armed);
//   } else if (json.containsKey("setpoint")) {
//     xPosSetpoint = json["setpoint"][0];
//     yPosSetpoint = json["setpoint"][1];
//     zPosSetpoint = json["setpoint"][2];
//     Serial.println("\n json SETPOINT!!! \n");
//   } else if (json.containsKey("pid")) {
//     Serial.println("\n json PID!!! \n");
//     xPosPID.SetTunings(json["pid"][0], json["pid"][1], json["pid"][2]);
//     yPosPID.SetTunings(json["pid"][0], json["pid"][1], json["pid"][2]);
//     zPosPID.SetTunings(json["pid"][3], json["pid"][4], json["pid"][5]);
//     yawPosPID.SetTunings(json["pid"][6], json["pid"][7], json["pid"][8]);

//     xVelPID.SetTunings(json["pid"][9], json["pid"][10], json["pid"][11]);
//     yVelPID.SetTunings(json["pid"][9], json["pid"][10], json["pid"][11]);
//     zVelPID.SetTunings(json["pid"][12], json["pid"][13], json["pid"][14]);

//     groundEffectCoef = json["pid"][15];
//     groundEffectOffset = json["pid"][16];
//   } else if (json.containsKey("trim")) {
//     Serial.println("\n json TRIM!!! \n");
//     xTrim = json["trim"][0];
//     yTrim = json["trim"][1];
//     zTrim = json["trim"][2];
//     yawTrim = json["trim"][3];
//   }

//   lastPing = micros();

// }


#define M_HEADER_0 0xba
#define M_HEADER_1 0xcc

#define M_ID_POS_VEL    0x01
#define M_ID_ARMED      0x02
#define M_ID_SETPOINT   0x03
#define M_ID_PID        0x04
#define M_ID_TRIM       0x05

// Define the polynomial for CRC-8
#define POLYNOMIAL 0x07

// Function to compute the CRC-8 checksum
uint8_t crc8(const uint8_t *data, size_t length) {
    uint8_t crc = 0x00; // Initial value

    for (size_t i = 0; i < length; i++) {
        crc ^= data[i]; // XOR the data byte with the CRC

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ POLYNOMIAL; // Shift left and XOR with the polynomial
            } else {
                crc <<= 1; // Just shift left
            }
        }
    }

    return crc;
}

// Function to convert an array of uint8_t to a float
float array_to_float(uint8_t *buff) {
    // Pointer to float that points to the same memory location as the input buffer
    float result;
    uint8_t *floatPointer = (uint8_t *)&result;

    // Serial.printf("\n");
    // Copy the bytes from the buffer to the float variable
    for (int i = 0; i < 4; i++) {
        floatPointer[i] = buff[i];
        // Serial.printf("\n buff %d \n", buff[i]);
    }
    // Serial.printf("\n result %f \n", result);

    return result;
}

// callback function that will be executed when data is received

void OnDataRecv(const uint8_t *incomingData, int len) {
  if (incomingData[0] == M_HEADER_0) {
    if (incomingData[1] == M_HEADER_1) {
      uint8_t checksum = incomingData[len-1];
      uint8_t _checksum = crc8(incomingData, len-1);

      if (_checksum == checksum) {
        uint8_t id = incomingData[2];

        /* position and velocity */
        if (id == M_ID_POS_VEL) {
          xPos = array_to_float((uint8_t*)&incomingData[3 + 0 * 4]);
          yPos = array_to_float((uint8_t*)&incomingData[3 + 1 * 4]);
          zPos = array_to_float((uint8_t*)&incomingData[3 + 2 * 4]);
          yawPos = array_to_float((uint8_t*)&incomingData[3 + 3 * 4]);

          xVel = array_to_float((uint8_t*)&incomingData[3 + 4 * 4]);;
          yVel = array_to_float((uint8_t*)&incomingData[3 + 5 * 4]);
          zVel = array_to_float((uint8_t*)&incomingData[3 + 6 * 4]);

          // Serial.printf("\n xPos!!! %f %f %f %f\n", xPos, yPos, zPos, yawPos);
          // Serial.printf("\n xVel!!! %f %f %f\n", xVel, yVel, zVel);
        }

        /* armed */
        if (id == M_ID_ARMED) {
          uint8_t status = incomingData[3];
          if (status != armed && status) {
            timeArmed = millis();
          }

          armed = status;
        }

        /* setpoint */
        if (id == M_ID_SETPOINT) {
          xPosSetpoint = array_to_float((uint8_t*)&incomingData[3 + 0 * 4]);
          yPosSetpoint = array_to_float((uint8_t*)&incomingData[3 + 1 * 4]);
          zPosSetpoint = array_to_float((uint8_t*)&incomingData[3 + 2 * 4]);

          // Serial.printf("\n SETPOINT!!! %f %f %f\n", xPosSetpoint, yPosSetpoint, zPosSetpoint);
          Serial.println("\n SETPOINT!!! \n");
        }

        /* pid */
        if (id == M_ID_PID) {
          xPosPID.SetTunings(array_to_float((uint8_t*)&incomingData[3 + 0 * 4]), array_to_float((uint8_t*)&incomingData[3 + 1 * 4]), array_to_float((uint8_t*)&incomingData[3 + 2 * 4]));
          yPosPID.SetTunings(array_to_float((uint8_t*)&incomingData[3 + 0 * 4]), array_to_float((uint8_t*)&incomingData[3 + 1 * 4]), array_to_float((uint8_t*)&incomingData[3 + 2 * 4]));
          zPosPID.SetTunings(array_to_float((uint8_t*)&incomingData[3 + 3 * 4]), array_to_float((uint8_t*)&incomingData[3 + 4 * 4]), array_to_float((uint8_t*)&incomingData[3 + 5 * 4]));
          yawPosPID.SetTunings(array_to_float((uint8_t*)&incomingData[3 + 6 * 4]), array_to_float((uint8_t*)&incomingData[3 + 7 * 4]), array_to_float((uint8_t*)&incomingData[3 + 8 * 4]));

          xVelPID.SetTunings(array_to_float((uint8_t*)&incomingData[3 + 9 * 4]), array_to_float((uint8_t*)&incomingData[3 + 10 * 4]), array_to_float((uint8_t*)&incomingData[3 + 11 * 4]));
          yVelPID.SetTunings(array_to_float((uint8_t*)&incomingData[3 + 9 * 4]), array_to_float((uint8_t*)&incomingData[3 + 10 * 4]), array_to_float((uint8_t*)&incomingData[3 + 11 * 4]));
          zVelPID.SetTunings(array_to_float((uint8_t*)&incomingData[3 + 12 * 4]), array_to_float((uint8_t*)&incomingData[3 + 13 * 4]), array_to_float((uint8_t*)&incomingData[3 + 14 * 4]));

          groundEffectCoef = array_to_float((uint8_t*)&incomingData[3 + 15 * 4]);
          groundEffectOffset = array_to_float((uint8_t*)&incomingData[3 + 16 * 4]);

          Serial.println("\n PID!!! \n");
          // Serial.printf("\n xPosPID!!! %f %f %f\n", array_to_float((uint8_t*)&incomingData[3 + 0 * 4]), array_to_float((uint8_t*)&incomingData[3 + 1 * 4]), array_to_float((uint8_t*)&incomingData[3 + 2 * 4]));
          // Serial.printf("\n zPosPID!!! %f %f %f\n", array_to_float((uint8_t*)&incomingData[3 + 3 * 4]), array_to_float((uint8_t*)&incomingData[3 + 4 * 4]), array_to_float((uint8_t*)&incomingData[3 + 5 * 4]));
          // Serial.printf("\n xVelPID!!! %f %f %f\n", array_to_float((uint8_t*)&incomingData[3 + 9 * 4]), array_to_float((uint8_t*)&incomingData[3 + 10 * 4]), array_to_float((uint8_t*)&incomingData[3 + 11 * 4]));
          // Serial.printf("\n zVelPID!!! %f %f %f\n", array_to_float((uint8_t*)&incomingData[3 + 12 * 4]), array_to_float((uint8_t*)&incomingData[3 + 13 * 4]), array_to_float((uint8_t*)&incomingData[3 + 14 * 4]));
          // Serial.printf("\n groundEffectCoef!!! %f \n", groundEffectCoef);
          // Serial.printf("\n groundEffectOffset!!! %f \n", groundEffectOffset);
        }

        /* trim */
        if (id == M_ID_TRIM) {
          xTrim = (int16_t)array_to_float((uint8_t*)&incomingData[3 + 0 * 4]);
          yTrim = (int16_t)array_to_float((uint8_t*)&incomingData[3 + 1 * 4]);
          zTrim = (int16_t)array_to_float((uint8_t*)&incomingData[3 + 2 * 4]);
          yawTrim = (int16_t)array_to_float((uint8_t*)&incomingData[3 + 3 * 4]);

          // Serial.printf("\n TRIM!!! %i %i %i\n", xTrim, yTrim, zTrim);
          Serial.println("\n TRIM!!! \n");
        }


        lastPing = micros();
      } else {
        Serial.println("\n bad checksum!!! \n");
      }
    }
  }
}

/* -------------------------------------------------------------- incoming data <<< */

void resetPid(PID &pid, double min, double max) {
  pid.SetOutputLimits(0.0, 1.0); 
  pid.SetOutputLimits(-1.0, 0.0);
  pid.SetOutputLimits(min, max);
}


void pid_setup() {
  // Initialize Serial Monitor
  // Serial.begin(115200);

  // sbus_tx.Begin();
  // data.failsafe = false;
  // data.ch17 = true;
  // data.ch18 = true;
  // data.lost_frame = false;

  // for (int j = 0; j < 16; j++) {
  //   data.ch[j] = 173;
  // }

  // data.ch[0] = 992;
  // data.ch[1] = 992;
  // data.ch[2] = 173;
  // data.ch[3] = 992;

  // sbus_tx.data(data);
  // sbus_tx.Write();

  xPosPID.SetMode(AUTOMATIC);
  yPosPID.SetMode(AUTOMATIC);
  zPosPID.SetMode(AUTOMATIC);
  yawPosPID.SetMode(AUTOMATIC);
  xVelPID.SetMode(AUTOMATIC);
  yVelPID.SetMode(AUTOMATIC);
  zVelPID.SetMode(AUTOMATIC);
  
  // Sample rate is determined by main loop
  xPosPID.SetSampleTime(0);
  yPosPID.SetSampleTime(0);
  zPosPID.SetSampleTime(0);
  yawPosPID.SetSampleTime(0);
  xVelPID.SetSampleTime(0);
  yVelPID.SetSampleTime(0);
  zVelPID.SetSampleTime(0);

  xPosPID.SetOutputLimits(-MAX_VEL, MAX_VEL);
  yPosPID.SetOutputLimits(-MAX_VEL, MAX_VEL);
  zPosPID.SetOutputLimits(-MAX_VEL, MAX_VEL);
  yawPosPID.SetOutputLimits(-1, 1);
  xVelPID.SetOutputLimits(-1, 1);
  yVelPID.SetOutputLimits(-1, 1);
  zVelPID.SetOutputLimits(-1, 1);

  EEPROM.begin(EEPROM_SIZE);

  // xTrim = EEPROM.read(0);
  // yTrim = EEPROM.read(1);
  // zTrim = EEPROM.read(2);
  // yawTrim = EEPROM.read(3);
 //------
  lastPing = micros();
  lastLoopTime = micros();
  lastSbusSend = micros();

  // delay(2000);
  millisFromStart = micros()/1000;
}

void input_data_loop() {
  int availableBytes = Serial.available();
  if (availableBytes) {
    // int droneIndex = Serial.read() - '0';
    // Serial.readBytes(buffer, availableBytes-1);
    // buffer[availableBytes-1] = '\0';

    // // Serial.printf("\n drone index %d: ", droneIndex);
    // // Serial.print(buffer);

    // // esp_err_t result = esp_now_send(broadcastAddresses[droneIndex], (uint8_t *)&buffer, strlen(buffer) + 1);
    // // if (result) {
    // //   Serial.println(esp_err_to_name(result));
    // // } else {
    // //   digitalWrite(2, !digitalRead(2));
    // // }

    // // need to parse data and what?
    // OnDataRecv((uint8_t *)&buffer, strlen(buffer) + 1);


    Serial.readBytes(buffer, availableBytes);
    OnDataRecv((uint8_t *)&buffer, availableBytes);

    digitalWrite(2, !digitalRead(2));
  } else {
    // yield();
  }
}

uint32_t max_t_diff = 0;
void pid_loop() {
  while (micros() - lastLoopTime < 1e6 / loopFrequency) {
    // yield();
    return;
  }
  uint32_t  __lastLoopTime = micros();
  uint32_t _max_t_diff = abs(((int64_t)__lastLoopTime - lastLoopTime) - 500);
  // if (_max_t_diff > max_t_diff) {
    max_t_diff = _max_t_diff;
  // }
  lastLoopTime = __lastLoopTime;

  if (micros() - lastPing > 2e6) {
    armed = false;
  }

  bool local_armed = armed;

  int alarm = digitalRead(ALARM_PIN);

  if (alarm != 0) {
    local_armed = false;
  }

  if (local_armed) {
    // data.ch[4] = 1812;
    crsf.PackedRCdataOut.ch4 = CRSF_CHANNEL_VALUE_MAX;      //  CH 4 throttle
  } else {
    /* reset if not armed */
    millisFromArm = micros()/1000;

    // data.ch[4] = 172;
    crsf.PackedRCdataOut.ch4 = CRSF_CHANNEL_VALUE_MIN;      //  CH 4 throttle
    resetPid(xPosPID, -MAX_VEL, MAX_VEL);
    resetPid(yPosPID, -MAX_VEL, MAX_VEL);
    resetPid(zPosPID, -MAX_VEL, MAX_VEL);
    resetPid(yawPosPID, -1, 1);
    resetPid(xVelPID, -1, 1);
    resetPid(yVelPID, -1, 1);
    resetPid(zVelPID, -1, 1);
  }

  if ((micros()/1000 - millisFromArm) > /* delay*/ 1000) {
    zBase = 992;
  } else {
    zBase = 173;
  }


  xPosPID.Compute();
  yPosPID.Compute();
  zPosPID.Compute();
  yawPosPID.Compute();

  xVelPID.Compute();
  yVelPID.Compute();
  zVelPID.Compute();

  int xPWM = 992 + (xVelOutput * 811) + xTrim;
  int yPWM = 992 + (-yVelOutput * 811) + yTrim;
    
  // int xPWM = 992 /* + (xVelSetpoint * 811) */ + xTrim;

  // int xPWM = 992 + (xVelSetpoint * 811) + xTrim;
  // int yPWM = 992 + (-yVelSetpoint * 811) + yTrim;
  int16_t zPWMshift = (Z_GAIN * zVelOutput * 811);
  int zPWM = zBase + zPWMshift + zTrim;
  int yawPWM = 992 + (yawPosOutput * 811) + yawTrim;
  double groundEffectMultiplier = 1 - groundEffectCoef*pow(((2*ROTOR_RADIUS) / (4*(zPos-groundEffectOffset))), 2);
  // zPWM *= max(0., groundEffectMultiplier);
  int _zPWM = zPWM * max(0., groundEffectMultiplier);

  if (zPWM < 173) {
    zPWM = 173;
  }

  // data.ch[0] = -yPWM;
  // data.ch[1] = xPWM;
  // data.ch[2] = zPWM;
  // data.ch[3] = yawPWM;

  if ((micros()/1000 - millisFromArm) > /* delay*/ 1000) {
    // zPWM = 992;
  } else {
    zPWM = 173;
  }

  // data.ch[0] = yPWM;
  // data.ch[1] = xPWM;
  // data.ch[2] = zPWM;
  // data.ch[3] = yawPWM;
  crsf.PackedRCdataOut.ch0 = yPWM;      //  CH 0 ROLL
  crsf.PackedRCdataOut.ch1 = xPWM;      //  CH 1 PITCH
  crsf.PackedRCdataOut.ch2 = zPWM;      //  CH 2 THROTTLE
  crsf.PackedRCdataOut.ch3 = yawPWM;      //  CH 3 YAW
  
  // angle on
  crsf.PackedRCdataOut.ch5 = CRSF_CHANNEL_VALUE_MAX;      //  CH 5 AUX 2 

  // my hack test <<
  // crsf.PackedRCdataOut.ch0 = CRSF_CHANNEL_VALUE_MID;      //  CH 0 ROLL
  // crsf.PackedRCdataOut.ch1 = CRSF_CHANNEL_VALUE_MID;      //  CH 1 PITCH
  // crsf.PackedRCdataOut.ch2 = CRSF_CHANNEL_VALUE_MIN;      //  CH 2 THROTTLE
  // crsf.PackedRCdataOut.ch3 = CRSF_CHANNEL_VALUE_MID;      //  CH 3 YAW

  // data.ch[0] = 992;
  // data.ch[1] = 992;
  // data.ch[2] = 173; // trottle
  // data.ch[3] = 992;
  // data.ch[4] = 173; // arm
  // data.ch[5] = 173; // angle off
  // data.ch[5] = 1811; // angle on
  // if ((micros()/1000 - millisFromStart) > DELAY_TO_ARM) {
  //   data.ch[4] = 1811;
  // }

  // my hack test <<


  // if ((micros() - lastPrint) > /* us */ (500 * 1000)) {
  // if ((micros() - lastPrint) > /* us */ (100 * 1000)) {
  if ((micros() - lastPrint) > /* us */ (200 * 1000)) {
    lastPrint = micros();
    if (local_armed) {
      // Serial.printf("\narmed 3 yes %d %u %d zBase %d \n", (uint8_t)armed, data.ch[4], zVelOutput * 100, zBase);
      // Serial.printf("\narmed 3 yes %d\n", (uint8_t)armed);
    } else {
      // Serial.printf("\narmed 3 no  %d\n", (uint8_t)armed);
    }

    // Serial.printf("xPWM %d yPWM %d zPWM %d yawPWM %d\n", xPWM, yPWM, zPWM, yawPWM);
    // Serial.printf("xPWM %d yPWM %d zPWM %d yawPWM %d\n", xPWM, yPWM, zPWM, yawPWM);   

    Serial.printf("{\"data\":\"\
%d,%d,%d,%d\
,%.2f,%.2f,%.2f,%.2f\
,%.2f,%.2f,%.2f\
,%.2f,%.2f,%.2f,%.2f\
,%.2f,%u,%u,%u\
\"}\n",
      xPWM, yPWM, zPWM, yawPWM,
      xPos, yPos, zPos, yawPos,
      xVelSetpoint, yVelSetpoint, zVelSetpoint,
      xVelOutput, yVelOutput, zVelOutput, yawPosOutput,
      groundEffectMultiplier, millis(), (uint8_t)armed, max_t_diff
    );
  }


  if (micros() - lastSbusSend > 1e6 / sbusFrequency) {
    lastSbusSend = micros();
    // Serial.printf("PWM x: %d, y: %d, z: %d, yaw: %d\nPos x: %f, y: %f, z: %f, yaw: %f\n", xPWM, yPWM, zPWM, yawPWM, xVel, yVel, zPos, yawPos);
    // Serial.printf("Setpoint x: %f, y: %f, z: %f\n", xVelSetpoint, yVelSetpoint, zVelSetpoint);
    // Serial.printf("Pos x: %f, y: %f, z: %f\n", xVel, yVel, zPos);
    //Serial.printf("Output x: %f, y: %f, z: %f\n", xVelOutput, yVelOutput, zVelOutput);
    // sbus_tx.data(data);
    // sbus_tx.Write();

    // crsf.PackedRCdataOut.ch0 = CRSF_CHANNEL_VALUE_MID;      //  CH 0 ROLL
    // crsf.PackedRCdataOut.ch1 = CRSF_CHANNEL_VALUE_MID;      //  CH 1 PITCH
    // crsf.PackedRCdataOut.ch2 = CRSF_CHANNEL_VALUE_MIN;      //  CH 2 THROTTLE
    // crsf.PackedRCdataOut.ch3 = CRSF_CHANNEL_VALUE_MID;      //  CH 3 YAW
    
    // crsf.PackedRCdataOut.ch5 = CRSF_CHANNEL_VALUE_MAX;      //  CH 5 AUX 2
    // // crsf.PackedRCdataOut.ch6 = CRSF_CHANNEL_VALUE_MAX;      //  CH 6 AUX 3
    // // crsf.PackedRCdataOut.ch7 = CRSF_CHANNEL_VALUE_MIN;      //  CH 7 AUX 4
    // // crsf.PackedRCdataOut.ch8 = CRSF_CHANNEL_VALUE_MAX;      //  CH 8 AUX 5

    if ((micros() / 1000 - millisFromStart) > DELAY_TO_ARM)
    {
      // data.ch[4] = 1811;
      // armed = true;
      // crsf.PackedRCdataOut.ch4 = CRSF_CHANNEL_VALUE_MAX;      //  CH 3 YAW
    }
    if ((micros() / 1000 - millisFromStart) > DELAY_TO_ARM + 2000)
    {
      // data.ch[4] = 1811;
      // crsf.PackedRCdataOut.ch2 = CRSF_CHANNEL_VALUE_MIN + 100;      //  CH 3 YAW
    }
  }
}


// -------------------------------------------------------------------------------
// -------------------------------------------------------------------------------
// -------------------------------------------------------------------------------

void setup() {
  
  // Init Serial Monitor
  // Serial.begin(1000000);
  Serial.begin(250000);
  // Serial.begin(500000);

  // csrf
  crsf.Begin();  
  crsf.PackedRCdataOut.ch0 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch1 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch2 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch3 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch4 = CRSF_CHANNEL_VALUE_MIN;
  crsf.sendRCFrameToFC();

  StartTimer();

  // --------------------------------
  pid_setup();

  /* led for indicating transmition */
  pinMode(2, OUTPUT);
  pinMode(ALARM_PIN, INPUT);

  Serial.println("setup finish");
}

void loop() {
  pid_loop();
  input_data_loop();
//  Betaflight AETR1234 
//  CRSF_CHANNEL_VALUE_MIN
//  CRSF_CHANNEL_VALUE_MID
//  CRSF_CHANNEL_VALUE_MAX
}

