// // https://github.com/kkbin505/Arduino-Transmitter-for-ELRS/blob/main/PPMtoCRSF/PPMtoCRSF.ino

#include <ArduinoJson.h>
#include <PID_v1.h>
#include <stdint.h>


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
#include "./messages.h"

#include "CRSF.h"

#define LOOP_STATE_STOPPED 0  //  CRSF DISARMED
#define LOOP_STATE_STARTED 1  //  CRSF ARMED

int loopState = LOOP_STATE_STOPPED; // start CRSFDisarmed

const int ledPin1 = 10;       // internal M5Stick-C Red LED pin

// #define CRSFinterval 5000 //in ms
#define CRSFinterval 5000 //in us
#define uartCRSFinverted true

#define BUTTONS_TIMER_INTERVAL 1000 //in us
#define BUTTONS_DEBOUNCE_INTERVAL 10 //in ms

CRSF crsf;

#define CRSF_CHANNEL_VALUE_MIN 172
#define CRSF_CHANNEL_VALUE_MID 992
#define CRSF_CHANNEL_VALUE_MAX 1811

// ----
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

// -------------------------------------------------------------------------------
#define BUTTON_ALARM_PIN 34

// #define SWITCH_PIN_ARM   12
// #define BUTTON_PIN_START 13
// #define BUTTON_PIN_STOP  14
#define SWITCH_PIN_ARM   16
#define BUTTON_PIN_START 17
#define BUTTON_PIN_STOP  5

// #define ENCODER_PIN_1    26
// #define ENCODER_PIN_2    27


// volatile uint8_t switch_arm_tmp_state = 0;
volatile uint8_t button_start_tmp_state = 0;
volatile uint8_t button_stop_tmp_state = 0;

volatile uint8_t switch_debounce_arm_cnt = 0;
volatile uint8_t button_debounce_start_cnt = 0;
volatile uint8_t button_debounce_stop_cnt = 0;

volatile uint8_t switch_arm_state = 0;
volatile uint8_t button_start_cnt = 0;
volatile uint8_t button_stop_cnt = 0;
volatile uint8_t button_alarm_state = 0;

#define ENCODER_TRIM_MAX  100
volatile uint8_t encoder_trim_value = 10;


/*

buttons_packet[4]:
- switch_arm_state
- button_start_cnt
- button_stop_cnt
- encoder trim value
- button_alarm_status
*/


// -------------------------------------------------------------------------------

hw_timer_t * timer2 = NULL;
portMUX_TYPE timer2Mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer2() {
  portENTER_CRITICAL_ISR(&timer2Mux);

  // make some buttons read logic
  uint8_t state;
  /* ----- */
  state = digitalRead(SWITCH_PIN_ARM);
  if (state != (!switch_arm_state)) {
    switch_debounce_arm_cnt = switch_debounce_arm_cnt + 1;
    
    if (switch_debounce_arm_cnt >= BUTTONS_DEBOUNCE_INTERVAL) {
      switch_debounce_arm_cnt = BUTTONS_DEBOUNCE_INTERVAL;

      /* pressed */
      switch_arm_state = !state;
    }
  } else {
    switch_debounce_arm_cnt = 0;
  }

  /* ----- */
  state = digitalRead(BUTTON_PIN_START);
  if (state != button_start_tmp_state) {
      button_debounce_start_cnt = button_debounce_start_cnt + 1;

    if (button_debounce_start_cnt >= BUTTONS_DEBOUNCE_INTERVAL-1) {
      button_debounce_start_cnt = BUTTONS_DEBOUNCE_INTERVAL;
      /* pressed */
      button_start_tmp_state = state;
      if (button_start_tmp_state == 0) {
        button_start_cnt = button_start_cnt + 1;
      }
    } else {
    }
  } else {
    button_debounce_start_cnt = 0;
  }

  /* ----- */
  state = digitalRead(BUTTON_PIN_STOP);
  if (state != button_stop_tmp_state) {
    button_debounce_stop_cnt = button_debounce_stop_cnt + 1;

    if (button_debounce_stop_cnt >= BUTTONS_DEBOUNCE_INTERVAL) {
      button_debounce_stop_cnt = BUTTONS_DEBOUNCE_INTERVAL;
      /* pressed */
      button_stop_tmp_state = state;
      if (button_stop_tmp_state == 0) {
        button_stop_cnt = button_stop_cnt + 1;
      }
    } else {
    }
  } else {
    button_debounce_stop_cnt = 0;
  }

  portEXIT_CRITICAL_ISR(&timer2Mux);
}

void StartTimer2() {
  timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer2, &onTimer2, true);
  timerAlarmWrite(timer2, BUTTONS_TIMER_INTERVAL, true);
  timerAlarmEnable(timer2);
}



// ----
// uart buffer
char buffer[1024];
// ----
// -------------------------------------------------------------------------------
// -------------------------------------------------------------------------------
// -------------------------------------------------------------------------------
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



// -------------------------------------------------------------------------------

/* -------------------------------------------------------------- incoming data >>> */

// Function to pack and send data
// void send_status_log(uint8_t message_id, const float *data, size_t data_len, uint8_t error_code) {
void send_status_log(uint8_t message_id, uint8_t error_code) {
    uint8_t buffer[64];
    uint8_t pos = 0;

    // Add headers
    buffer[pos++] = M_HEADER_0;
    buffer[pos++] = M_HEADER_1;
    
    // Add message ID
    buffer[pos++] = message_id;

    // // Add data payload (float values)
    // for (size_t i = 0; i < data_len; i++) {
    //     memcpy(&buffer[pos], &data[i], sizeof(float));
    //     pos += sizeof(float);
    // }

    // Add error code
    buffer[pos++] = error_code;

    // Calculate and add CRC-8 checksum
    uint8_t checksum = crc8(buffer, pos);
    buffer[pos++] = checksum;
    buffer[pos++] = '\r'; /* for transfer parsing.... */
    buffer[pos++] = '\n'; /* for transfer parsing.... */

    // Send the buffer via Serial
    Serial.write(buffer, pos);
}

void send_data_array(uint8_t message_id, uint8_t * data, uint8_t data_length) {
    uint8_t buffer[64];
    uint8_t pos = 0;

    // Add headers
    buffer[pos++] = M_HEADER_0;
    buffer[pos++] = M_HEADER_1;
    
    // Add message ID
    buffer[pos++] = message_id;

    // memcpy(&buffer[pos], data, data_length);
    for (uint8_t i = 0; i < data_length; i++) {
      buffer[pos + i] = data[i];
    }
    pos = pos + data_length;

    // Calculate and add CRC-8 checksum
    uint8_t checksum = crc8(buffer, pos);
    buffer[pos++] = checksum;
    buffer[pos++] = '\r'; /* for transfer parsing.... */
    buffer[pos++] = '\n'; /* for transfer parsing.... */
    // Serial.printf("pos %d data_length %d\n", pos, data_length);
    // Send the buffer via Serial
    Serial.write(buffer, pos);
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
          // Serial.println("\n SETPOINT!!! \n");
          // Serial.printf("{\"type\":\"SET_LOG\",\"data\":\"SETPOINT OK\"}");
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

          send_status_log(M_ID_SETTINGS_PID, M_ERROR_OK);
          // Serial.printf("{\"type\":\"SET_LOG\",\"data\":\"PID OK\"}");
          // Serial.println("\n PID!!! \n");
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

          send_status_log(M_ID_SETTINGS_TRIM, M_ERROR_OK);
          // Serial.printf("\n TRIM!!! %i %i %i\n", xTrim, yTrim, zTrim);
          // Serial.println("\n TRIM!!! \n");
          // Serial.printf("{\"type\":\"SET_LOG\",\"data\":\"TRIM OK\"}");
        }

        lastPing = micros();
      } else {
        send_status_log(M_ID_SETTINGS_CHEKSUM, M_ERROR_GENERIC);
        // Serial.printf("{\"type\":\"SET_LOG\",\"data\":\"SETTINGS BAD CHECKSUM!!!\"}");
        // Serial.println("\n bad checksum!!! \n");
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

  if (button_alarm_state != 0) {
    local_armed = false;
  }
  // int alarm = digitalRead(ALARM_PIN);
  // if (alarm != 0) {
  //   local_armed = false;
  // }

  if (local_armed) {
    // data.ch[4] = 1812;
    crsf.PackedRCdataOut.ch4 = CRSF_CHANNEL_VALUE_MAX;      //  CH 4 arm
    // angle on
    crsf.PackedRCdataOut.ch5 = CRSF_CHANNEL_VALUE_MAX;  //  CH 5 AUX 2

  } else {
    /* reset if not armed */
    millisFromArm = micros()/1000;

    // data.ch[4] = 172;
    crsf.PackedRCdataOut.ch4 = CRSF_CHANNEL_VALUE_MIN;      //  CH 4 arm
    // angle off
    crsf.PackedRCdataOut.ch5 = CRSF_CHANNEL_VALUE_MIN;      //  CH 5 AUX 2 


    resetPid(xPosPID, -MAX_VEL, MAX_VEL);
    resetPid(yPosPID, -MAX_VEL, MAX_VEL);
    resetPid(zPosPID, -MAX_VEL, MAX_VEL);
    resetPid(yawPosPID, -1, 1);
    resetPid(xVelPID, -1, 1);
    resetPid(yVelPID, -1, 1);
    resetPid(zVelPID, -1, 1);
  }

  /* todo: rewrite to statemachine */
  if ((micros()/1000 - millisFromArm) > /* delay*/ 1000) {
    zBase = CRSF_CHANNEL_VALUE_MID;
  } else {
    zBase = CRSF_CHANNEL_VALUE_MIN;
  }


  xPosPID.Compute();
  yPosPID.Compute();
  zPosPID.Compute();
  yawPosPID.Compute();

  xVelPID.Compute();
  yVelPID.Compute();
  zVelPID.Compute();

  int xPWM = CRSF_CHANNEL_VALUE_MID + (xVelOutput * 811) + xTrim;
  int yPWM = CRSF_CHANNEL_VALUE_MID + (-yVelOutput * 811) + yTrim;

  int16_t zPWMshift = (Z_GAIN * zVelOutput * 811);
  int zPWM = zBase + zPWMshift + zTrim;
  
  int yawPWM = CRSF_CHANNEL_VALUE_MID + (yawPosOutput * 811) + yawTrim;

  /* todo: check if it works right? */
  double groundEffectMultiplier = 1 - groundEffectCoef*pow(((2*ROTOR_RADIUS) / (4*(zPos-groundEffectOffset))), 2);
  
  int _zPWM = zPWM * max(0., groundEffectMultiplier);

  /* trim value */
  if (zPWM < CRSF_CHANNEL_VALUE_MIN) {
    zPWM = CRSF_CHANNEL_VALUE_MIN;
  } else if (zPWM > CRSF_CHANNEL_VALUE_MAX) {
    zPWM = CRSF_CHANNEL_VALUE_MAX;
  }

  if ((micros()/1000 - millisFromArm) > /* delay*/ 1000) {

  } else {
    zPWM = CRSF_CHANNEL_VALUE_MIN;
  }

  crsf.PackedRCdataOut.ch0 = yPWM;      //  CH 0 ROLL
  crsf.PackedRCdataOut.ch1 = xPWM;      //  CH 1 PITCH
  crsf.PackedRCdataOut.ch2 = zPWM;      //  CH 2 THROTTLE
  crsf.PackedRCdataOut.ch3 = yawPWM;      //  CH 3 YAW
  
  // my hack test >>
  // crsf.PackedRCdataOut.ch0 = CRSF_CHANNEL_VALUE_MID;      //  CH 0 ROLL
  // crsf.PackedRCdataOut.ch1 = CRSF_CHANNEL_VALUE_MID;      //  CH 1 PITCH
  // crsf.PackedRCdataOut.ch2 = CRSF_CHANNEL_VALUE_MIN;      //  CH 2 THROTTLE
  // crsf.PackedRCdataOut.ch3 = CRSF_CHANNEL_VALUE_MID;      //  CH 3 YAW

  if ((micros() - lastPrint) > /* us */ (200 * 1000)) {
    lastPrint = micros();
    if (local_armed) {
      // Serial.printf("\narmed 3 yes %d %u %d zBase %d \n", (uint8_t)armed, data.ch[4], zVelOutput * 100, zBase);
      // Serial.printf("\narmed 3 yes %d\n", (uint8_t)armed);
    } else {
      // Serial.printf("\narmed 3 no  %d\n", (uint8_t)armed);
    }
    // Serial.printf("xPWM %d yPWM %d zPWM %d yawPWM %d\n", xPWM, yPWM, zPWM, yawPWM);   

    Serial.printf("{\"type\":\"PID_LOG\",\"data\":\"\
%d,%d,%d,%d\
,%.2f,%.2f,%.2f,%.2f\
,%.2f,%.2f,%.2f\
,%.2f,%.2f,%.2f,%.2f\
,%.2f,%u,%u,%u\
\"}\r\n",
      xPWM, yPWM, zPWM, yawPWM,
      xPos, yPos, zPos, yawPos,
      xVelSetpoint, yVelSetpoint, zVelSetpoint,
      xVelOutput, yVelOutput, zVelOutput, yawPosOutput,
      groundEffectMultiplier, millis(), (uint8_t)armed, max_t_diff
    );
  }

  // if (micros() - lastSbusSend > 1e6 / sbusFrequency) {
  //   if ((micros() / 1000 - millisFromStart) > DELAY_TO_ARM + 2000)
  //   {
  //     // data.ch[4] = 1811;
  //     // crsf.PackedRCdataOut.ch2 = CRSF_CHANNEL_VALUE_MIN + 100;      //  CH 3 YAW
  //   }
  //}
}

// -------------------------------------------------------------------------------
// -------------------------------------------------------------------------------
// -------------------------------------------------------------------------------

void setup() {
  // Init Serial Monitor
  // Serial.begin(1000000);
  // Serial.begin(500000);
  Serial.begin(250000);

  // --------------------------------
  // csrf
  crsf.Begin();  
  crsf.PackedRCdataOut.ch0 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch1 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch2 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch3 = CRSF_CHANNEL_VALUE_MIN;
  crsf.PackedRCdataOut.ch4 = CRSF_CHANNEL_VALUE_MIN;
  crsf.sendRCFrameToFC();

  // --------------------------------
  StartTimer();  /* crsf */
  StartTimer2(); /* buttons */
  // --------------------------------
  
  pid_setup();

  // --------------------------------
  pinMode(2, OUTPUT); /* led for indicating transmition */
  
  /* buttons */
  pinMode(BUTTON_ALARM_PIN, INPUT);
  pinMode(SWITCH_PIN_ARM, INPUT);
  pinMode(BUTTON_PIN_START, INPUT);
  pinMode(BUTTON_PIN_STOP, INPUT);

  Serial.println("setup finish");
}

uint64_t last_controls_send = 0;

void loop() {
  /* buttons start... */
  button_alarm_state = digitalRead(BUTTON_ALARM_PIN);
  
  uint64_t current_ms = millis();
  if ((current_ms - last_controls_send) > /* ms */ (100)) {
    last_controls_send = current_ms;

    // - switch_arm_state
    // - button_start_cnt
    // - button_stop_cnt
    // - encoder trim value
    uint8_t data[5];
    data[0] = switch_arm_state;
    data[1] = button_start_cnt;
    data[2] = button_stop_cnt;
    data[3] = encoder_trim_value;
    data[4] = button_alarm_state;

    send_data_array(M_ID_CONTROLS_STATE, data, 5);
  }
  /* buttons ...end */
  
  pid_loop();
  input_data_loop();
}

