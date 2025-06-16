#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
/*********************************************************************
 *  ROSArduinoBridge
 *  ...
 *********************************************************************/

#define USE_BASE      // Enable the base controller code

#ifdef USE_BASE
   #define ARDUINO_ENC_COUNTER
   #define ESCON_MAXON
   #define GRABBER
   #define BATTERY
   #define CUSTOM_SERVOS
   //Adafruit_MPU6050 mpu;
   #define SPIN_BACK  2000
#endif

#undef USE_SERVOS     // Disable use of PWM servos

#define BAUDRATE       57600
#define MAX_PWM        255

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "commands.h"
#include "sensors.h"
#include "sensors_updated.h"

#ifdef USE_SERVOS
  #include <Servo.h>
  #include "servos.h"
#endif

#ifdef USE_BASE
  #include <Servo.h>
  #include "custom_servos.h"
  #include "motor_driver.h"
  #include "encoder_driver.h"
  #include "diff_controller.h"

  #define PID_RATE           30     // Hz
  const int PID_INTERVAL = 1000 / PID_RATE;
  unsigned long nextPID = PID_INTERVAL;

  #define AUTO_STOP_INTERVAL 2000
  long lastMotorCommand   = AUTO_STOP_INTERVAL;
  long lastGrabberCommand = AUTO_STOP_INTERVAL;
  long lastServoCommand   = AUTO_STOP_INTERVAL;
#endif

/* Globals for serial parsing */
int arg = 0;
int index = 0;
char chr;
char cmd;
char argv1[16];
char argv2[16];
long arg1;
long arg2;
bool collector_activated = false;
float left_speed = 0;
float right_speed = 0;
float voltage_battery = 0;
float a_x = 0; float a_y = 0; float a_z = 0;
float g_x = 0; float g_y = 0; float g_z = 0;

const int encoderPinA = A0;
unsigned long switchCount = 0;
bool lastA = LOW;
unsigned long lastReport = 0;
bool grabber = false;
bool stuck = false;
unsigned long stuckTime = 0;
unsigned long checkEncoder = 0;
bool set_time = false;

void battery_level(){
        float value = analogRead(BATTERY_PIN);
        float pin_volt = value / MAX_ARD_VAL * MAX_PIN_VAL;
        float true_bat_volt = pin_volt * (R7 + R8) / R8;
        //Serial.println("Battery Voltage:");
        //Serial.println(true_bat_volt);
        voltage_battery = true_bat_volt;  // Store for later use
    }

/* Encoder/stuck‐detection specifics */
#ifdef USE_BASE
  // We’ll poll A0 as a digital input for the grabber encoder
  #define ENCODER_A_PIN         A0

  volatile unsigned long pulseCount         = 0;    // counts BOTH edges on A0
  uint8_t                lastA0state        = LOW;

  const unsigned long    CHECK_WINDOW       = 500UL;   // 500 ms window
  const unsigned int     MIN_PULSES         = 30;      // ≥30 edges per window ⇒ “turning”
  const unsigned long    REVERSE_DURATION   = 2000UL;  // 2 seconds to reverse when stuck

  unsigned long          lastWindowTime       = 0;
  unsigned long          lastWindowPulseCount = 0;
  unsigned long          reverseUntil         = 0;     // millis until which we reverse
  bool                   currentlyReversing   = false;

  // Default grabber speed (PWM). Adjust as needed.
  const int              DEFAULT_GRABBER_SPEED = 100;
#endif

/* Forward declarations */
void resetCommand();
int runCommand();
void setGrabberSpeed(int spd);

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command. Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd) {

#ifdef USE_BASE
    case READ_ENCODERS:
      Serial.print(readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT));
      break;
    case MOTOR_RAW_PWM:
      lastMotorCommand = millis();
      resetPID();
      moving = 0; // temporarily disable PID
      setMotorSpeeds(arg1, arg2);
      left_speed  = arg1;
      right_speed = arg2;
      //if (stuck){
      //  setGrabberSpeed(80);
      //}
      Serial.println("OK");
      break;
    case GRABBER_RAW_PWM:
      lastGrabberCommand = millis();
      if(arg1!= 0){
        collector_activated = true;
        checkEncoder = millis();
      }
      if(stuck){
        if(millis() - stuckTime > SPIN_BACK){
          stuck = false;
          collector_activated = false;
          //Serial.println("UNSTUCK");
        }
        setGrabberSpeed(-arg1);
      }
      else{
        setGrabberSpeed(arg1);
      }

      if(arg1 == 0) {
        collector_activated = false;
        currentlyReversing  = false;
        reverseUntil        = 0;
        grabber = false;
        stuck = false;
        set_time = false;
      }
      Serial.println("OK");
      break;
    case BATTERY_LEVEL:
      battery_level();
      Serial.print("Battery Voltage: ");
      Serial.println(voltage_battery);
      Serial.println("OK");
      break;
    case ACTIVATE_SERVO:
      lastServoCommand = millis();
      setServo(arg1, arg2);
      Serial.println("OK");
      break;
    case GET_INFO:
      Serial.print(left_speed);
      Serial.print(" ");
      Serial.println(right_speed);
      break;
    case GET_IMU:
      //Serial.print(a_x);
      //Serial.print(" ");
      //Serial.print(a_y);
      //Serial.print(" ");
      //Serial.print(a_z);
      //Serial.print(" ");
      //Serial.print(g_x);
      //Serial.print(" ");
      //Serial.print(g_y);
      //Serial.print(" ");
      //Serial.println(g_z);
      break;
#endif

    default:
      Serial.println("Invalid Command");
      break;
  }
  return 0;
}

/* Setup function—runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);
  //Wire.begin();                    //  ← Initialize I²C bus
  //if (! mpu.begin()) {             //  ← Probe the sensor
  //  Serial.println("Failed to find MPU6050!");
  //  while (1) delay(10);
  //}
  pinMode(LEFT_ENABLE, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIRECTION, OUTPUT);

  pinMode(RIGHT_ENABLE, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIRECTION, OUTPUT);

  pinMode(GRABBER_DIRECTION, OUTPUT);
  pinMode(GRABBER_PWM, OUTPUT);

  pinMode(BATTERY_PIN, INPUT);
  pinMode(encoderPinA, INPUT_PULLUP);
  lastA = digitalRead(encoderPinA);
  //mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  //mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  //mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
#ifdef USE_BASE
  #ifdef ARDUINO_ENC_COUNTER
    // Left encoder pins (directly on Arduino pins)
    DDRD &= ~(1 << LEFT_ENC_PIN_A);
    DDRD &= ~(1 << LEFT_ENC_PIN_B);
    PORTD |= (1 << LEFT_ENC_PIN_A);
    PORTD |= (1 << LEFT_ENC_PIN_B);

    // Right encoder pins
    DDRC &= ~(1 << RIGHT_ENC_PIN_A);
    DDRC &= ~(1 << RIGHT_ENC_PIN_B);
    PORTC |= (1 << RIGHT_ENC_PIN_A);
    PORTC |= (1 << RIGHT_ENC_PIN_B);

    // Enable pin‐change interrupts for left and right encoders
    PCMSK2 |= (1 << LEFT_ENC_PIN_A) | (1 << LEFT_ENC_PIN_B);
    PCMSK1 |= (1 << RIGHT_ENC_PIN_A) | (1 << RIGHT_ENC_PIN_B);
    PCICR  |= (1 << PCIE1) | (1 << PCIE2);
  #endif

  initMotorController();
  initServos();
  resetPID();

  // — Grabber‐encoder polling setup —
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  lastA0state = digitalRead(ENCODER_A_PIN);

  lastWindowTime       = millis();
  lastWindowPulseCount = 0;
  reverseUntil         = 0;
  currentlyReversing   = false;
#endif

#ifdef USE_SERVOS
  for (int i = 0; i < N_SERVOS; i++) {
    servos[i].initServo(
      servoPins[i],
      stepDelay[i],
      servoInitPosition[i]
    );
  }
  delay(100);
#endif
}

/* Main loop: read serial, run commands, PID, auto‐stop, and stuck check */
void loop() {
  //sensors_event_t a, g, temp;
  //mpu.getEvent(&a, &g, &temp);
  // 1) Handle incoming serial commands
  while (Serial.available() > 0) {
    chr = Serial.read();
    if (chr == 13) {               // CR terminates command
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    else if (chr == ' ') {         // space delimits
      if (arg == 0) arg = 1;
      else if (arg == 1) {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        cmd = chr;
      }
      else if (arg == 1) {
        argv1[index++] = chr;
      }
      else if (arg == 2) {
        argv2[index++] = chr;
      }
    }
  }

#ifdef USE_BASE
  // 2) PID loop
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
      // 3) Auto‐stop for drive motors
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0);
    left_speed  = 0;
    right_speed = 0;
    moving = 0;
  }

  // 4) Auto‐stop for grabber (if no grabber command recently)
  if ((millis() - lastGrabberCommand) > AUTO_STOP_INTERVAL) {
    setGrabberSpeed(0);
    collector_activated = false;
    currentlyReversing  = false;
    reverseUntil        = 0;
    grabber = false;
    stuck = false;
    set_time = false;
  }

  // 5) Auto‐stop for servos
  if ((millis() - lastServoCommand) > AUTO_STOP_INTERVAL) {
    setServo(0, 0);
    setServo(1, 0);
  }
  battery_level();
  //a_x = a.acceleration.x;
  //a_y = a.acceleration.y;
  //a_z = a.acceleration.z;
  //g_x = g.gyro.x;
  //g_y = g.gyro.y;
  //g_z = g.gyro.z;

  if (millis() - checkEncoder > 100){
    if(grabber != true && collector_activated == true){
      //Serial.print("CHECKING");
      grabber = true;
    }
  }

  bool nowA = digitalRead(encoderPinA);
  if (nowA != lastA) {
    switchCount++;
    lastA = nowA;
  }
  // report every second
  if (millis() - lastReport >= 1000) {
    //Serial.print("Edges in last 1 s: ");
    //Serial.println(switchCount);
    if (switchCount < 200 && grabber) {
      if(stuck != true){
        //Serial.print("STUCK");
        stuck = true;
        stuckTime = millis();
      }
    }
    switchCount = 0;
    lastReport += 1000;
  }

#endif
#ifdef USE_SERVOS
  for (int i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }

  
#endif
}
