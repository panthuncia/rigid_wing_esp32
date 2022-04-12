/**
 * @file Trim-tab.ino
 * @author Irina Lavryonova (ilavryonova@wpi.edu) - 2019/2020
 * @author Connor Burri (cjburri@wpi.edu) - 2020/2021
 * @author Tom Nurse (tjnurse@wpi.edu) - 2021/2022
 * @brief File containing the execution code for the controller embedded within the adjustable trim tab
 * @version 2.0.2
 * @date 2022-4-11
 * @copyright Copyright (c) 2022
 */

/* Custom struct for the possible states that the trim tab can be in */
typedef enum _TrimState_TRIM_STATE {
    TrimState_TRIM_STATE_MAX_LIFT_PORT = 0,
    TrimState_TRIM_STATE_MAX_LIFT_STBD = 1,
    TrimState_TRIM_STATE_MAX_DRAG_PORT = 2,
    TrimState_TRIM_STATE_MAX_DRAG_STBD = 3,
    TrimState_TRIM_STATE_MIN_LIFT = 4,
    TrimState_TRIM_STATE_MANUAL = 5
} TrimState_TRIM_STATE;

/* File containing all constants for the trim tab to operate; mainly pins and comms */
#include "Constants.h"

/* Libraries */
#include <ArduinoBLE.h>                       // Library for handling Bluetooth Low Energy (BLE) communications
#include <Servo.h>                            // Driver code for operating servo
#include <arduino-timer.h>                    // Library to handle non-blocking function calls at a set interval
#include <Battery.h>                          // Library for monitoring battery level 

/* Battery */
Battery battery = Battery(3000, 4200, batteryPin);

/* BLE variables */
BLEService batteryService("180F");            // Using Bluetooth assigned UUID for Battery service
BLEService sttService("1819");                // Using Bluetooth assigned UUID for Location and Navigation service (0x1819)
BLEService deviceInfoService("180A");         // Using Bluetooth assigned UUID for Device Information service

BLEFloatCharacteristic windDirection("2A73", BLERead | BLENotify);  // Using Bluetooth assigned UUID for apparent wind direction (0x2A73)
BLEIntCharacteristic batteryLevel("2A19", BLERead | BLENotify);     // Using Bluetooth assigned UUID for battery level (0x2A19)
BLEIntCharacteristic tabState("2A6A", BLEWrite);                    // Using Bluetooth assigned UUID for Location and Navigation Feature (0x2A6A) -> Trim Tab State
BLEIntCharacteristic tabAngle("2A6B", BLEWrite);                    // Using Bluetooth assigned UUID for Location and Navigation Control Point (0x2A6B) -> Manual Angle
BLEStringCharacteristic versionString("2A28", BLERead, 20);         // Using Bluetooth assigned UUID for Software Revision String
BLEStringCharacteristic manufacturerString("2A29", BLERead, 64);    // Using Bluetooth assigned UUID for Manufacturer Name String

/* Control variables */
volatile int ledState;                        // For controlling the state of an LED
bool batteryWarning = false;                  // If True, battery level is low (at or below 20%)
bool bleConnected = false;                    // Set to True if a device is connected
auto LEDTimer = timer_create_default();       // Sets the LED timer function to be called asynchronously on an interval
auto servoTimer = timer_create_default();     // Sets the servo timer function to be called asynchronously on an interval
Servo servo;                                  // Servo object
volatile float windAngle;                     // Mapped reading from wind direction sensor on the front of the sail
int control_angle;                            // The current angle that the servo is set to
TrimState_TRIM_STATE state;                   // The variable responsible for knowing what state the trim tab is in

void setup()
{
  /* Setting the mode of the pins necessary */
  pinMode(powerLED, OUTPUT);                  // Green LED
  pinMode(bleLED, OUTPUT);                    // Blue LED
  pinMode(errorLED, OUTPUT);                  // Red LED

  /* Set up battery monitoring */
  battery.begin(3300, 1.43, &sigmoidal);
  
  /* Initializing variables to initial conditions */
  ledState = LOW;                             // LED starts in off position
  control_angle = SERVO_CTR;                  // Trim tab starts off centralized
  state = TrimState_TRIM_STATE_MIN_LIFT;      // The state is set to be in the center position or what we consider to be min lift
  
  /* Starting the serial monitor */
  Serial.begin(115200);
  delay(1000);

  /* Giving feedback that the power is on */
  digitalWrite(powerLED, HIGH);

  /* Set up BLE */
  if (!BLE.begin()) {
    Serial.println("BLE: Failed to start :(");
    while (1) {
      for (int i = 0; i < 3; i++) {
        digitalWrite(errorLED, HIGH);
        delay(500);
        digitalWrite(errorLED, LOW);
        delay(500);
      }
      delay(2000);
    }
  }

  BLE.setDeviceName("Sailbot - Trim Tab");        // The device name characteristic
  BLE.setLocalName("Sailbot - Trim Tab");
  BLE.setAdvertisedService(sttService);           // Sets the primary advertised service

  Serial.print("Device address: ");
  Serial.println(BLE.address());
  
  sttService.addCharacteristic(windDirection);    // Add characteristics to their respective services
  sttService.addCharacteristic(tabState);
  sttService.addCharacteristic(tabAngle);
  batteryService.addCharacteristic(batteryLevel);
  deviceInfoService.addCharacteristic(versionString);
  deviceInfoService.addCharacteristic(manufacturerString);
  
  tabState.writeValue(state);

  BLE.addService(deviceInfoService);              // Add the services
  BLE.addService(batteryService);
  BLE.addService(sttService);

  BLE.advertise();                                // Start BLE
  Serial.println("BLE: Advertising");

  versionString.writeValue("2.0.2");              // Software version
  manufacturerString.writeValue("Worcester Polytechnic Institute - Robotics Engineering");
  
  /* Initializing the servo and setting it to its initial condition */
  servo.attach(servoPin);
  servo.write(control_angle);

  /* Starting the asynchronous function calls */
  servoTimer.every(10, servoControl);
  LEDTimer.every(1000, blinkState);
}

void loop()
{
  BLEDevice central = BLE.central();

  /* If a device is connected */
  if (central) {
	  // Print BLE address
    Serial.print("Connected to ");
    Serial.println(central.address());
    
    while (central.connected()) {
      bleConnected = true;
      if (tabState.written()) {
        if (tabState.value() || tabState.value() == 0) {
          state = (TrimState_TRIM_STATE)tabState.value();
        }
      }

      if (tabAngle.written()) {
        if (tabAngle.value()) {
          control_angle = tabAngle.value();
        }
      }

      batteryLevel.writeValue(battery.level());

      if (battery.level() < 20) {
        batteryWarning = true;
      }else{
        batteryWarning = false;
      }

      if (windAngle) {
        windDirection.writeValue(windAngle);
      }

      servoTimer.tick();
      LEDTimer.tick();
    }

    bleConnected = false;
    Serial.println("Client disconnected");
  }
  
  servoTimer.tick();
  LEDTimer.tick();
}

/**
 * @author Irina Lavryonova
 * @brief Sets the angle of the servo based on the angle of attack read from the encoder at the front
 * @TODO: send this state to the telemetry interface
 */
bool servoControl(void *)
{
  // Read, format, and scale angle of attack reading from the encoder
  windAngle = analogRead(potPin) - POT_HEADWIND;                                            // reads angle of attack data and centers values on headwind
  windAngle = windAngle < 0 ? POT_HEADWIND + windAngle + (1023 - POT_HEADWIND) : windAngle; // wraps angle around
  windAngle = windAngle / 1023.0 * 360.0;                                             // Convert to degrees, positive when wind from 0-180, negative when wind 180-359
  if(windAngle > 180){
    windAngle = (360 - windAngle) * -1;
  }
  //Serial.println(windAngle);
  
  // Set debug LEDs to on to indicate servo control is active
  // digitalWrite(led1Pin, HIGH);
  // digitalWrite(led2Pin, HIGH);

  // Write servo position to one read from the Arduino
  //  servo.write(SERVO_CTR + control_angle - 200 - 90);

  switch(state){
    case TrimState_TRIM_STATE_MAX_LIFT_PORT:
      if (MAX_LIFT_ANGLE > windAngle) {
          control_angle+=2;
      }
      else if ((MAX_LIFT_ANGLE < windAngle)) {
          control_angle-=2;
      }
      control_angle = min(max(control_angle,(SERVO_CTR-55)), (SERVO_CTR+55));
      servo.write(control_angle);
      break;
    case TrimState_TRIM_STATE_MAX_LIFT_STBD:
      windAngle*=-1;
      if(MAX_LIFT_ANGLE > windAngle) {
          control_angle-=2;
      }
      else if ((MAX_LIFT_ANGLE < windAngle)) {
          control_angle+=2;
      }
      control_angle = min(max(control_angle,(SERVO_CTR-55)), (SERVO_CTR+55));
      servo.write(control_angle);
      break;
    case TrimState_TRIM_STATE_MAX_DRAG_PORT:
      servo.write(SERVO_CTR - 55);
      break;
    case TrimState_TRIM_STATE_MAX_DRAG_STBD:
      servo.write(SERVO_CTR + 55);
      break;
    case TrimState_TRIM_STATE_MIN_LIFT:
      servo.write(SERVO_CTR);
      break;
    case TrimState_TRIM_STATE_MANUAL:
      servo.write(control_angle);
      break;
    default:
      servo.write(control_angle);
      break;
  }

  return true;
}

/**
 * @author Irina Lavryonova
 * @author Tom Nurse
 * @brief controls the blinking operations within the LEDS
 */ 
bool blinkState(void *)
{
  // Toggle state
  ledState = ledState == LOW ? HIGH : LOW;

  digitalWrite(powerLED, HIGH);

  if (batteryWarning) {
    digitalWrite(errorLED, HIGH);
  }

  if (bleConnected) {
    digitalWrite(bleLED, ledState);
  }else{
    digitalWrite(bleLED, LOW);
  }

  switch(state){
    case TrimState_TRIM_STATE_MAX_LIFT_PORT:
      digitalWrite(powerLED, ledState);
      break;
    case TrimState_TRIM_STATE_MAX_LIFT_STBD:
      if (ledState == HIGH) {
        delay(100);
        digitalWrite(powerLED, LOW);
        delay(100);
        digitalWrite(powerLED, HIGH);
        delay(100);
        digitalWrite(powerLED, LOW);
      }else{
        digitalWrite(powerLED, LOW);
      }
      break;
    case TrimState_TRIM_STATE_MAX_DRAG_PORT:
      if (ledState == HIGH) {
        delay(100);
        digitalWrite(powerLED, LOW);
        delay(100);
        digitalWrite(powerLED, HIGH);
        delay(100);
        digitalWrite(powerLED, LOW);
        delay(100);
        digitalWrite(powerLED, HIGH);
        delay(100);
        digitalWrite(powerLED, LOW);
      }else{
        digitalWrite(powerLED, LOW);
      }
      break;
    case TrimState_TRIM_STATE_MAX_DRAG_STBD:
      if (ledState == HIGH) {
        delay(100);
        digitalWrite(powerLED, LOW);
        delay(100);
        digitalWrite(powerLED, HIGH);
        delay(100);
        digitalWrite(powerLED, LOW);
        delay(100);
        digitalWrite(powerLED, HIGH);
        delay(100);
        digitalWrite(powerLED, LOW);
        delay(100);
        digitalWrite(powerLED, HIGH);
        delay(100);
        digitalWrite(powerLED, LOW);
      }else{
        digitalWrite(powerLED, LOW);
      }
      break;
    case TrimState_TRIM_STATE_MIN_LIFT:
      if (ledState == HIGH) {
        delay(100);
        digitalWrite(powerLED, LOW);
        delay(100);
        digitalWrite(powerLED, HIGH);
        delay(100);
        digitalWrite(powerLED, LOW);
      }else{
        digitalWrite(powerLED, HIGH);
        delay(100);
        digitalWrite(powerLED, LOW);
        delay(100);
        digitalWrite(powerLED, HIGH);
        delay(100);
        digitalWrite(powerLED, LOW);
      }
      break;
    case TrimState_TRIM_STATE_MANUAL:
      digitalWrite(powerLED, HIGH);
      break;
    default:
      // Water is wet
      break;
  }

  return true;
}
