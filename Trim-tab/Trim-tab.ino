/**
 * @file Trim-tab.ino
 * @author Irina Lavryonova (ilavryonova@wpi.edu) - 2019/2020
 * @author Connor Burri (cjburri@wpi.edu) - 2020/2021
 * @author Tom Nurse (tjnurse@wpi.edu) - 2021/2022
 * @author Matthew Gomes (mhgomes@wpi.edu) - 2023-2024
 * @brief File containing the execution code for the controller embedded within the adjustable trim tab
 * @version 2.0.2
 * @date 2023-11-12
 * @copyright Copyright (c) 2023
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
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>  // This is needed for notifications

#include <ESP32Servo.h>                            // Driver code for operating servo
#include <arduino-timer.h>                    // Library to handle non-blocking function calls at a set interval
#include <Battery.h>                          // Library for monitoring battery level 

/* Battery */
Battery battery = Battery(3000, 4200, batteryPin);

/* BLE */
BLECharacteristic *windDirection;
BLECharacteristic *batteryLevel;
BLECharacteristic *tabState;
BLECharacteristic *tabAngle;
BLECharacteristic *versionString;
BLECharacteristic *manufacturerString;

BLEService *sttService;
BLEService *batteryService;
BLEService *deviceInfoService;
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
  //BLE.setSupervisionTimeout(100); //not available in esp32 BLE?
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
  // if (!BLE.begin()) {
  //   Serial.println("BLE: Failed to start :(");
  //   while (1) {
  //     for (int i = 0; i < 3; i++) {
  //       digitalWrite(errorLED, HIGH);
  //       delay(500);
  //       digitalWrite(errorLED, LOW);
  //       delay(500);
  //     }
  //     delay(2000);
  //   }
  // } //not needed?

  BLEDevice::init("Sailbot - Trim Tab"); // Name your device
  BLEServer *pServer = BLEDevice::createServer();
  //BLE.setAdvertisedService(sttService);           // Sets the primary advertised service
  BLEService *sttService = pServer->createService("1819");
  BLEService *batteryService = pServer->createService("180F");
  BLEService *deviceInfoService = pServer->createService("180A");

  Serial.print("Device address: ");
  //Serial.println(BLE.address());
  
  // Create BLE Characteristics
  windDirection = batteryService->createCharacteristic(
                                    "2A73",
                                    BLECharacteristic::PROPERTY_READ | 
                                    BLECharacteristic::PROPERTY_NOTIFY
                                  );
  batteryLevel = batteryService->createCharacteristic(
                                    "2A19",
                                    BLECharacteristic::PROPERTY_READ | 
                                    BLECharacteristic::PROPERTY_NOTIFY
                                  );
  tabState = sttService->createCharacteristic(
                                "2A6A",
                                BLECharacteristic::PROPERTY_WRITE
                              );
  tabAngle = sttService->createCharacteristic(
                                "2A6B",
                                BLECharacteristic::PROPERTY_WRITE
                              );
  versionString = deviceInfoService->createCharacteristic(
                                      "2A28",
                                      BLECharacteristic::PROPERTY_READ
                                    );
  manufacturerString = deviceInfoService->createCharacteristic(
                                          "2A29",
                                          BLECharacteristic::PROPERTY_READ
                                        );

  
  windDirection->addDescriptor(new BLE2902());
  batteryLevel->addDescriptor(new BLE2902());
  int current_state = state;
  tabState->setValue(current_state);

  deviceInfoService->start();
  batteryService->start();
  sttService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(batteryService->getUUID());
  pAdvertising->addServiceUUID(sttService->getUUID());
  pAdvertising->addServiceUUID(deviceInfoService->getUUID());
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // Functions that help with iPhone connections issue
  BLEDevice::startAdvertising();                               // Start BLE
  Serial.println("BLE: Advertising");

  versionString->setValue("2.0.2");              // Software version
  manufacturerString->setValue("Worcester Polytechnic Institute - Robotics Engineering");
  
  /* Initializing the servo and setting it to its initial condition */
  servo.attach(servoPin);
  servo.write(control_angle);

  /* Starting the asynchronous function calls */
  servoTimer.every(10, servoControl);
  LEDTimer.every(1000, blinkState);
}

void loop()
{
    if (tabState->getValue().length() > 0) {
        int tabStateValue = *(tabState->getData()); // Get the value as int
        if (tabStateValue || tabStateValue == 0) {
            state = (TrimState_TRIM_STATE)tabStateValue;
        }
    }

    if (tabAngle->getValue().length() > 0) {
        int tabAngleValue = *(tabAngle->getData()); // Get the value as int
        control_angle = tabAngleValue;
    }

    // Writing to a characteristic is similar to Arduino BLE
    std::string batteryLevelValue = std::to_string(battery.level());
    batteryLevel->setValue(batteryLevelValue);
    batteryLevel->notify(); // Notify if needed

    if (battery.level() < 20) {
        batteryWarning = true;
    } else {
        batteryWarning = false;
    }

    if (windAngle) {
        windDirection->setValue(std::to_string(windAngle));
        windDirection->notify(); // Notify if needed
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
      // No, it's not.
      break;
  }

  return true;
}
