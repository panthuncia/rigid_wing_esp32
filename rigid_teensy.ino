#include <Servo.h>

#define servoOffset 96 //offset to make 0 degrees in code equal to 0 degrees on the tab
#define maxLiftAngle 30 //angle calculated for maximum lift from sail

//Pins for devices
#define potPin A0
#define servoPin 20
#define liftPin 2
#define dragPin 6
#define windSidePin 3
#define led1Pin 36
#define led2Pin 37
#define controlPin 11
#define angleControlPin A3
#define wifiLED 38
#define powerLED 13
#define vInPin A2

#define SSID "sailbot"
#define PASS "Passphrase123"
#define DST_IP "192.168.0.21"
#define DST_PORT 3333
#define THIS_DEVICE_IP "192.168.0.25"


int control = 0; //to enable direct control over tab angle
int lift = 0; //0 to produce no lift 1 to produce lift
int drag = 0;
int windSide = 0; //0 for wind from port 1 for wind from starboard
bool manual = false;

int heelIn; //reading from hull heel sensor
int heelAngle = 0; //mapped heel angle, 0 degrees is straight up 90 would be on its side
int maxHeelAngle = 30;//settable max heel angle

int angleIn;//reading from wind direction sensor on the front of the sail
int readAttackAngle; //mapped value from wind sensor
int sentAttackAngle; //value mapped to correct sending format

int controlAngle = 0; //manual angle set by boat

int tabAngle = 0; //angle of tab relative to centered being 0

int count = 0; //count to have leds blink

int state;
int printing = 0;
int tcpConnection = 0;
int connectionCount = 0;

int ledState = LOW;
unsigned long previousMillis = 0;
volatile unsigned long blinkCount = 0; // use volatile for shared variables

int servoAngle;

IntervalTimer LEDtimer;
IntervalTimer servoTimer;

Servo servo;

void setup() {
  //init
  pinMode(potPin, INPUT);
  pinMode(liftPin, INPUT);
  pinMode(dragPin, INPUT);
  pinMode(windSidePin, INPUT);
  pinMode(controlPin, INPUT);
  pinMode(angleControlPin, INPUT);
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  pinMode(wifiLED, OUTPUT);
  pinMode(powerLED, OUTPUT);
  pinMode(vInPin, INPUT);
  servo.attach(servoPin);

  // Initialize Everything
  initializeComs();
  initializeWifi();

  // Connect to the network
  digitalWrite(wifiLED, LOW);
  connectToNetwork(SSID, PASS);

  LEDtimer.begin(blinkState, 916682);
  servoTimer.begin(servoControl, 50000);

  servo.write(servoOffset); //in place so lift starts at 0 degrees or neutral state

  digitalWrite(powerLED, HIGH);// turn on power led
}

void loop() {
  //delay(50);  for serial testing no wifi
  //----------------------------------------------------------------------
  //Wifi communication and message parsing

  if (Serial.available() > 0) {
    // read the incoming byte:
    state = Serial.read() - 48;

    Serial.print("State:");
    Serial.print(state);
  }

  int vIn = analogRead(vInPin);

  if (windSide) {
    servoAngle = tabAngle + 60;
  }
  else {
    servoAngle = -tabAngle + 60;
  }


  sentAttackAngle = (360 + readAttackAngle) % 360;

  //Serial.print("  Angle of Attack:");
  //Serial.print(readAttackAngle);

  //Serial.print("  Servo Angle:");
  //Serial.println(tabAngle);

  stateSet();

  if (connectedTCP()) {
    connectionCount = 0;
    digitalWrite(wifiLED, HIGH);

    sendBoatMessage(sentAttackAngle, servoAngle, vIn);  //message sent to hull
    delay(10);                    //delay for message to send before recieving

    if (readMessage(250)) {
      //Serial.print("S: ");
      //Serial.print(state);
      //Serial.print(", A:");
      //Serial.print(heelAngle);
      //Serial.print(", B:");
      //Serial.print(maxHeelAngle);
      //Serial.print(", C:");
      //Serial.println(controlAngle);
    }

  } else {
    connectionCount++;
    if (connectionCount >= 4) {
      control = 0;
      lift = 0;
      drag = 0;
    }
    openTCP(DST_IP, DST_PORT);    //if no message is recieved than there is no connection so the port is openend
    delay(50);
  }
}





void sendBoatMessage(int wind, int servoPos, int volt) {
  String msg = "[";
  msg += addZerosToString(wind, 3) + ",";
  msg += addZerosToString(servoPos, 3) + ",";
  msg += addZerosToString(volt, 3) + "]";

  sendTCPMessage(msg);
}


String addZerosToString(int n, int z) {
  String result = String(n);

  int s = 10;

  while (s < pow(10, z)) {
    if (s >= n) {
      result = "0" + result;
    }
    s = s * 10;
  }

  return result;
}



// This initializes the serial buses
int initializeComs() {
  Serial.begin(115200);
  Serial4.begin(115200);

  if (printing) Serial.println("Communication Initialized");

  return 0;
}



// This initializes the ESP8266 module
int initializeWifi() {

  // Reset the module
  sendMessageToESP("AT+RST");

  if (printing) Serial.println("Resetting Wifi Module");

  // wait for a "ready" command
  bool reset_successful = waitForStringSerial4("ready", 3000);

  if (reset_successful) {
    if (printing) Serial.println("Wifi Reset Successfully");
    return 0;

  } else {
    if (printing) Serial.println("Wifi Reset Failed");
    return 1;
  }
}



// This scans for networks and returns a list of networks
int scanForNetworks() {
  // Send the command to print all nearby networks
  sendMessageToESP("AT+CWLAP");

  // TODO: print out all networks
  return 0;
}



// This searches for networks and returns true if the selected network is found
int searchForNetwork(String networkName) {
  return 0;
}



// This attempts to connect to a network. If it is succesful, True is returned
bool connectToNetwork(String ssid, String password) {

  if (printing) {
    Serial.println("Attempting to connect to " + ssid);
    Serial.println("Password is " + password);
  }

  // Maybe search for network to see it it's available first?

  // Set the operating mode to Client
  // Client = 1, AP = 2, Client and AP = 3
  sendMessageToESP("AT+CWMODE=1");

  // Build the message to connect to the given ssid with the password
  String cmd = "AT+CWJAP=\"" + ssid + "\",\"" + password + "\"";
  sendMessageToESP(cmd);

  // wait for a "OK" command
  bool connection_successful = waitForStringSerial4("OK", 3000);

  cmd = "AT+CIPSTA=\"" THIS_DEVICE_IP "\"";
  sendMessageToESP(cmd);

  // wait for a "OK" command
  connection_successful = connection_successful && waitForStringSerial4("OK", 3000);

  if (connection_successful) {
    if (printing) Serial.println("Connection Successful");
    return true;

  } else {
    if (printing) Serial.println("Connection Failed");
    return false;
  }
}



// Get ip address if it's connected to a network
String getIP() {
  sendMessageToESP("AT_CIFSR");

  // Sort out IP address
  return "0.0.0.0";
}



// Open a TCP connection
// A returned value of True indicates it was successful
boolean openTCP(String ip, int port) {
  // Set transparent mode to 1 so that messages recieved will be sent directly to serial
  // Set transparent mode to 0
  //  sendMessageToESP("AT+CIPMODE=0", printing);

  // build command
  String cmd = "AT+CIPSTART=\"TCP\",\"" + ip + "\"," + port;

  sendMessageToESP(cmd);
  //  Serial.println(cmd);

  // wait for a "OK" command
  bool connection_successful = waitForStringSerial4("OK", 3000);

  if (connection_successful) {
    if (printing) Serial.println("TCP Connection to " + ip + " port number " + String(port) + " successful");
    return true;
  } else {
    if (printing) Serial.println("TCP Connection to " + ip + " port number " + String(port) + " failed");
    return false;
  }
}



// Send a message over TCP()
void sendTCPMessage(String msg) {

  // build initial message
  String instructionToSend = "AT+CIPSEND=" + String(msg.length());

  if (printing) Serial.println("Sending message: " + msg);

  // Send the message
  sendMessageToESP(instructionToSend);
  delay(20);
  sendMessageToESP(msg);
}



// Close the current TCP connection
int closeTCP() {
  sendMessageToESP("AT+CIPCLOSE");

  if (printing) Serial.println("TCP Closed");

  return 0;
}



// Return true if connected to TCP, false otherwise
bool connectedTCP() {
  sendMessageToESP("AT+CIPSTATUS");

  if (waitForStringSerial4("STATUS:3", 500)) {
    if (printing) Serial.println("TCP still connected");
    tcpConnection = 1;
    return true;
  } else {
    if (printing) Serial.println("TCP connection lost");
    tcpConnection = 0;
    return false;
  }
}







bool readMessage(int timeout) {
  int start_time = millis();

  bool recievedNewData = false;
  String data = "";

  //  "[1,180,180,100]"

  while (millis() < start_time + timeout) {
    if (Serial4.available()) {
      data += Serial4.readString();
      Serial.println(data);

      for (int i = 0; i < data.length(); i++) {
        if (data.substring(i, i + 1) == "[") {
          if (data.length() > i + 15) {
            String validData = data.substring(i, i + 15);

            // Serial.println("Special string: " + validData);
            state =             validData.substring(1, 2).toInt();
            heelAngle =         validData.substring(3, 6).toInt();
            maxHeelAngle =      validData.substring(7, 10).toInt();
            controlAngle =   validData.substring(11, 14).toInt() - 50;

            recievedNewData = true;
            return true;
          }
        }
      }
    }
  }

  return recievedNewData;
}












void sendMessageToESP(String commandToSend) {
  Serial4.println(commandToSend);

  if (printing >= 2) Serial.println("--- " + commandToSend);
}



// This method scans the input from Serial4 for a specific key
// If this key is found before the timeout, true is returned.
// Othertime false is returned
bool waitForStringSerial4(String key, int timeout) {

  int start_time = millis();

  while (millis() < start_time + timeout) {
    if (Serial4.available()) {
      String data = Serial4.readString();
      // Serial.println(data);

      for (int i = 0; i < data.length() - key.length(); i++) {
        if (data.substring(i, i + key.length()) == key) {
          return true;
        }
      }
    }
  }

  return false;
}

void blinkState() {
  if (ledState == LOW) {
    ledState = HIGH;
    blinkCount = blinkCount + 1;  // increase when LED turns on
  } else {
    ledState = LOW;
  }
  if (!tcpConnection) {
    digitalWrite(wifiLED, ledState);
  }
  if (lift) {
    if (windSide) {
      digitalWrite(led1Pin, HIGH);
      digitalWrite(led2Pin, LOW);
    }
    else {
      digitalWrite(led2Pin, LOW);
      digitalWrite(led1Pin, ledState);
    }
  }
  if (drag) {
    if (windSide) {
      digitalWrite(led2Pin, HIGH);
      digitalWrite(led1Pin, LOW);
    }
    else {
      digitalWrite(led1Pin, LOW);
      digitalWrite(led2Pin, ledState);
    }
  }
}

void stateSet() {
  if (state == 0) {
    control = 0;
    lift = 0;
    drag = 0;
  }
  else if (state == 1) {
    control = 0;
    lift = 1;
    drag = 0;
    windSide = 1;
  }
  else if (state == 2) {
    control = 0;
    lift = 1;
    drag = 0;
    windSide = 0;
  }
  else if (state == 3) {
    control = 0;
    lift = 0;
    drag = 1;
    windSide = 1;
  }
  else if (state == 4) {
    control = 0;
    lift = 0;
    drag = 1;
    windSide = 0;
  }
  else if (state == 7) {
    control = 1;
    lift = 0;
    drag = 0;
  }
}

void servoControl() {

  angleIn = analogRead(potPin); // reads angle of attack data
  readAttackAngle = angleIn * 0.3442 - 122.93;
  //---------------------------------------------------------------------------------------------------
  //set for manual control
  if (control) {
    digitalWrite(led1Pin, HIGH);
    digitalWrite(led2Pin, HIGH);
    servo.write(servoOffset + controlAngle);
  }

  //------------------------------------------------------------------------------------------------------
  //when lift is desired
  if (lift) {

    if (!windSide) {
      readAttackAngle = readAttackAngle * -1;
    }

    //if the lift angle isnt enough and the heel angle isnt too much the angle of attack is increased
    if ((maxLiftAngle > readAttackAngle+1)) {  //&& (abs(heelAngle) <= maxHeelAngle))) {
      if (tabAngle >= 55) { }
      else {
        tabAngle++;
      }
    }

    //if the lift angle is too much or the max heel angle is too much the sail lightens up
    else if ((maxLiftAngle < readAttackAngle)) {  //&& (abs(heelAngle) <= maxHeelAngle)) || (abs(heelAngle) >= maxHeelAngle)) {
      if (tabAngle <= -55) {  }
      else {
        tabAngle--;
      }
    }

    //if the angle of attack is correct
    else if (maxLiftAngle == readAttackAngle) { }

    //to adjust tab angle according to wind side
    if (windSide) {
      servo.write(servoOffset + tabAngle);
    }
    else {
      servo.write(servoOffset - tabAngle);
    }
  }
  //-----------------------------------------------------------------------------------------------------------
  //while drag if desired
  if (drag) {

    //set sail to most possible angle of attack with respect to direction of wind
    if (windSide) {
      servo.write(servoOffset + 55);
    }
    else if (!windSide) {
      servo.write(servoOffset - 55);
    }
  }
  //----------------------------------------------------------------------------------
  //minimum lift (windvane)
  if (!lift && !drag && !control) {
    digitalWrite(led1Pin, LOW);
    digitalWrite(led2Pin, LOW);

    servo.write(servoOffset);
    /*
      if (readAttackAngle < 2 && readAttackAngle > -2) {  }            // if angle of attack is within -2 to 2 do nothing
      else if (readAttackAngle > 2 && tabAngle < 60) {       // if angle of attack is to much adjust
      tabAngle--;
      }
      else if (readAttackAngle < -2 && tabAngle > -60) { // if angle of attack is to much adjust
      tabAngle++;
      }
      servo.write(servoOffset + tabAngle);
    */
  }
}

