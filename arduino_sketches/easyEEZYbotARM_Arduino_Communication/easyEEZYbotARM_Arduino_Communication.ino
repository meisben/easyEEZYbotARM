
/*
   ~~~~~Prerequisites~~~~~
   ServoEasing.h (https://github.com/ArminJo/ServoEasing) -> Just a fantastic library! wow :)

   ~~~~~Details~~~~~~~~~~~~
   Program title: EEZYbotARM control
   Author: Ben Money-Coomes (ben.money@gmail.com)
   Date: 20/11/19
   Purpose: Communicate with Python program for EEZYbotARM, move the arm

   IMPORTANT: This program receives data over the serial monitor. This data must be in the form of:
   "<HelloWorld,10.0,90.7,81.5,80.0,1000,2000,1000,1000>"
   Where this is the equivalent of <string, float, float, float, float, int, int, int, int>
   This is interpreted as <instruction, jointAngleEE (degrees), jointAngle1, jointAngle2, jointAngle3, movementTimeEE (milli seconds), movementTime1, movementTime2, movementTime3>

   ~~~~~Licence~~~~~~~~~~~~
   MIT License

   ~~~~~Read me~~~~~~~~~~~~
   Can be found at https://github.com/meisben/easyEEZYbotARM

   ~~~~~Version Control~~~
   v1.00 - exactly the same implementation as Robin2 code on Arduino forums
   v2.00 - no change, version control added. Works with same version of python program
   v3.00 - Beginning to take structure from 'servo easing' program -> BUT ONLY FOR COMMS
   v3.10 - Now including servo motor and buzzer control elements -> All servos working ! :)
   v3.20 - Changing order of angles so that end effector becomes pin0
*/

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  Library Includes.                                                              *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// include these libraries for using the servo add on board. Taken from servo example code
#include <Arduino.h>
// include this library for servo easing functionality
#include "ServoEasing.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  Definitions                                                                    *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define VERSION "3.1"
#define BUZZER_PIN 5 //buzzer to featherwing pin 5, 470 ohm resistor
#define ACTION_TIME_PERIOD 1000
const int SERVO1_PIN = 1; //servo pin for joint 1
const int SERVO2_PIN = 2; //servo pin for joint 2
const int SERVO3_PIN = 3; //servo pin for joint 3
const int SERVO0_PIN = 0; //servo pin for end effector

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  Variables                                                                      *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//-------- Variables for receiving serial data -------------
const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;
char messageFromPC[buffSize] = {0};

// -------- Variables to hold time -------------
unsigned long curMillis; // Variable for current time
//unsigned long general_timer;

// -------- Variables to hold the parsed data -------------
float floatFromPC0 = 90.0; // initial values are mid range for joint angles
float floatFromPC1 = 90.0;
float floatFromPC2 = 90.0;
float floatFromPC3 = 90.0;
int intFromPC0 = 1000; // inital values are acceptable movement times
int intFromPC1 = 1000;
int intFromPC2 = 1000;
int intFromPC3 = 1000;
float last_servoAngle_q1 = floatFromPC1; // initial values are mid range for joint angles
float last_servoAngle_q2 = floatFromPC2;
float last_servoAngle_q3 = intFromPC3;
float last_servoAngle_EE = floatFromPC0;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  Instatiate clasess for libraries                                               *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
ServoEasing Servo1(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing Servo2(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing Servo3(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing Servo0(PCA9685_DEFAULT_ADDRESS, &Wire);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  START OF PROGRAM (Setup)                                                       *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void setup() {
  //flash LED so we know we are alive
  blinkLED();

  //Setup pins
  pinMode(LED_BUILTIN, OUTPUT); // setup built in LED for flashing
  pinMode(BUZZER_PIN, OUTPUT); // Set buzzer pin as an output

  //Play tone so we can tell if the board accidently reset
  playTone();

  //Begin serial communications
  Serial.begin(9600);

  //Wait for serial communications to start before continuing
  while (!Serial); //delay for Leonardo

  // Just to know which program is running on my Arduino
  Serial.println(F("START " __FILE__ "\r\nVersion " VERSION " from " __DATE__));

  //Attach servo to pin
  Servo1.attach(SERVO1_PIN);
  Servo2.attach(SERVO2_PIN);
  Servo3.attach(SERVO3_PIN);
  Servo0.attach(SERVO0_PIN);

  // Set servo to start position.
  Servo1.setEasingType(EASE_CUBIC_IN_OUT);
  Servo2.setEasingType(EASE_CUBIC_IN_OUT);
  Servo3.setEasingType(EASE_CUBIC_IN_OUT);
  Servo0.setEasingType(EASE_CUBIC_IN_OUT); // end effector
  
  Servo1.write(last_servoAngle_q1);
  Servo2.write(last_servoAngle_q2);
  Servo3.write(last_servoAngle_q3);
  Servo0.write(last_servoAngle_EE); // end effector

  // Just wait for servos to reach position
  delay(500); // delay() is OK in setup as it only happens once

  // tell the PC we are ready
  Serial.println("<Arduino is ready>");
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  MAIN PROGRAM (Loop)                                                            *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void loop() {
  // This part of the loop for the serial communication is not inside a timer -> it happens very quickly
  curMillis = millis(); // get current time
  getDataFromPC(); // receive data from PC and save it into inputBuffer

  // need if statement -> flag to say if new data is available
  if (newDataFromPC == true)
  {
    processMessageFromPC(); // Processes text message from PC
    actionInstructionsFromPC(); // Arrange for things to move, beep, light up
    replyToPC(); // Reply to PC
  }

  //  // This part of the loop is inside a timer -> maybe delete
  //  unsigned long elapsed_time_general_timer = millis() - general_timer;
  //
  //  if ( elapsed_time_general_timer > ACTION_TIME_PERIOD ) // set a target for the Romi periodically
  //  {
  //    // update timestamp
  //    general_timer = millis();
  //  }


}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
              FUNCTIONS FOR MAKING THINGS MOVE OR LIGHT UP OR BEEP!              *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//~~~~~~~~~~~~~Fuction: Blink LED~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void blinkLED() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~Fuction: Play Buzzer Tone~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void playTone() {
  tone(BUZZER_PIN, 650, 300);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~Fuction: Action the instructions from the PC~~~~~~~~~~~~~~~~~~~~~~~

void actionInstructionsFromPC() {
  //Local variables
  // -- joint angles
  float servoAngle_q1 = floatFromPC1;
  float servoAngle_q2 = floatFromPC2;
  float servoAngle_q3 = floatFromPC3;
  float servoAngle_EE = floatFromPC0;
  // -- joint speeds
  int servoTime_q1 = intFromPC1;
  int servoTime_q2 = intFromPC2;
  int servoTime_q3 = intFromPC3;
  int servoTime_EE = intFromPC0;

  // Check if the joint angle has changed!
  if (servoAngle_q1 != last_servoAngle_q1) {
    Serial.println(F("Servo 1 moving to position using interrupts"));

    Servo1.startEaseToD(servoAngle_q1, servoTime_q1);

    //    while (Servo3.isMovingAndCallYield()) {
    //      ; // no delays here to avoid break between forth and back movement
    //    }
  }

  if (servoAngle_q2 != last_servoAngle_q2) {
    Serial.println(F("Servo 2 moving to position using interrupts"));

    Servo2.startEaseToD(servoAngle_q2, servoTime_q2);

    //    while (Servo3.isMovingAndCallYield()) {
    //      ; // no delays here to avoid break between forth and back movement
    //    }
  }


  if (servoAngle_q3 != last_servoAngle_q3) {
    Serial.println(F("Servo 3 moving to position using interrupts"));

    Servo3.startEaseToD(servoAngle_q3, servoTime_q3);

    //    while (Servo3.isMovingAndCallYield()) {
    //      ; // no delays here to avoid break between forth and back movement
    //    }
  }

  if (servoAngle_EE != last_servoAngle_EE) {
    Serial.println(F("Servo EE moving to position using interrupts"));

    Servo0.startEaseToD(servoAngle_EE, servoTime_EE);

    //    while (Servo3.isMovingAndCallYield()) {
    //      ; // no delays here to avoid break between forth and back movement
    //    }
  }

  // Store current joint angle
  last_servoAngle_q1 = servoAngle_q1;
  last_servoAngle_q2 = servoAngle_q2;
  last_servoAngle_q3 = servoAngle_q3;
  last_servoAngle_EE = servoAngle_EE;

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
              FUNCTIONS FOR RECEIVING DATA VIA SERIAL MONITOR                    *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//~~~~~~~~~~~~~Fuction: Receive data with start and end markers~~~~~~~~~~~~~~~~~~

void getDataFromPC() {

  // This function receives data from PC and saves it into inputBuffer

  if (Serial.available() > 0 && newDataFromPC == false) {

    char x = Serial.read();

    // the order of these IF clauses is significant

    if (x == endMarker) {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;
      parseData();
    }

    if (readInProgress) {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1;
      }
    }

    if (x == startMarker) {
      bytesRecvd = 0;
      readInProgress = true;
    }
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~Fuction: Split data into known component parts~~~~~~~~~~~~~~~~~~

void parseData() {

  // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(inputBuffer, ",");     // get the first part - the string
  strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  floatFromPC0 = atof(strtokIndx);     // convert this part to a float -> to convert to an integer we would use atoi

  strtokIndx = strtok(NULL, ",");
  floatFromPC1 = atof(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ",");
  floatFromPC2 = atof(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ",");
  floatFromPC3 = atof(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ",");
  intFromPC0 = atoi(strtokIndx);     // convert this part to a int

  strtokIndx = strtok(NULL, ",");
  intFromPC1 = atoi(strtokIndx);     // convert this part to a int

  strtokIndx = strtok(NULL, ",");
  intFromPC2 = atoi(strtokIndx);     // convert this part to a int

  strtokIndx = strtok(NULL, ",");
  intFromPC3 = atoi(strtokIndx);     // convert this part to a int

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~Fuction: Send message back to PC~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


void replyToPC() {

  if (newDataFromPC) {
    newDataFromPC = false;
    Serial.print(F("<Msg "));
    Serial.print(messageFromPC);
    Serial.print(F(" floatFromPC0 "));
    Serial.print(floatFromPC0);
    Serial.print(F(" floatFromPC1 "));
    Serial.print(floatFromPC1);
    Serial.print(F(" floatFromPC2 "));
    Serial.print(floatFromPC2);
    Serial.print(F(" floatFromPC3 "));
    Serial.print(floatFromPC3);
    Serial.print(F(" intFromPC0 "));
    Serial.print(intFromPC0);
    Serial.print(F(" intFromPC1 "));
    Serial.print(intFromPC1);
    Serial.print(F(" intFromPC2 "));
    Serial.print(intFromPC2);
    Serial.print(F(" intFromPC3 "));
    Serial.print(intFromPC3);
    Serial.print(F(" Time "));
    Serial.print(curMillis / 1000); // divide by 512 is approx = half-seconds
    Serial.println(F(">"));
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~Fuction: Process the string message from the PC~~~~~~~~~~~~~~~~~~~~


void processMessageFromPC() {

  // this illustrates using different inputs to call different functions
  // strcmp compares two strings and returns zero if the strings are equal

  if (strcmp(messageFromPC, "LED") == 0) {
    blinkLED();
  }

  if (strcmp(messageFromPC, "BUZZ") == 0) {
    playTone();
  }

  if (strcmp(messageFromPC, "BUZZ") == 0) {
    playTone();
  }


}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
