/*
Names: Ian McGrath and Daniel Langdon
Code: Demo 1
Setup:
Using the motor drive Polula Dual MC33926 Motor Driver shield, the Arduino Uno, and 
a Voltage Monitor. Connect the Motor Driver onto the Arduino Uno, connect a battery to the
voltage monitor and connect the monitor to the motor driver through the two pins. Then connect 
the motors to the two pins next to voltage pins of the motor driver and the encoder wired to an interrupt pin 2 or 3 
and digital pin 5 or 6 with the blue and green wired going to Vcc and ground pins on the motor driver. The motor sign pins 
allow voltage to flow to the motor and the Voltage pins send the voltage to allow the motor to spin. 
Code Usage:
This code uses a PI controller to allow the wheels to travel a certain angle and distance and stop. You set these values in the setup called
desiredPhi and desiredDis in radians and meters respectively and if you connect the arduino and upload the code it will turn that angle and drive that distance 
*/

#include <Wire.h>
#define MY_ADDR 8
#define PI 3.14159265358979323  // Defines irrational number PI for mathematical use later

volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = { 0 };
volatile uint8_t msgLength = 0;
volatile uint8_t distanceBit1 = 0;
volatile uint8_t distanceBit2 = 0;
volatile uint8_t angleBit = 0;
volatile bool detection_flag = false;
volatile bool circle_flag = false;
volatile bool go_straight_flag = false;
volatile bool first_flag = true;
volatile bool firstFlagPhi = true;

// These lines define the pins each motor and encoder pins go to
const int MotorVoltage[2] = { 10, 9 };
const int MotorSign[2] = { 8, 7 };
const int encoderInterrupts[2] = { 2, 3 };
const int encoderPins[2] = { 5, 6 };
const int MotorEnable = 4;

// Defines desired delay time
float desired_Ts_ms = 10;
float last_time_ms;

float d = 0.3556;  //m between middles of wheels
float r = 0.07;    //wheel rad m

float lastEncoderTime[2];  //Initializes time variable for use in ISR
float startTime;           //Set up in Setup function to calculate current time in loop
float initialTime;         //Used to calculate time elapsed for velocity
float lastTime;            //Used to compute current time
float currentTime;         //Initializes current time for loop
float timeElapsed;


int count1 = 0;                              //Initializes encoder counts for the ISR
int count2 = 0;                              //Initializes encoder counts for the second encoder ISR
int currentEncoderCount[2] = { 0, 0 };       //Intilization for Encoder Counts to be used in loop
float currentEncoderCountRad[2] = { 0, 0 };  //Converted counts to radian for velocity in rad
int initialEncoderCount[2] = { 0, 0 };       //Found in setup to check for change in counts
float initialEncoderCountRad[2] = { 0, 0 };  //Radian version of initial count
float vel[2] = { 0, 0 };                     //Velocity found from timeelapsed and count/radian change;
float voltage[2] = { 0, 0 };                 // voltage to be used for speed and position control
float batteryVoltage = 7.8;                  //Sets saturation point for battery

// Instantiates all variables for the velocity controller
float phiVel = 0;
float rhoVel = 0;
float errorRhoVel = 0;
float errorPhiVel = 0;
float derivativePhiVel = 0;
float derivativeRhoVel = 0;
float Vbar = 0;
float deltaV = 0;
float errorRhoVelInitial = 0;
float errorPhiVelInitial = 0;
float integralRhoVel = 0;
float integralPhiVel = 0;
float desiredRhoVel = 0;
float desiredPhiVel = 0;

// Instantiates all variables for position controller
float KiPhi = .45;
float KdPhi = 0;
float KpPhi = 45;

float KiRho = .383;
float KdRho = .2038;
float KpRho = 2.3514;
//Instantiates gain values for PID velocity controller

float KpRhoVel = 100;
float KpPhiVel = 20;
// float desiredRho = 50;
float Rho = 0;
float errorRho = 0;
float errorRhoInitial = 0;
float errorPhiInitial = 0;
float derivativeRho = 0;
float integralRho = 0;
float desiredPhi = 0;
float rotations = 0;
float desiredRho = 0;  //m
float errorPhi = 0;
float derivativePhi = 0;
float integralPhi = 0;
float phi = 0;
int mode = 2;
int lastMode = 2;
float startCircleTime;
float stutter = 0;
float markerAngle;
float markerDistance;



//ISR to check for a change in state of the encoder
void myISR1() {
  if (micros() - lastEncoderTime[0] > 100) {
    if (digitalRead(encoderInterrupts[0]) == digitalRead(encoderPins[0])) {
      count1++;
      count1++;
    } else {
      count1--;
      count1--;
    }
    lastEncoderTime[0] = micros();
  }
}

//ISR to check for a change in state of the encoder
void myISR2() {
  if (micros() - lastEncoderTime[1] > 100) {
    if (digitalRead(encoderInterrupts[1]) == digitalRead(encoderPins[1])) {
      count2++;
      count2++;
    } else {
      count2--;
      count2--;
    }
    lastEncoderTime[1] = micros();
  }
}

//Function to return encoder counts
int MyEnc1() {
  if (digitalRead(encoderInterrupts[0]) != digitalRead(encoderPins[0])) {
    return (count1 + 1);
  } else {
    return (count1);
  }
}

//Function to return encoder counts
int MyEnc2() {
  if (digitalRead(encoderInterrupts[1]) != digitalRead(encoderPins[1])) {
    return (count2 + 1);
  } else {
    return (count2);
  }
}

void setup() {
  // put your setup code here, to run once:
  // For loop to assign pins as outputs/inputs and sets encoders to high for pullup resistor
  for (int i = 0; i < 2; i++) {
    pinMode(MotorVoltage[i], OUTPUT);
    pinMode(MotorSign[i], OUTPUT);
    pinMode(encoderInterrupts[i], INPUT);
    digitalWrite(encoderInterrupts[i], HIGH);
  }
  attachInterrupt(digitalPinToInterrupt(encoderInterrupts[0]), myISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderInterrupts[1]), myISR2, CHANGE);
  pinMode(MotorEnable, OUTPUT);
  digitalWrite(MotorEnable, HIGH);

  initialEncoderCount[0] = MyEnc1();
  initialEncoderCount[1] = MyEnc2();
  for (int i = 0; i < 2; i++) {
    initialEncoderCountRad[i] = 2 * PI * (float)(initialEncoderCount[i]) / 3200;
    lastEncoderTime[i] = micros();
  }

  Serial.begin(115200);
  // We want to control the built-in LED (pin 13)
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize I2C
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts
  Wire.onReceive(receive);

  startTime = millis();
  initialTime = millis();

  // Variables for desired angle and distance to travel in radians and meters
  //desiredPhi = PI/2;
  // desiredRho = 0.3;
}

void loop() {

  // If there is data on the buffer, read it
  if (msgLength > 0) {
    if (offset == 1) {
      digitalWrite(LED_BUILTIN, instruction[0]);
    }
    printReceived();
    msgLength = 0;
  }

  lastTime = millis();
  //Compute current time
  currentTime = (float)((lastTime - startTime) / 1000);
  timeElapsed = (float)(millis() - initialTime) / 1000;
  currentEncoderCount[0] = MyEnc1();
  currentEncoderCount[1] = MyEnc2();


  for (int i = 0; i < 2; i++) {  // set encoders to radians and finds velocity
    currentEncoderCountRad[i] = 2 * PI * (float)(currentEncoderCount[i]) / 3200;
    if (timeElapsed > 0) {
      vel[i] = (currentEncoderCountRad[i] - initialEncoderCountRad[i]) / timeElapsed;
    }
  }
  Rho = (r / 2) * (currentEncoderCountRad[0] + currentEncoderCountRad[1]);
  phi = (r / d) * (currentEncoderCountRad[0] - currentEncoderCountRad[1]);



  switch (mode) {
    case 0:  // case to turn a desired angle phi
      desiredRho = 0;
      desiredRhoVel = 0;  ////might cause issues
      if (phi <= (desiredPhi + (3.5 * PI / 180)) && phi >= (desiredPhi + (-3.5 * PI / 180))) {  // if atobot is correctly oriented towards marker within 3 deg
        // Serial.print("reached desired phi");
        if (lastMode == 2 || go_straight_flag == true) {  // go straight
          resetVelController();
          lastMode = 0;
          mode = 1; //////see line 252
          if ((markerAngle <= -15*PI/180) ||(markerAngle >= 15*PI/180)){ ///set variable marker distances to account for fish-eyeing on camera lens
            desiredRho = markerDistance - .05;
          }
          if ((markerAngle <= -10*PI/180) ||(markerAngle >= 10*PI/180)){ ///set variable marker distances to account for fish-eyeing on camera lens
            desiredRho = markerDistance - .085;
          }
          if ((markerAngle <= -5*PI/180) ||(markerAngle >= 5*PI/180)){
            desiredRho = markerDistance - .23;
            // desiredRhoVel += 5;  /////////////////////////////new            analogWrite; 

          }
          if ((markerAngle <= -2*PI/180) || (markerAngle >= 2*PI/180)){
            desiredRho = markerDistance - .3;
            // desiredRhoVel += 5;  /////////////////////////////new
          }
          else{
            desiredRho = markerDistance - .1;


          }
         }
      }
      break;
    case 1:
      if (abs(Rho) >= abs(desiredRho)) {  //reached marker
        // Serial.print("reached desired Rho");
        lastMode = mode;
        mode = 3;  //wait to get graded
      }
      break;
    case 2:  //spin while waiting

      desiredRhoVel = 0;
      //desiredPhiVel = 7.5;
      if ((currentTime - stutter < .77) && (detection_flag != true) ) { ////burst length
        if(detection_flag == false){ 
          desiredPhiVel = -8.5;
          desiredRhoVel = 0;
        }
      }
       else {
        stutter = currentTime;
        lastMode = mode;
        // resetVelController();
        mode = 3;
      }
      if (detection_flag == true) {
        Serial.print("Found marker");
        resetVelController();
        lastMode = 2;
        mode = 0;
        desiredPhi = markerAngle;

        delay(200);
      }
      
      break;
    case 3:  //wait
      Serial.print("waiting...");
      // resetVelController();///////redacted
      desiredPhiVel = 0;
      desiredRhoVel = 0;
      desiredPhi = 0;
      desiredRho = 0;
      if (detection_flag == true && lastMode == 2) {
        Serial.print("Found marker");
        resetVelController();
        lastMode = 2;
        mode = 0;
        desiredPhi = markerAngle;

        delay(200);
      }

      if (lastMode == 2 && (currentTime - stutter >= 2.7) && (detection_flag != true)) { //delay time between bursts
        lastMode = 3;
        mode = 2;
        stutter = currentTime;
      }
      // else{ ///NEW
      //   errorRho = 0;
      //   errorPhi = 0;
      //   errorPhiVel = 0;
      //   errorRhoVel = 0;
      // }
      analogWrite(MotorVoltage[0], 0);
      analogWrite(MotorVoltage[1], 0);
      break;
  }

  //Serial.println(mode);
  if (mode == 1 || mode == 0) {                                                      // if we need a velocity controller,
    errorPhi = desiredPhi - phi;                                                     //error phi
    derivativePhi = (errorPhi - errorPhiInitial) / ((float)(desired_Ts_ms / 1000));  //D controller
    integralPhi = integralPhi + errorPhi * ((float)(desired_Ts_ms / 1000));          //I controller
    desiredPhiVel = errorPhi * KpPhi + KdPhi * derivativePhi + KiPhi * integralPhi;  //PID controller
    if (abs(desiredPhiVel) > 12) {
      desiredPhiVel = 12 * desiredPhiVel / abs(desiredPhiVel);
    }

    errorRho = desiredRho - Rho;
    derivativeRho = (errorRho - errorRhoInitial) / ((float)(desired_Ts_ms / 1000));
    integralRho = integralRho + errorRho * ((float)(desired_Ts_ms / 1000));
    desiredRhoVel = errorRho * KpRho + KdRho * derivativeRho + KiRho * integralRho;

    if (abs(desiredRhoVel) > 12) {
      desiredRhoVel = 12 * desiredRhoVel / abs(desiredRhoVel);
    }
  }
  phiVel = (r / d) * (vel[0] - vel[1]);
  rhoVel = (r / 2) * (vel[0] + vel[1]);

  errorRhoVel = rhoVel - desiredRhoVel;
  errorPhiVel = phiVel - desiredPhiVel;


  Serial.print("\t mode ");
  Serial.print(mode);
  Serial.print("\t");
  Serial.print(desiredPhi);
  Serial.print("\t");
  Serial.print(phi);
  Serial.print("\t");
  Serial.print(desiredRho);
  Serial.print("\t");
  Serial.println(Rho);
  // Serial.println(circle_flag);

  Vbar = errorRhoVel * KpRhoVel;
  deltaV = errorPhiVel * KpPhiVel;

  voltage[0] = (Vbar + deltaV) / 2;
  voltage[1] = (Vbar - deltaV) / 2;
  if (mode != 3) {
    for (int i = 0; i < 2; i++) {  // for each wheel
      if (voltage[i] >= 0) {       //turn correct direction
        digitalWrite(MotorSign[i], HIGH);
        analogWrite(MotorVoltage[i], abs(voltage[i]));  //apply voltage to motors
      } else {
        digitalWrite(MotorSign[i], LOW);
        analogWrite(MotorVoltage[i], abs(voltage[i]));  // apply motor voltage
      }
    }
  } else if (mode == 3) {  //mode 3 does nothing
    analogWrite(MotorVoltage[0], 0);
    analogWrite(MotorVoltage[1], 0);
  }


  //Sets old values to new values
  // errorRhoVelInitial = errorRhoVel;
  // errorPhiVelInitial = errorPhiVel;
  errorRhoInitial = errorRho;
  errorPhiInitial = errorPhi;
  for (int i = 0; i < 2; i++) {
    initialEncoderCount[i] = currentEncoderCount[i];
    initialEncoderCountRad[i] = currentEncoderCountRad[i];
  }
  initialTime = millis();
  while (millis() < last_time_ms + desired_Ts_ms) {
    //wait till desired time passes
  }
  last_time_ms = millis();
}

// printReceived helps us see what data we are getting from the leader
void printReceived() {
  detection_flag = true;
  Serial.print("Message: ");
  for (int i = 0; i < msgLength; i++) {
    Serial.print(String((char)instruction[i]));
  }
  Serial.println("");

  // Pulling info from our Pi encoding
  distanceBit1 = instruction[0] - 48;
  distanceBit2 = instruction[1] - 48;
  angleBit = instruction[2] - 48;

  Serial.print("Distance bit: ");
  Serial.print((distanceBit1 * 10) + distanceBit2);
  Serial.println("");

  Serial.print("Angle bit: ");
  Serial.print(angleBit);
  Serial.println("");

  // Decoding the bits to an average distance / angle
  // DISTANCE
  if (mode == 0) {
    desiredRho = 0;
  }
  /*
  else if (distanceBit == 1) {
    markerDistance = 3.5;
  } else if (distanceBit == 2) {
    markerDistance = 3;
  } else if (distanceBit == 3) {
    markerDistance = 2.5;
  } else if (distanceBit == 4) {
    markerDistance = 2;
  } else if (distanceBit == 5) {
    markerDistance = 1.5;
  } else if (distanceBit == 6) {
    markerDistance = 1;
  } else if (distanceBit == 7) {
    markerDistance = 0.75;
  } else if (distanceBit == 8) {
    markerDistance = 0.55;
  } else if (distanceBit == 9) {
    markerDistance = 0.5;
    go50 = true;
  }
  */

  markerDistance = ((distanceBit1 * 10) + distanceBit2) * 0.025;

  Serial.print("Distance: ");
  Serial.print(markerDistance);
  Serial.println("");

  // ANGLE
  if (circle_flag == true) {
    // do nothing
  } else if (angleBit == 1) {
    markerAngle = -26.25;
    markerAngle = markerAngle * 0.0174533;

  } else if (angleBit == 2) {
    markerAngle = -18.75;
    markerAngle = markerAngle * 0.0174533;

  } else if (angleBit == 3) {
    markerAngle = -11.25;
    markerAngle = markerAngle * 0.0174533;

  } else if (angleBit == 4) {
    markerAngle = -5.25;
    markerAngle = markerAngle * 0.0174533;

  } else if (angleBit == 5) {
    markerAngle = 0;
    markerAngle = markerAngle * 0.0174533;
    go_straight_flag = true;
    //mode = 1;

  } else if (angleBit == 6) {
    markerAngle = 5.25;
    markerAngle = markerAngle * 0.0174533;

  } else if (angleBit == 7) {
    markerAngle = 11.25;
    markerAngle = markerAngle * 0.0174533;

  } else if (angleBit == 8) {
    markerAngle = 18.75;
    markerAngle = markerAngle * 0.0174533;

  } else if (angleBit == 9) {
    markerAngle = 26.25;
    markerAngle = markerAngle * 0.0174533;
  }


  Serial.print("Angle: ");
  Serial.print(markerAngle);
  Serial.println("");
}


// function called when an I2C interrupt event happens
void receive(){
  // Set the offset, this will always be the first byte.
  offset = Wire.read();
  // If there is information after the offset, it is telling us more about the command.
  while (Wire.available()) {
    instruction[msgLength] = Wire.read();
    msgLength++;
  }
}
void resetVelController(){
  phi = 0;
  Rho = 0;
  errorRho = 0;
  errorPhi = 0;
  errorPhiVel = 0;
  errorRhoVel = 0;
  count1 = 0;
  count2 = 0;
  desiredPhi = 0;
  desiredRho = 0;
}
