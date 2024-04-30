/*
Names: Ian McGrath and Daniel Langdon
Code: Final Demo (Demo 2 Redo(Drive in Circle around a beacon))
Setup:
Using the motor drive Polula Dual MC33926 Motor Driver shield, the Arduino Uno, and 
a Voltage Monitor. Connect the Motor Driver onto the Arduino Uno, connect a battery to the
voltage monitor and connect the monitor to the motor driver through the two pins. Then connect 
the motors to the two pins next to voltage pins of the motor driver and the encoder wired to an interrupt pin 2 or 3 
and digital pin 5 or 6 with the blue and green wired going to Vcc and ground pins on the motor driver. The motor sign pins 
allow voltage to flow to the motor and the Voltage pins send the voltage to allow the motor to spin. 
Code Usage:
This code uses a PID controller to allow the wheels to travel a distance set by the camera (mode 1), turn 90 degrees (mode 0), and drive in a 1ft radius circle around a beacon (mode 4). As
long as you have the correct hardware setup, you simply have to upload the code and it will do this task. Mode 2 is used in another demo to rotate until a marker is found.
The receive function are if you want to attach a PI and a camera to find the distance and angle away from an Aruco marker and use those distances to drive the robot 
to within a foot of the marker and then go in a circle

*/

#include <Wire.h>
#define MY_ADDR 8
#define PI 3.14159265358979323 // Defines irrational number PI for mathematical use later

// Essential global variables for communication with CV team and flags given to controls team to know when to stop and get distances/angles
volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;
volatile uint8_t distanceBit1 = 0;
volatile uint8_t distanceBit2 = 0;
volatile uint8_t angleBit = 0;
volatile bool detection_flag = false;
volatile bool circle_flag = false;
volatile bool go_straight_flag=false;
volatile bool go50 = false;
volatile bool first_flag = true;
volatile bool firstFlagPhi = true;
float markerAngle = 0;
float markerDistance = 0;

// These lines define the pins each motor and encoder pins go to 
const int MotorVoltage[2] = {10,9}; 
const int MotorSign[2] = {8,7};
const int encoderInterrupts[2] = {2,3};
const int encoderPins[2] = {5,6};
const int MotorEnable = 4;

// Defines desired delay time
float desired_Ts_ms = 10;
float last_time_ms;

float d = 0.3556; //m between middles of wheels
float r = 0.07; //wheel rad m

float lastEncoderTime[2]; //Initializes time variable for use in ISR
float startTime; //Set up in Setup function to calculate current time in loop
float initialTime; //Used to calculate time elapsed for velocity
float lastTime; //Used to compute current time
float currentTime; //Initializes current time for loop
float timeElapsed;

int count1 = 0; //Initializes encoder counts for the ISR
int count2 = 0; //Initializes encoder counts for the second encoder ISR
int currentEncoderCount[2] = {0,0}; //Intilization for Encoder Counts to be used in loop 
float currentEncoderCountRad[2] = {0,0}; //Converted counts to radian for velocity in rad
int initialEncoderCount[2] = {0,0}; //Found in setup to check for change in counts
float initialEncoderCountRad[2] = {0,0}; //Radian version of initial count
float vel[2] = {0,0}; //Velocity found from timeelapsed and count/radian change;
float voltage[2] = {0,0}; // voltage to be used for speed and position control
float batteryVoltage = 7.8; //Sets saturation point for battery

// Instantiates all variables for the velocity controller
float phiVel = 0;
float rhoVel = 0;
float errorRhoVel = 0;
float errorPhiVel = 0;
float Vbar = 0;
float deltaV = 0;
float desiredRhoVel = 0;
float desiredPhiVel = 0;

// Instantiates all PID gain values for position controller
float KiPhi = 0.45;
float KdPhi = 0;
float KpPhi = 45;

float KiRho = .383;
float KdRho = .2038;
float KpRho = 2.3514;

//Instantiates gain values for PID velocity controller
float KpRhoVel = 100;
float KpPhiVel = 20;

// Instantiates all necessary variables for the position controller
float Rho = 0;
float errorRho = 0;
float errorRhoInitial = 0;
float errorPhiInitial = 0;
float derivativeRho = 0;
float integralRho = 0;
float desiredPhi = 0;
float rotations = 0;
float desiredRho = 0; //m
float errorPhi = 0;
float derivativePhi = 0;
float integralPhi = 0;
float phi = 0;

// Variables for state machine and timing for certain functionality 
int mode = 1;
int lastMode = 1;
float startCircleTime;
float stutter = 0;
int count = 1;

//ISR to check for a change in state of the encoder
void myISR1() {
  if(micros() - lastEncoderTime[0] > 100) {
    if(digitalRead(encoderInterrupts[0]) == digitalRead(encoderPins[0])) {
      count1++;
      count1++;
    } else{
      count1--;
      count1--;
    }
    lastEncoderTime[0] = micros();
  }
}

//ISR to check for a change in state of the encoder
void myISR2() {
  if(micros() - lastEncoderTime[1] > 100) {
    if(digitalRead(encoderInterrupts[1]) == digitalRead(encoderPins[1])) {
      count2++;
      count2++;
    } else{
      count2--;
      count2--;
    }
    lastEncoderTime[1] = micros();
  }
}

//Function to return encoder counts
int MyEnc1() {
  if(digitalRead(encoderInterrupts[0]) != digitalRead(encoderPins[0])) {
    return(count1+1);
  } else{
    return(count1);
  }
}

//Function to return encoder counts
int MyEnc2() {
  if(digitalRead(encoderInterrupts[1]) != digitalRead(encoderPins[1])) {
    return(count2+1);
  } else{
    return(count2);
  }
}

void setup() {
  // put your setup code here, to run once:
  // For loop to assign pins as outputs/inputs and sets encoders to high for pullup resistor
  for(int i = 0; i<2; i++) {
    pinMode(MotorVoltage[i],OUTPUT);
    pinMode(MotorSign[i],OUTPUT);
    pinMode(encoderInterrupts[i],INPUT);
    digitalWrite(encoderInterrupts[i],HIGH);
  }
  attachInterrupt(digitalPinToInterrupt(encoderInterrupts[0]), myISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderInterrupts[1]), myISR2, CHANGE);
  pinMode(MotorEnable, OUTPUT);
  digitalWrite(MotorEnable,HIGH);

  // Initializes initial values for time and encoder counts/position
  initialEncoderCount[0] = MyEnc1();
  initialEncoderCount[1] = MyEnc2();
  for (int i = 0; i < 2; i++){
    initialEncoderCountRad[i] = 2*PI*(float)(initialEncoderCount[i])/3200;
    lastEncoderTime[i] = micros();
  }
  startTime = millis();
  initialTime = millis();

  Serial.begin(115200);
  // We want to control the built-in LED (pin 13)
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize I2C
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts
  Wire.onReceive(receive);


  // Variables for desired angle and distance to travel in radians and meters
  //desiredPhi = PI/2;
  // desiredRho = 0.3;
}

void loop() {
  // If there is data on the buffer, read it
  if (msgLength > 0) {
    if (offset==1) {
      digitalWrite(LED_BUILTIN,instruction[0]);
    }
    printReceived();
    msgLength = 0;
  }

  //Compute current time
  lastTime = millis();
  currentTime = (float)((lastTime-startTime)/1000);
  timeElapsed = (float)(millis()-initialTime)/1000;
  currentEncoderCount[0] = MyEnc1();
  currentEncoderCount[1] = MyEnc2();

  // Set encoders to radians and finds velocity
  for(int i = 0;i<2;i++){ 
    currentEncoderCountRad[i] = 2*PI*(float)(currentEncoderCount[i])/3200;
    if(timeElapsed > 0 ){
      vel[i] = (currentEncoderCountRad[i]-initialEncoderCountRad[i])/timeElapsed;
    }
  }
  // Calculates current angle and forward distance travelled
  Rho = (r/2)*(currentEncoderCountRad[0]+currentEncoderCountRad[1]);
  phi = (r/d) * (currentEncoderCountRad[0]-currentEncoderCountRad[1]);

  // State machine: mode 0: turns specified angle, mode 1: move forward specified forward distance
  // mode 2: Rotate slowly until marker is detected, mode 3: Turn off motors, mode 4: spin in 1 ft circle
  switch (mode) {
    case 0:
      desiredRho = 0; // We do not want forward distance when turning towards marker initially
      if (lastMode == 1 && count == 1) {
        desiredPhi = -3*PI/8; ////////////////// -1*PI/2; 
        count++;
      }
      if ((phi <= (desiredPhi + (PI/180))) && (phi >= (desiredPhi+(-1*PI/180)))) { // 2 to 1 degree
        if (lastMode == 1) {
          lastMode = 0;
          mode = 4;
          startCircleTime = currentTime; //start timer for driving in a circle
          reset();
          analogWrite(MotorVoltage[0], 0);
          analogWrite(MotorVoltage[1], 0);
          delay(200);
        }
        else if(lastMode == 2 || go_straight_flag==true){ //If previously trying to find marker, turn desiredPhi and go straight (mode 1)
          lastMode = 0;
          desiredRhoVel = 0;
          mode = 1;
        }
      }
    break;
      case 1:
        // Only set forward distance once
        if (detection_flag == true) {
          desiredRho = markerDistance-0.35; // driving approximately seven feet
        }
        // Constantly check changes in PHI to correct while driving ()
        //desiredPhi = markerAngle; //commented out for this demo since driving in circle does not use camera input
        if(abs(Rho) >= abs(desiredRho) && detection_flag == true) { 
          lastMode = 1; 
          mode = 0; //turn 90deg
          //desiredPhi = PI/2;
          reset();
          analogWrite(MotorVoltage[0], 0);
          analogWrite(MotorVoltage[1], 0);
          delay(200);

        } 
      break;
      case 2: //spin while waiting
        if (currentTime - stutter < 0.75){ // Spin for 0.75 seconds
          desiredPhiVel = -7.5;
          desiredRhoVel = 0;
        }
        else { // Head to mode three to allow processing for camera
          stutter = currentTime;
          lastMode = mode;
          desiredPhiVel = 0;
          desiredRhoVel = 0;
          mode = 3;
        }
        // If marker detected then reset counts wait and switch to mode 0 (turn)
        if (detection_flag == true){
          desiredPhiVel = 0;
          desiredRhoVel = 0;
          lastMode = 2;
          desiredPhi = markerAngle;
          mode = 0;
          count1 = 0;
          count2 = 0;
          delay(1500);
        }
      break;
      case 3: //wait
        desiredPhiVel = 0;
        desiredRhoVel = 0;
        // Waits 3.75 seconds and then goes back to mode 2
        if(lastMode == 2 && (currentTime - stutter >= 3.75)){
          lastMode = 3;
          mode = 2;
          stutter = currentTime;
        }
        // Turns off motors
      break;
      case 4: // Spin in circle
        // Sets desiredRhoVel and desiredPhiVel to use for velocity controller to go in a 1 foot circle
        desiredRhoVel = 11;
        desiredPhiVel = desiredRhoVel/(0.385*1.5);
        // Only spin for 9.35 seconds
        if (currentTime-startCircleTime >= 5.45) {
          desiredRhoVel = 0;
          desiredPhiVel = 0;
          // Turn off when done
          mode = 3;
          lastMode = 4;
        }
      break;
    }
  
  // Position controller (only for mode 0 and 1) 
  if(mode == 1 || mode == 0) {
    // Finds error, derivative term, and integral term to find desiredPhiVel
    errorPhi = desiredPhi - phi;
    derivativePhi = (errorPhi - errorPhiInitial)/((float)(desired_Ts_ms/1000));
    integralPhi = integralPhi + errorPhi*((float)(desired_Ts_ms/1000));
    desiredPhiVel = errorPhi*KpPhi + KdPhi*derivativePhi + KiPhi*integralPhi;

    // Caps max phiVel to fix speed/voltage issues
    if(abs(desiredPhiVel) > 10){
      desiredPhiVel = 10*desiredPhiVel/abs(desiredPhiVel);
    }

    // Finds error, derivative term, and integral term to find desiredRhoVel
    errorRho = desiredRho - Rho;
    derivativeRho = (errorRho - errorRhoInitial)/((float)(desired_Ts_ms/1000));
    integralRho = integralRho + errorRho*((float)(desired_Ts_ms/1000));
    desiredRhoVel = errorRho*KpRho + KdRho*derivativeRho + KiRho*integralRho;

    // Caps max rhoVel to fix speed/volt issues
    if(abs(desiredRhoVel) > 10){
        desiredRhoVel = 10*desiredRhoVel/abs(desiredRhoVel);
    }
  }
  //Calculates current angular and forward velocities
  phiVel = (r/d)*(vel[0]-vel[1]);
  rhoVel = (r/2)*(vel[0]+vel[1]);

  // Finds error between desiredVel and actual Vel
  errorRhoVel = rhoVel-desiredRhoVel;
  errorPhiVel = phiVel-desiredPhiVel;

  // Uses error Values and gain to find Vbar and deltaV which is used to calculate desired voltages for each motor
  Vbar = errorRhoVel*KpRhoVel;
  deltaV = errorPhiVel*KpPhiVel;
  voltage[0] = (Vbar+deltaV)/2;
  voltage[1] = (Vbar - deltaV)/2;

  Serial.print(mode);
  Serial.print("\t");
  Serial.println(Rho);
    Serial.print("\t");
  Serial.println(Rho);
  //Serial.prin

  if (mode != 3) { //Only apply voltage if mode is not 3
    for(int i = 0; i < 2; i++) {
      if(voltage[i] >= 0) {
        digitalWrite(MotorSign[i],HIGH);
        analogWrite(MotorVoltage[i],abs(voltage[i]));
      } else {
          digitalWrite(MotorSign[i],LOW);
          analogWrite(MotorVoltage[i],abs(voltage[i]));
        }
    }
  } else if (mode == 3) {
    analogWrite(MotorVoltage[0],0);
    analogWrite(MotorVoltage[1],0);
  }

  //Sets old values to new values
  errorRhoInitial = errorRho;
  errorPhiInitial = errorPhi;
  for(int i = 0;i<2;i++){ 
    initialEncoderCount[i] = currentEncoderCount[i];
    initialEncoderCountRad[i] = currentEncoderCountRad[i];
  }
  initialTime = millis();

  while(millis()<last_time_ms+desired_Ts_ms){
    //wait till desired time passes
  }
  last_time_ms = millis();
}

// printReceived helps us see what data we are getting from the leader
void printReceived() {
  detection_flag = true;
  //Serial.print("Message: ");
  for (int i=0;i<msgLength;i++) {
    //Serial.print(String((char) instruction[i]));
  }
  //Serial.println(""); 

  // Pulling info from our Pi encoding
  distanceBit1 =  instruction[0] - 48;
  distanceBit2 =  instruction[1] - 48;
  angleBit = instruction[2] - 48;

  // Serial.print("Distance bit: ");
  // Serial.print((distanceBit1 * 10) + distanceBit2);
  // Serial.println("");

  // Serial.print("Angle bit: ");
  // Serial.print(angleBit);
  // Serial.println("");

  // Decoding the bits to an average distance / angle
  // DISTANCE
  if (mode == 0) {
    desiredRho = 0;
  }
  markerDistance = ((distanceBit1 * 10) + distanceBit2) * 0.025;

  // Serial.print("Distance: ");
  // Serial.print(markerDistance);
  // Serial.println("");
  // ANGLE
  if (circle_flag == true) {
    // do nothing
  }
  else if (angleBit == 1) {
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
    mode = 1;
    go_straight_flag=true;
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
  // Serial.print("Angle: ");
  // Serial.print(markerAngle);
  // Serial.println("");
}


// function called when an I2C interrupt event happens
void receive() {
  // Set the offset, this will always be the first byte.
  offset = Wire.read();
  // If there is information after the offset, it is telling us more about the command.
  while (Wire.available()) {
    instruction[msgLength] = Wire.read();
    msgLength++;
    
  }
}

// used to set phi, rho, phiVel, and rhoVel to 0
void reset() {
  desiredRhoVel = 0;
  desiredPhiVel = 0;
  errorRhoVel = 0;
  errorPhiVel = 0;
  count1 = 0;
  count2 = 0;
  desiredRho = 0;
  desiredPhi = 0;
  voltage[0] = 0;
  voltage[1] = 0;
}



