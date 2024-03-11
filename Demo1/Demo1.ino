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
#define PI 3.14159265358979323 // Defines irrational number PI for mathematical use later

// These lines define the pins each motor and encoder pins go to 
const int MotorVoltage[2] = {10,9}; 
const int MotorSign[2] = {8,7};
const int encoderInterrupts[2] = {2,3};
const int encoderPins[2] = {5,6};
const int MotorEnable = 4;

// Defines desired delay time
unsigned long desired_Ts_ms = 10;
unsigned long last_time_ms;

float lastEncoderTime[2]; //Initializes time variable for use in ISR
float startTime; //Set up in Setup function to calculate current time in loop
float initialTime; //Used to calculate time elapsed for velocity
float lastTime; //Used to compute current time
float currentTime; //Initializes current time for loop
float timeElapsed;

int count1 = 0; //Initializes encoder counts for the ISR
int count2 = 0; //Initializes encoder counts for the second encoder ISR
int currentEncoderCount[2] = {0,0}; //Intilization for Encoder Counts to be used in loop 
float currentEncoderCountRad[2] = {0,0}; //Converted counts to radian for velocity in rad/s
int initialEncoderCount[2] = {0,0}; //Found in setup to check for change in counts
float initialEncoderCountRad[2] = {0,0}; //Radian version of initial count
float vel[2]; //Velocity found from timeelapsed and count/radian change
float desiredVel[2]={0,0}; //Desired velocity to achieve in radian/second
float Kp_pos = 7.85; //Controller gain for proportional 
float Ki_pos = 1.25; //Controller gain for integrator
float Kp = 2.5;
float voltage[2] = {0,0}; // voltage to be used for speed and position control
float batteryVoltage = 7.8; //Sets saturation point for battery
//Defines error values for position and integral
float pos_error[2]= {0,0}; 
float desiredPos[2]= {0,0};
// float actualPos[2];
float integralError[2] = {0,0};
float error[2] = {0,0};

///////position variables
float b = 0.3556; //m between middles of wheels
float r = 0.07; //wheel rad m
int mode = 0; ///0 for turn 1 for move
int count = 0;

float distance[2] = {0,0};

float phiNew = 0;
float phiOld = 0;
float desiredPhi = 0;
float phiError = 0;


int PWM[2] = {0,0}; //PWM variable to be used for later

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

  initialEncoderCount[0] = MyEnc1();
  initialEncoderCount[1] = MyEnc2();
  for (int i = 0; i < 2; i++){
    initialEncoderCountRad[i] = 2*PI*(float)(initialEncoderCount[i])/3200;
    lastEncoderTime[i] = micros();
  }

  Serial.begin(115200);
  // We want to control the built-in LED (pin 13)
  pinMode(LED_BUILTIN, OUTPUT);

  startTime = millis();
  initialTime = millis();

  // Variables for desired angle and distance to travel in radians and meters
  desiredPhi = 5*PI/4-4*PI/180;
  desiredDis = 0.85; //M

}

void loop() {
  // put your main code here, to run repeatedly:
  lastTime = millis();
  //Compute current time
  currentTime = (float)((lastTime-startTime)/1000);
  timeElapsed = (float)(millis()-initialTime)/1000;
  currentEncoderCount[0] = MyEnc1();
  currentEncoderCount[1] = MyEnc2();

  for(int i = 0;i<2;i++){ // set encoders to radians and finds velocity
    currentEncoderCountRad[i] = 2*PI*(float)(currentEncoderCount[i])/3200;
    vel[i] = (currentEncoderCountRad[i]-initialEncoderCountRad[i])/timeElapsed;
    distance[i] = currentEncoderCountRad[i]*r;
  }
  // Finds current angle and angular velocity
  phiNew = (((initialEncoderCountRad[0]*r - distance[1]) + (initialEncoderCountRad[1]*r-distance[1]))/b);
  phiVel = (((vel[0]*r) - (vel[1]*r))/b);

  //Sets old values to new values
  for(int i = 0;i<2;i++){ 
    initialEncoderCount[i] = currentEncoderCount[i];
    initialEncoderCountRad[i] = currentEncoderCountRad[i];
  }
  initialTime = millis();
  // Switch mode to separate turning and moving forward code. Turn is mode 0 and then switched to mode 1 to go straight
  switch (mode){
    case 0:
      if (desiredPhi > 0){ ///calculate desired anglular position for each wheel based off of phi
        desiredPos[0] = desiredPhi*(b/(2*r));
        desiredPos[1] = -1*desiredPhi*(b/(2*r));
      } else if (desiredPhi < 0){
        desiredPos[0] = -1*desiredPhi*(b/(2*r));  
        desiredPos[1] = desiredPhi*(b/(2*r));     
      }
      //Checks if within angular range and changes mode 
      if(phiNew <= desiredPhi + PI/180 && phiNew >= desiredPhi - PI/180){ 
        mode = 1;
        delay(1000);
        PWM[0] = 0;
        PWM[1] = 0;
        count1 = 0;
        count2 = 0;
      }
    break;

    case 1:
    // Finds distance each wheel needs to travel to reach desired distance
      desiredPos[0] = desiredDis/(r);
      desiredPos[1] = desiredDis/(r);
      desiredPhi = 0;
      
      //Checks if position is reached and turns off motors. 
      if ((currentEncoderCountRad[0]*r <= desiredDis+0.0254 && currentEncoderCountRad[0]*r >= desiredDis - 0.0254) && (currentEncoderCountRad[1]*r <= desiredDis+0.0254 && currentEncoderCountRad[1]*r >= desiredDis - 0.0254) ) {
        PWM[0] = 0;
        PWM[1] = 0;
      }
    break;
  }

  //Run PI controller for each wheel 
  for (int i = 0; i<2; i++) { // Calculate Porportional controler for each wheel
      pos_error[i] = desiredPos[i] - currentEncoderCountRad[i]; // calculate position error from desired pos
      integralError[i] = integralError[i] + pos_error[i]*((float)(desired_Ts_ms/1000)); // integral error
      desiredVel[i] = Kp_pos*pos_error[i] + Ki_pos * integralError[i]; // calculate desired velocity for each wheel with K values found in simulation
      if (abs(desiredVel[i]) > 4) {
        if(desiredVel[i] < 0) {
          desiredVel[i] = -4;
        } else {
          desiredVel[i] = 4;
        }
      }
      error[i] = desiredVel[i] - vel[i];
      voltage[i] = Kp * error[i];
      if (voltage[i] >= 0) {
        digitalWrite(MotorSign[i],LOW);
      } else {
        digitalWrite(MotorSign[i], HIGH);   
      }
      PWM[i] = 255*abs(voltage[i])/batteryVoltage; //run motors with calculated PID 
      analogWrite(MotorVoltage[i],min(PWM[i],255));
  }


  while(millis()<last_time_ms+desired_Ts_ms){
    //wait till desired time passes
  }
  last_time_ms=millis();
}
