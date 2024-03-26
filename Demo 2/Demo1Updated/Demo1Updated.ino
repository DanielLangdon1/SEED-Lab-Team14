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
float desired_Ts_ms = 5;
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
unsigned long int currentEncoderCount[2] = {0,0}; //Intilization for Encoder Counts to be used in loop 
float currentEncoderCountRad[2] = {0,0}; //Converted counts to radian for velocity in rad
int initialEncoderCount[2] = {0,0}; //Found in setup to check for change in counts
float initialEncoderCountRad[2] = {0,0}; //Radian version of initial count
float vel[2]; //Velocity found from timeelapsed and count/radian change;
float voltage[2] = {0,0}; // voltage to be used for speed and position control
float batteryVoltage = 7.8; //Sets saturation point for battery

//Instantiates gain values for PID velocity controller
float KiRhoVel = .25; 
float KiPhiVel = .25; 
float KdRhoVel = .03;
float KdPhiVel = .03;
float KpRhoVel = 3;
float KpPhiVel = 3;

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
float Ki = 0.1;
float Kd = 0.1;
float Kp = 5;
float desiredRho = 50;
float Rho;
float errorRho;
float errorRhoInitial;
float derivativeRho;
float integralRho;
float desiredPhi = PI;
float rotations = 0;
float desiredDis = .3048; //m
float desiredRot = 1.00*((desiredDis*4)/(2*2*PI*r));

float phi;
int mode = 1;

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
  desiredPhi = PI;
  desiredDis = 3;
  desiredRot = ((desiredDis)/(2*PI*r));

}

void loop() {
  // put your main code here, to run repeatedly:
  // analogWrite(MotorVoltage[0],100);

    lastTime = millis();
    //Compute current time
    currentTime = (float)((lastTime-startTime)/1000);
    timeElapsed = (float)(currentTime-initialTime)/1000;
    currentEncoderCount[0] = MyEnc1();
    currentEncoderCount[1] = MyEnc2();

    for(int i = 0;i<2;i++){ // set encoders to radians and finds velocity
      currentEncoderCountRad[i] = 2*PI*(float)(currentEncoderCount[i])/3200;
      vel[i] = (currentEncoderCountRad[i]-initialEncoderCountRad[i])/timeElapsed;
    }
    switch (mode) {
      case 0:
        if (desiredPhi >= 0) {
          desiredRhoVel = 0;
          desiredPhiVel = 5;
        } else if (desiredPhi < 0) {
          desiredRhoVel = 0;
          desiredPhiVel = -5;
        }

        phiVel = (r/d)*(vel[0]-vel[1]);
        rhoVel = (r/2)*(vel[0]+vel[1]);
        errorRhoVel = rhoVel - desiredRhoVel;
        errorPhiVel = phiVel - desiredPhiVel;

        derivativeRhoVel = (errorRhoVel - errorRhoVelInitial)/(desired_Ts_ms);
        derivativePhiVel = (errorPhiVel - errorPhiVelInitial)/(desired_Ts_ms);

        integralRhoVel += errorRhoVel*(desired_Ts_ms); // may need to change to *((float)(desired_Ts_ms/1000))
        integralPhiVel += errorPhiVel*(desired_Ts_ms);

        Vbar = errorRhoVel*KpRhoVel + KdRhoVel*derivativeRhoVel + KiRhoVel*integralRhoVel;
        deltaV = errorPhiVel*KpPhiVel + KdPhiVel*derivativePhiVel + KiPhiVel*integralPhiVel;

        voltage[0] = (Vbar + deltaV)/2;
        voltage[1] = (Vbar - deltaV)/2;

        for(int i = 0; i < 2; i++) {
          PWM[i] = 255*abs(voltage[i])/batteryVoltage;
          if(voltage[i] >= 0) {
            digitalWrite(MotorSign[i],LOW);
            analogWrite(MotorVoltage[0],min(PWM[i],255));
          } else {
            digitalWrite(MotorSign[i],HIGH);
            analogWrite(MotorVoltage[0],-1*min(PWM[i],255));
          }
        } 
        phi = (r/d) * (currentEncoderCountRad[0]-currentEncoderCountRad[1]);
      break;
      case 1:
        rotations = Rho/(2*PI*r);
        errorRho = desiredRho - Rho;
        derivativeRho = (errorRho - errorRhoInitial)/((float)(desired_Ts_ms/1000));
        integralRho = integralRho + errorRho*((float)(desired_Ts_ms/1000));

        desiredRhoVel = errorRho*Kp + Kd*derivativeRho + Ki*integralRho;
        desiredPhiVel = 0;

        phiVel = (r/d)*(vel[0]-vel[1]);
        rhoVel = (r/2)*(vel[0]+vel[1]);
        errorRhoVel = rhoVel - desiredRhoVel;
        errorPhiVel = phiVel - desiredPhiVel;

        derivativeRhoVel = (errorRhoVel - errorRhoVelInitial)/((desired_Ts_ms));
        derivativePhiVel = (errorPhiVel - errorPhiVelInitial)/(desired_Ts_ms);

        integralRhoVel += errorRhoVel*(desired_Ts_ms);
        integralPhiVel += errorPhiVel*(desired_Ts_ms);

        Vbar = errorRhoVel*KpRhoVel + KdRhoVel*derivativeRhoVel + KiRhoVel*integralRhoVel;
        deltaV = errorPhiVel*KpPhiVel + KdPhiVel*derivativePhiVel + KiPhiVel*integralPhiVel;
        voltage[0] = (Vbar+deltaV)/2;
        voltage[1] = (Vbar - deltaV)/2;

        for(int i = 0; i < 2; i++) {
          PWM[i] = 255*abs(voltage[i])/batteryVoltage;
          if(voltage[i] >= 0) {
            digitalWrite(MotorSign[i],HIGH);
            analogWrite(MotorVoltage[i],min(PWM[i],200));
          } else {
            digitalWrite(MotorSign[i],LOW);
            analogWrite(MotorVoltage[i],min(PWM[i],200));
          }
        }
      break;
    }
    //Sets old values to new values
    errorRhoVelInitial = errorRhoVel;
    errorPhiVelInitial = errorPhiVelInitial;
    errorRhoInitial = errorRho;
    for(int i = 0;i<2;i++){ 
      initialEncoderCount[i] = currentEncoderCount[i];
      initialEncoderCountRad[i] = currentEncoderCountRad[i];

    }
    initialTime = millis();
    while(millis()<last_time_ms+desired_Ts_ms){
      //wait till desired time passes
    }
    last_time_ms=millis();
  }
