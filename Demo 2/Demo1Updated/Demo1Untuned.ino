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

volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;
volatile uint8_t distanceBit = 0;
volatile uint8_t angleBit = 0;
volatile bool detection_flag = false;
volatile bool circle_flag = false;

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
float KiPhi = 0.45;
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
float desiredRho = .3048; //m
float errorPhi = 0;
float derivativePhi = 0;
float integralPhi = 0;
float phi = 0;
int mode = 2;
int lastMode = 2;
float startCircleTime;
int freq = 1;
float stutter;



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

  // Initialize I2C
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts
  Wire.onReceive(receive);

  startTime = millis();
  initialTime = millis();

  // Variables for desired angle and distance to travel in radians and meters
  desiredPhi = PI/2;
  // desiredRho = 0.3;
}

void loop() {
  // put your main code here, to run repeatedly:
  // analogWrite(MotorVoltage[0],100);

    // If there is data on the buffer, read it
  if (msgLength > 0) {
    if (offset==1) {
      digitalWrite(LED_BUILTIN,instruction[0]);
    }
    printReceived();
    msgLength = 0;
  }

    lastTime = millis();
    //Compute current time
    currentTime = (float)((lastTime-startTime)/1000);
    timeElapsed = (float)(millis()-initialTime)/1000;
    currentEncoderCount[0] = MyEnc1();
    currentEncoderCount[1] = MyEnc2();


    for(int i = 0;i<2;i++){ // set encoders to radians and finds velocity
      currentEncoderCountRad[i] = 2*PI*(float)(currentEncoderCount[i])/3200;
      if(timeElapsed > 0 ){
        vel[i] = (currentEncoderCountRad[i]-initialEncoderCountRad[i])/timeElapsed;
      }
    }
    Rho = (r/2)*(currentEncoderCountRad[0]+currentEncoderCountRad[1]);
    phi = (r/d) * (currentEncoderCountRad[0]-currentEncoderCountRad[1]);



    switch (mode) {
      case 0:
        if (desiredPhi >= 0) {
          desiredRhoVel = 0;
          desiredPhiVel = 5;
        } else if (desiredPhi < 0) {
          desiredRhoVel = 0;
          desiredPhiVel = -5;
        }
        if (phi <= desiredPhi + PI/180 && phi >= desiredPhi-PI/180) {
          Serial.print("reached desired phi");
          if(lastMode == 1){
            lastMode = mode;
            mode = 4;
            startCircleTime = currentTime;

          }
          else if(lastMode == 2){
            lastMode = mode;
            mode = 1;
            
            }
        }
      break;
      case 1:
        //desiredRho = 2.5;
        //desiredPhi = 0;
        if(abs(Rho) >= (desiredRho) || circle_flag == true) {//reached marker
          Serial.print("reached desired Rho");
          lastMode = mode; 
          mode = 0; //turn 90deg
          desiredPhi = PI/2;        
          }
      break;
      case 2: //spin while waiting
        if (currentTime  < 2){
          desiredPhiVel = 4;
          desiredRho = 0;
          }
        else {
          stutter = currentTime;
          lastMode = mode;
          desiredPhiVel = 0;
          desiredRhoVel = 0;
          mode = 3;
        }
        
        
        if (detection_flag == true){
          Serial.print("Found marker");
          desiredPhiVel = 0;
          desiredRhoVel = 0;
          lastMode = mode;
          mode = 0;
          count1 = 0;
          count2 = 0;
          delay(500);
        }
        //delay(1000);
        break;
      case 3: //wait
        Serial.print("waiting...");
        desiredPhiVel = 0;
        desiredRhoVel = 0;
        if(lastMode == 2 && (currentTime - stutter > 5)){
          lastMode = 3;
          mode = 2;
        }
        // analogWrite(MotorVoltage[0], 0);
        // analogWrite(MotorVoltage[1], 0);
      case 4:
        Serial.print("tracing... ");
        desiredRhoVel = 5;
        desiredPhiVel = desiredRhoVel/(.3048*2);
        if (startCircleTime-currentTime >= 7) {
          desiredRhoVel = 0;
          desiredPhiVel = 0;
          mode = 3;
          lastMode = 4;
        }

        // startCircleTime = currentTime;


        break;
    }

  //Serial.println(mode);
  if(mode == 1 || mode == 0 || mode == 2) {
    errorPhi = desiredPhi - phi;
    derivativePhi = (errorPhi - errorPhiInitial)/((float)(desired_Ts_ms/1000));
    integralPhi = integralPhi + errorPhi*((float)(desired_Ts_ms/1000));
    desiredPhiVel = errorPhi*KpPhi + KdPhi*derivativePhi + KiPhi*integralPhi;
    if(abs(desiredPhiVel) > 10){
      desiredPhiVel = 10 *desiredPhiVel/abs(desiredPhiVel);
    }

    errorRho = desiredRho - Rho;
    derivativeRho = (errorRho - errorRhoInitial)/((float)(desired_Ts_ms/1000));
    integralRho = integralRho + errorRho*((float)(desired_Ts_ms/1000));
    desiredRhoVel = errorRho*KpRho + KdRho*derivativeRho + KiRho*integralRho;

    /*if(abs(desiredRhoVel) > 10){
        desiredRhoVel = 10 *desiredRhoVel/abs(desiredRhoVel);
    }*/
  }
    phiVel = (r/d)*(vel[0]-vel[1]);
    rhoVel = (r/2)*(vel[0]+vel[1]);

    errorRhoVel = rhoVel - desiredRhoVel;
    errorPhiVel = phiVel - desiredPhiVel;

    // Serial.print(currentEncoderCountRad[0]);
    // Serial.print("\t");
    // Serial.print(currentEncoderCountRad[1]);
    // Serial.println("\t");
    Serial.print(Vbar);
    Serial.print("\t");
    Serial.print(deltaV);
    Serial.print("\t");
    Serial.print(errorPhiVel);
    Serial.print("\t");
    Serial.print(errorRhoVel);
    Serial.print("\t mode: ");
    Serial.print(mode);
    Serial.print("\t last mode: ");
    Serial.print(lastMode);
    Serial.print("\t voltage: ");
    Serial.print(voltage[0]);
    Serial.print("\t");
    Serial.println(voltage[1]);
    // Serial.print("\t");
    // Serial.print(rhoVel);
    // Serial.print("\t");
    // Serial.print(derivativeRho);
    // Serial.print("\t");
    // Serial.println(errorRho);

    Vbar = errorRhoVel*KpRhoVel;
    deltaV = errorPhiVel*KpPhiVel;

    voltage[0] = (Vbar+deltaV)/2;
    voltage[1] = (Vbar - deltaV)/2;
    // if (mode == 0 || mode == 2) {
    //   voltage[0] = abs(voltage[0])*-1;
    // }

    for(int i = 0; i < 2; i++) {
      //PWM[i] = 255*abs(voltage[i])/batteryVoltage;
      if(voltage[i] >= 0) {
        digitalWrite(MotorSign[i],HIGH);
        analogWrite(MotorVoltage[i],abs(voltage[i]));
      } else {
          digitalWrite(MotorSign[i],LOW);
          analogWrite(MotorVoltage[i],abs(voltage[i]));
        }
    }
  

    //Sets old values to new values
    errorRhoVelInitial = errorRhoVel;
    errorPhiVelInitial = errorPhiVelInitial;
    errorRhoInitial = errorRho;
    errorPhiInitial = errorPhi;
    freq++;
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
  Serial.print("Message: ");
  for (int i=0;i<msgLength;i++) {
    Serial.print(String((char) instruction[i]));
  }
  Serial.println(""); 

  // Pulling info from our Pi encoding
  distanceBit =  instruction[0] - 48;
  angleBit = instruction[1] - 48;

  Serial.print("Distance bit: ");
  Serial.print(distanceBit);
  Serial.println("");

  Serial.print("Angle bit: ");
  Serial.print(angleBit);
  Serial.println("");

  // Decoding the bits to an average distance / angle
  // DISTANCE
  if (distanceBit == 1) {
    desiredRho = 1;
  } else if (distanceBit == 2) {
    desiredRho = 0.875;
  } else if (distanceBit == 3) {
    desiredRho = 0.625;
  } else if (distanceBit == 4) {
    desiredRho = 0.425;
  } else if (distanceBit == 5) {
    desiredRho = 0.30;
  } else if (distanceBit == 6) {
    desiredRho = 0.20;
  } else if (distanceBit == 7) {
    desiredRho = 0.125;
  } else if (distanceBit == 8) {
    desiredRho = 0.65;
  } else if (distanceBit == 9) {
    desiredRho = 0.00;
    circle_flag = true;
  }

  Serial.print("Distance: ");
  Serial.print(desiredRho);
  Serial.println("");

  // ANGLE
  if (circle_flag == true) {
    // do nothing
  }
  else if (angleBit == 1) {
    desiredPhi = -22.5;
    desiredPhi = desiredPhi * 0.0174533;

  } else if (angleBit == 2) {
    desiredPhi = -15;
    desiredPhi = desiredPhi * 0.0174533;

  } else if (angleBit == 3) {
    desiredPhi = -7.5;
    desiredPhi = desiredPhi * 0.0174533;

  } else if (angleBit == 4) {
    desiredPhi = -3.5;
    desiredPhi = desiredPhi * 0.0174533;

  } else if (angleBit == 5) {
    desiredPhi = 0;
    desiredPhi = desiredPhi * 0.0174533;

  } else if (angleBit == 6) {
    desiredPhi = 3.5;
    desiredPhi = desiredPhi * 0.0174533;

  } else if (angleBit == 7) {
    desiredPhi = 7.5;
    desiredPhi = desiredPhi * 0.0174533;

  } else if (angleBit == 8) {
    desiredPhi = 15;
    desiredPhi = desiredPhi * 0.0174533;

  } else if (angleBit == 9) {
    desiredPhi = 22.5;
    desiredPhi = desiredPhi * 0.0174533;
  }


  Serial.print("Angle: ");
  Serial.print(desiredPhi);
  Serial.println("");

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
