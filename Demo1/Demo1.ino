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
float Kp_pos = 7.95548607436073; //Controller gain for proportional 
float Ki_pos = 0.5; //Controller gain for integrator
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
float distanceTraveled = 0;
int mode = 0; ///0 for turn 1 for move
int count = 0;

float xVel1 = 0;
float xVel2 = 0;
float xNew = 0;
float yNew = 0;
float xOld = 0;
float yOld = 0;
float distance[2] = {0,0};


float phiNew = 0;
float phiOld = 0;

float desiredPhi = 0;
float phiError = 0;
float intErrorPhi = 0;
float phiVelError = 0;
float phiVel = 0;
float desiredPhiVel = 0;
float KpPhiPos = 7.025265;
float KiPhiPos = .740405;
float KpPhiVel = .213320;


float xVel = 0;
float yVel = 0;
float desiredDis;
float disError;
float intErrorDis = 0;
float forwardVel;
float deltaV;
float Vbar;
float Vleft;
float Vright;
float disVelError;
float desiredDisVel;


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
    // if (count1 >= 3200) {
    //   count1-=3200;
    // } else if (count1 <= -3200) {
    //     count1 += 3200;
    // }
    return(count1+1);
  } else{
    //  if (count1 >= 3200) {
    //   count1-=3200;
    // } else if (count1 <= -3200) {
    //     count1 += 3200;
    // }
    return(count1);
  }
}

//Function to return encoder counts
int MyEnc2() {
  if(digitalRead(encoderInterrupts[1]) != digitalRead(encoderPins[1])) {
  // if (count1 >= 3200) {
  //     count2-=3200;
  //   } else if (count1 <= -3200) {
  //       count2 += 3200;
  //   }
    return(count2+1);
  } else{
    //  if (count2 >= 3200) {
    //   count2-=3200;
    // } else if (count2 <= -3200) {
    //     count2 += 3200;
    // }
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

  desiredPhi = PI/2;
  desiredDis = 1.524; //M

}

void loop() {
  // put your main code here, to run repeatedly:
  lastTime = millis();
  //Compute current time
  currentTime = (float)((lastTime-startTime)/1000);
  timeElapsed = (float)(millis()-initialTime)/1000;
  currentEncoderCount[0] = MyEnc1();
  currentEncoderCount[1] = MyEnc2();

  for(int i = 0;i<2;i++){ // configure encoders
    currentEncoderCountRad[i] = 2*PI*(float)(currentEncoderCount[i])/3200;
    vel[i] = (currentEncoderCountRad[i]-initialEncoderCountRad[i])/timeElapsed;
    distance[i] = currentEncoderCountRad[i]*r;
  }
  phiNew = (((initialEncoderCountRad[0]*r - distance[1]) + (initialEncoderCountRad[1]*r-distance[1]))/b);
  phiVel = (((vel[0]*r) - (vel[1]*r))/b);
  for(int i = 0;i<2;i++){ 
    initialEncoderCount[i] = currentEncoderCount[i];
    initialEncoderCountRad[i] = currentEncoderCountRad[i];
  }

  initialTime = millis();
  switch (mode){
    case 0:
      if (desiredPhi > 0){ ///calculate desired anglular position for each wheel based off of phi
        desiredPos[0] = desiredPhi*(b/(2*r));
        desiredPos[1] = -1*desiredPhi*(b/(2*r));
      } else if (desiredPhi < 0){
        desiredPos[0] = -1*desiredPhi*(b/(2*r));  
        desiredPos[1] = desiredPhi*(b/(2*r));     
      }
      Serial.print(phiNew/(PI/180));
      Serial.print("\t");
      Serial.println(phiError);
      Serial.print("\t");
      Serial.print(currentEncoderCount[0]);
      Serial.print("\t");
      Serial.println(currentEncoderCount[1]);
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
      desiredPos[0] = desiredDis/(r);
      desiredPos[1] = desiredDis/(r);
      desiredPhi = 0;
      Serial.print(voltage[0]);
      Serial.print("\t");
      Serial.println(voltage[1]);


      if ((currentEncoderCountRad[0]*r <= desiredDis+0.0254 && currentEncoderCountRad[0]*r >= desiredDis - 0.0254) && (currentEncoderCountRad[1]*r <= desiredDis+0.0254 && currentEncoderCountRad[1]*r >= desiredDis - 0.0254) ) {
        PWM[0] = 0;
        PWM[1] = 0;
      }
    break;
    case 3:
      PWM[0] = 0;
      PWM[1] = 0;
    break;
  }
  phiError = desiredPhi - phiNew;
  if (abs(phiError) <= 0.1704 ){
    
  }
  for (int i = 0; i<2; i++) { // Calculate Porportional controler for each wheel
      pos_error[i] = desiredPos[i] - currentEncoderCountRad[i]; // calculate position error from desired pos
      integralError[i] = integralError[i] + pos_error[i]*((float)(desired_Ts_ms/1000)); // integral error
      desiredVel[i] = Kp_pos*pos_error[i] + Ki_pos * integralError[i]; // calculate desired velocity for each wheel with K values found in simulation
      if (abs(desiredVel[i]) > 5) {
        if(desiredVel[i] < 0) {
          desiredVel[i] = -5;
        } else {
          desiredVel[i] = 5;
        }
      }
      // if(vel[0] == vel[1] || mode == 0){ 
      //   error[i] = desiredVel[i] - vel[i];}
     error[i] = desiredVel[i] - vel[i];
     voltage[i] = Kp * error[i];
      // if((vel[0]>vel[1]) && (mode == 1)){
      //   voltage[0] = voltage[0] - (voltage[0]-voltage[1])/2;
      //   voltage[1] = voltage[1] + (voltage[0]-voltage[1])/2;
      // }
      // else if((vel[0]<vel[1]) && (mode == 1)){
      //   voltage[1] = voltage[1] - (voltage[1]-voltage[0])/2;
      //   voltage[0] = voltage[0] + (voltage[1]-voltage[0])/2;
      // }
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
