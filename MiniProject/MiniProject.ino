#include <Wire.h>
#define MY_ADDR 8
#define PI 3.14159265358979323 // Defines irrational number PI for mathematical use later

// Global variables to be used for I2C communication
volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;
int wheel_1=0;
int wheel_2=0;


// These lines define the pins each motor and encoder pins go to 
const int MotorVoltage[2] = {9,10}; 
const int MotorSign[2] = {7,8};
const int encoderInterrupts[2] = {2,3};
const int encoderPins[2] = {5,6};
const int MotorEnable = 4;

// Defines desired delay time
unsigned long desired_Ts_ms = 25;
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
float Kp_pos = 9.241809; //Controller gain for proportional 
float Ki_pos = 1.37281; //Controller gain for integrator
float Kp = 3.3;
float voltage[2] = {0,0}; // voltage to be used for speed and position control
float batteryVoltage = 7.8; //Sets saturation point for battery
//Defines error values for position and integral
float pos_error[2]= {0,0}; 
float desiredPos[2]= {0,0};
// float actualPos[2];
float integralError[2] = {0,0};
float error[2] = {0,0};

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
    if (count1 >= 3200) {
      count1-=3200;
    } else if (count1 <= -3200) {
        count1 += 3200;
    }
    return(count1+1);
  } else{
     if (count1 >= 3200) {
      count1-=3200;
    } else if (count1 <= -3200) {
        count1 += 3200;
    }
    return(count1);
  }
}

//Function to return encoder counts
int MyEnc2() {
  if(digitalRead(encoderInterrupts[1]) != digitalRead(encoderPins[1])) {
  if (count1 >= 3200) {
      count2-=3200;
    } else if (count1 <= -3200) {
        count2 += 3200;
    }
    return(count2+1);
  } else{
     if (count2 >= 3200) {
      count2-=3200;
    } else if (count2 <= -3200) {
        count2 += 3200;
    }
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

}

void loop() {
  // put your main code here, to run repeatedly:
  lastTime = millis();
  //Compute current time
  currentTime = (float)((lastTime-startTime)/1000);
  // If there is data on the buffer, read it
  if (msgLength > 0) {
    printReceived();
    msgLength = 0;
  }

  timeElapsed = (float)(millis()-initialTime)/1000;
  currentEncoderCount[0] = MyEnc1();

  if (currentEncoderCount[0] >= 3200){ //wraps encoder count around 2pi for wheel 1
    currentEncoderCount[0] = currentEncoderCount[0] - 3200;
  }
  else if (currentEncoderCount[0] <= -3200){
        currentEncoderCount[0] = currentEncoderCount[0] + 3200;
  }
  
  currentEncoderCountRad[0] = 2*PI*(float)(currentEncoderCount[0])/3200;
  timeElapsed = (float)(millis()-initialTime)/1000;
  vel[0] = (currentEncoderCountRad[0]-initialEncoderCountRad[0])/timeElapsed;


  currentEncoderCount[1] = MyEnc2();

  if (currentEncoderCount[1] >= 3200){//wraps encoder count around 2pi for wheel 2
    currentEncoderCount[1] = currentEncoderCount[1]- 3200;
  }
  else if (currentEncoderCount[1]<= -3200){
        currentEncoderCount[1] = currentEncoderCount[1] +3200;
  }
  currentEncoderCountRad[1] = 2*PI*(float)(currentEncoderCount[1])/3200;
  vel[1] = (currentEncoderCountRad[1]-initialEncoderCountRad[1])/timeElapsed;

  initialEncoderCount[0] = currentEncoderCount[0];
  initialEncoderCountRad[0] = currentEncoderCountRad[0];
  initialEncoderCount[1] = currentEncoderCount[1];
  initialEncoderCountRad[1] = currentEncoderCountRad[1];



  initialTime = millis();
  desiredPos[0] = 0;
  desiredPos[1] = 0;
  
  // if (wheel_1 == 1 ) {
  //   desiredPos[0] = PI;
  // } else if (wheel_1 == 0) {
  //   desiredPos[0] = 0;
  // }
  // if (wheel_2 == 1) {
  //   desiredPos[1] = PI;
  // } else if (wheel_2 == 0) {
  //   desiredPos[1] = 0;
  // } 

  for (int i = 0; i<2; i++) {
    

    pos_error[i] = desiredPos[i] - currentEncoderCountRad[i];

    integralError[i] = integralError[i] + pos_error[i]*((float)(desired_Ts_ms/1000));
    desiredVel[i] = Kp_pos*pos_error[i] + Ki_pos * integralError[i];

    error[i] = desiredVel[i] - vel[i];
    voltage[i] = Kp * error[i];

    if (voltage[i] > 0) {
      digitalWrite(MotorSign[i],HIGH);
    } else {
      digitalWrite(MotorSign[i], LOW);
    }
    PWM[i] = 255*abs(voltage[i])/batteryVoltage;
    analogWrite(MotorVoltage[i],min(PWM[i],255));

  }
    // Serial.print(vel[0]);
    // Serial.print("\t");
    // Serial.print(vel[1]);
    // Serial.print("\t");
    Serial.print(integralError[0]);
    Serial.print("\t");
    Serial.print(integralError[1]);
    Serial.print("\t pos err ");
    Serial.print(pos_error[0]);
    Serial.print("\t");
    Serial.print(pos_error[1]);   
    Serial.print("\tdesired vel ");
    Serial.print(desiredVel[0]);  
    Serial.print("\t");
    Serial.print(desiredVel[1]);
    Serial.print("\t encoder");
    Serial.print(initialEncoderCountRad[0]);
    Serial.print("\t");
    Serial.print(initialEncoderCountRad[1]);
    Serial.print("\t");
    Serial.print(currentEncoderCountRad[0]);
    Serial.print("\t");
    Serial.print(currentEncoderCountRad[1]);
    Serial.print("\t");
    Serial.print(currentEncoderCount[0]);
    Serial.print("\t");
    Serial.println(currentEncoderCount[1]);
    // Serial.print("\t");
    // Serial.print(count1);
    // Serial.print("\t");
    // Serial.println(count2);



  while(millis()<last_time_ms+desired_Ts_ms){
    //wait till desired time passes
  }
  last_time_ms=millis();
}
// printReceived helps us see what data we are getting from the leader
void printReceived() {

  Serial.print("Message: ");
  for (int i=0;i<msgLength;i++) {
    Serial.print(String((char) instruction[i]));
  }
  Serial.println(""); 

  wheel_1 =  instruction[0] - 48;
  wheel_2 = instruction[1] - 48;

  Serial.print("Wheel 1: ");
  Serial.print(wheel_1);
  Serial.println("");

  Serial.print("Wheel 2: ");
  Serial.print(wheel_2);
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

