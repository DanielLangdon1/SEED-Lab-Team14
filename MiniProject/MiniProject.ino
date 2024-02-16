#define PI 3.14159265358979323 // Defines irrational number PI for mathematical use later

// These lines define the pins each motor and encoder pins go to 
const int MotorVoltage[2] = {9,10}; 
const int MotorSign[2] = {7,8};
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

int count1; //Initializes encoder counts for the ISR
int count2; //Initializes encoder counts for the second encoder ISR
int currentEncoderCount[2]; //Intilization for Encoder Counts to be used in loop 
float currentEncoderCountRad[2]; //Converted counts to radian for velocity in rad/s
int initialEncoderCount[2] = {0,0}; //Found in setup to check for change in counts
float initialEncoderCountRad[2]; //Radian version of initial count
float vel[2]; //Velocity found from timeelapsed and count/radian change
float desiredVel[2]; //Desired velocity to achieve in radian/second
float Kp; //Controller gain for proportional 
float Ki; //Controller gain for integrator
float voltage = 0; // voltage to be used for speed and position control
float batteryVoltage = 7.8; //Sets saturation point for battery
//Defines error values for position and integral
float pos_error[2]; 
float desiredPos[2];
float actualPos[2];
float integralError[2];
float error[2];

unsigned int PWM; //PWM variable to be used for later

//ISR to check for a change in state of the encoder
void myISR1() {
  if(micros() - lastEncoderTime > 100) {
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
  if(micros() - lastEncoderTime > 100) {
    if(digitalRead(encoderInterrupts[0]) == digitalRead(encoderPins[0])) {
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
  if(digitalRead(encoderInterrupts[0]) != digitalRead(encoderPins[0])) {
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

  startTime = millis();
  initialTime = millis();
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
if (currentEncoderCount[0] != desiredEncoderCount{
    if(currentEncoderCount[0]> desiredEncoderCount){
      digitalWrite(MotorSign[0],LOW);}
    if(currentEncoderCount[0]< desiredEncoderCount){
        digitalWrite(MotorSign[0],HIGH);
    }
  }
 if (currentEncoderCount[1] != desiredEncoderCount{
    if(currentEncoderCount[1]> desiredEncoderCount){
      digitalWrite(MotorSign[1],LOW);}
    if(currentEncoderCount[1]< desiredEncoderCount){
        digitalWrite(MotorSign[1],HIGH);
    }
  }
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

