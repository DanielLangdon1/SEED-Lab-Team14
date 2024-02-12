/*
Name: Daniel Langdon
Code: Velocity Control
Setup:
Using the motor drive Polula Dual MC33926 Motor Driver shield, the Arduino Uno, and 
a Voltage Monitor. Connect the Motor Driver onto the Arduino Uno, connect a battery to the
voltage monitor and connect the monitor to the motor driver through the two pins. Then connect 
the motor to the two pins next to voltage pins of the motor driver and the encoder wired to an interrupt pin 2 or 3 
and digital pin 5 or 6 with the blue and green wired going to Vcc and ground pins on the motor driver. The motor sign pins 
allow voltage to flow to the motor and the Voltage pins send the voltage to allow the motor to spin. 
Code Usage:
This code will then use a proportional controller to set the velocity of the motor to a specified value called desiredVel that you can change in the code
Just connect to the arduino uno on your computer and send this code to the arduino to run
*/

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

float lastEncoderTime; //Initializes time variable for use in ISR
float startTime; //Set up in Setup function to calculate current time in loop
float initialTime; //Used to calculate time elapsed for velocity
float lastTime; //Used to compute current time
float currentTime; //Initializes current time for loop

int count; //Initializes encoder counts for the ISR
int currentEncoderCount; //Intilization for Encoder Count to be used in loop 
float currentEncoderCountRad; //Converted counts to radian for velocity in rad/s
int initialEncoderCount = 0; //Found in setup to check for change in counts
float initialEncoderCountRad; //Radian version of initial count
float vel; //Velocity found from timeelapsed and count/radian change
float desiredVel = 5; //Desired velocity to achieve in radian/second
float Kp = 2.5; //Controller gain found from experiments
float voltage = 0; // voltage to be used later to control speed
float batteryVoltage = 7.8; //Sets battery voltage to 7.8
float error; // Error in speed to adjust and get the right speed
unsigned int PWM; //PWM variable to be used later

//ISR to check for a change in state of the encoder
void myISR() {
  if(micros() - lastEncoderTime > 100) {
    if(digitalRead(encoderInterrupts[0]) == digitalRead(encoderPins[0])) {
      count++;
      count++;
    } else{
      count--;
      count--;
    }
    lastEncoderTime = micros();
  }
}

//Function to return encoder counts
int MyEnc() {
  if(digitalRead(encoderInterrupts[0]) != digitalRead(encoderPins[0])) {
    return(count+1);
  } else{
    return(count);
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
    attachInterrupt(digitalPinToInterrupt(encoderInterrupts[i]), myISR, CHANGE);
  }
  pinMode(MotorEnable, OUTPUT);
  digitalWrite(MotorEnable,HIGH);

  // Assigns values to variable defined above that will use initial values in the loop
  initialEncoderCount = MyEnc();
  initialEncoderCountRad = 2*PI*(float)(initialEncoderCount)/3200;
  startTime = millis();
  initialTime = millis();
  lastEncoderTime = micros();

  //Begin Serial monitor at baud rate 9600
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Set initial voltage to zero and get last time through millis function
  voltage = 0;
  lastTime = millis();
  //Compute current time
  currentTime = (float)((lastTime-startTime)/1000);

  // Use for Experimental Data set voltage to 3 for 3 seconds Commented out so actual controller can be uses
  /*if(currentTime >=1 && currentTime <=3) {
    voltage = 3;
  } else {
    voltage = 0;
  }
  // Sets motor to voltage level. Only used one motor to get K, Sigma, and Kp value
  digitalWrite(MotorSign[1],HIGH);
  analogWrite(MotorVoltage[1],voltage*255/5);
  digitalWrite(MotorSign[0],HIGH);
  analogWrite(MotorVoltage[0],0);*/

  // Retrieves the current count from the encoders and converts to radians
  currentEncoderCount = MyEnc();
  currentEncoderCountRad = 2*PI*(float)(currentEncoderCount)/3200;

  // Calculates the time elapsed between now and the last time and finds velocity
  float timeElapsed = (float)(millis()-initialTime)/1000;
  vel = (currentEncoderCountRad-initialEncoderCountRad)/timeElapsed;
  
  // Sets old counts and times to current counts and times for future use
  initialEncoderCount = currentEncoderCount;
  initialEncoderCountRad = currentEncoderCountRad;
  initialTime = millis();
  //Finds error between desired velocity and actual velocity and sets voltage to correct value to correct this error
  error = desiredVel - vel;
  voltage = Kp*error;
  // If voltage is needed (error is greater than 0) then set voltage for Motor to high
  if (voltage > 0) {
    digitalWrite(MotorSign[1],HIGH);
  } else {
    digitalWrite(MotorSign[1], LOW);
  }

  //Apply voltage up to maximum available 
  PWM = 255*abs(voltage)/batteryVoltage;
  analogWrite(MotorVoltage[1],min(PWM,255));

// Used for experimental data to get K, Sigma, and Kp values. Commented out since not needed for actual task
/*
  if(currentTime<=3) {
    Serial.print(currentTime,3);
    Serial.print(" ");
    Serial.print(voltage);
    Serial.print(" ");
    Serial.println(vel,3);
    Serial.println("");
  } else{
    //Serial.println("Finished");
  }*/

  while(millis()<last_time_ms+desired_Ts_ms){
    //wait till desired time passes
  }
  last_time_ms=millis();
}
