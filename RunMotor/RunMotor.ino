
///////////////Calculation vars
float pi = 3.14159;
int i;
float startTime = 0;
float elapsedTime = 0;
float currentTime = 0; 
float prevTime = 0;
float lastTime1 = 0;
float lastTime2 = 0;

float b = 0.355; //m between middles of wheels
float r = 0.0685; //wheel rad m
float angVel1 = 0;
float angVel2 = 0;
float linVel1 = 0;
float linVel2 = 0;
float dist1 = 0;
float dist2 = 0;
float radians1 = 0;
float radians2 = 0;
float oldRadians1 = 0;
float oldRadians2 = 0;

float xVel1 = 0;
float xVel2 = 0;
float xNew = 0;
float yNew = 0;
float xOld = 0;
float yOld = 0;

float phi = 0;
float phiOld = 0;

float xVel = 0;
float yVel = 0;



int enablePin = 4;

int encoderA1 = 2; //Motor 1 Pins
int encoderB1 = 5;
int outPin1 = 10;
int motorDir1 = 7;

int encoderA2 = 3; //Motor 2 Pins
int encoderB2 = 6;
int outPin2 = 9;
int motorDir2 = 8;

////////////////////Encoder vars
int encoder1Count = 0;
int encoder2Count = 0;
int currA1 = 0;
int currA2 = 0;
int currB1 = 0;
int currB2 = 0;
int lastA1 = 0;
int lastB1 = 0;
int lastA2 = 0;
int lastB2 = 0;

void setup() {

  pinMode(motorDir1,OUTPUT);
  pinMode(motorDir2,OUTPUT);

  pinMode(outPin1,OUTPUT);
  pinMode(outPin2,OUTPUT);


  pinMode(enablePin,OUTPUT);
  digitalWrite(enablePin, HIGH);

  digitalWrite(encoderA1,HIGH);
  digitalWrite(encoderA2,HIGH);

  attachInterrupt(digitalPinToInterrupt(encoderA1), count1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA2), count2, CHANGE);

  Serial.begin(9600);

  startTime = millis();
  
}

void loop() {
  prevTime = millis();
  currentTime = (prevTime-startTime)/1000;
  elapsedTime = (millis()-startTime)/1000;

  digitalWrite(motorDir1,LOW);
  digitalWrite(motorDir2, LOW);
  analogWrite(outPin1, 200);
  analogWrite(outPin2, 200);
  int currCount1 = MyEnc1();
  int currCount2 = MyEnc2();

  ////////////////////////////////////////////////////////////////////calculate new position data
  xNew = cos(phiOld)*(((dist2) + (dist1))/2);
  yNew = sin(phiOld)*(((dist2) + (dist1))/2);
  xVel = cos(phi)*(linVel1+linVel2)/2;
  yVel = sin(phi)*(linVel1+linVel2)/2;
  phi = (((dist1-oldRadians1*r) + (dist2-oldRadians1*r)/b));
  xOld = xNew;
  yOld = yNew;
  
  while (phi > 2*pi || phi < -2*pi){
    if(phi > 2*pi){
      phi = phi-2*pi;}
    else{
      phi = phi+2*pi;
    }
  }
  phiOld = phi;
////////////////////////////////////  debug o/p's

  // Serial.print(digitalRead(encoderA1));
  // Serial.print("\t");
  // Serial.print(digitalRead(encoderB1));
  // Serial.print("\t");
  // Serial.print("count: ");
  // Serial.print(encoder1Count);
  // Serial.print("\t");
  // Serial.print("Radians: ");
  // Serial.print(radians1);
  // Serial.print("currentCounts: ");
  // Serial.print(encoder1Count);
  // Serial.print("\t");
  // Serial.print(encoder2Count);
  // Serial.print("\t");
  // Serial.print("distance 1: ");
  // Serial.print(dist1);
  // Serial.print("\t");
  // Serial.print("distance 2: ");
  // Serial.print(dist2);
  // Serial.print("\t");
  // Serial.print("linear velocity 1,2:");
  // Serial.print(linVel1);
  // Serial.print("\t");
  // Serial.print(linVel2);
  // Serial.print("\t");
  // Serial.print("phi: ");
  // Serial.println(phi);

//*/
/////////////////////////////////////  MATLAB o/p
  Serial.print(elapsedTime);
  Serial.print("\t");
  Serial.print(xNew);
  Serial.print("\t");
  Serial.print(yNew);
  Serial.print("\t");
  Serial.println(phi);
}
void count2(){
  lastA2 = currA2;
  currA2 = digitalRead(encoderA2);
  if(micros()-lastTime1>100){
    if(currA2 == digitalRead(encoderB2)){
    encoder2Count = encoder2Count+2;
    }
  
    else{
      encoder2Count = encoder2Count-2;
    }
  }
  //////////////////////////////////////    WHEEL 2 CALCULATIONS 

  radians2 = (encoder2Count*2*pi)/3200;
  angVel2 = ((radians2-oldRadians2)/(currentTime-lastTime2));   //angular velocity
  linVel2 = angVel2*r;                                         //linear velocity
  dist2 = radians2*r;                                          //left wheel distance traveled;
  oldRadians2 = radians2;                                      //stores current position for later reference

  
  lastTime1 = millis();
}


long MyEnc2(){
 if(digitalRead(encoderA2) != digitalRead(encoderB2)){
  return(++encoder2Count);
 }
 else{
  return(encoder2Count);
 }

}

void count1(){
  lastA1 = currA1;
  currA1 = digitalRead(encoderA1);
  if(micros()-lastTime1>100){
    if(currA1 == digitalRead(encoderB1)){
    encoder1Count = encoder1Count+2;
    }
  
    else{
      encoder1Count = encoder1Count-2;
    }
  }
  /////////////////////////////////////     WHEEL 1 CALCULATIONS 

  radians1 = (encoder1Count*2*pi)/3200;
  angVel1 = ((radians1-oldRadians1)/(currentTime-lastTime1));   // angular velocity
  linVel1 = angVel1*r;                                         // linear velocity
  dist1 = radians1*r;                                          // distance traveled;
  oldRadians1 = radians1;                                      //stores current position for later reference
  
  lastTime2 = millis();
}

long MyEnc1(){
 if(digitalRead(encoderA1) != digitalRead(encoderB1)){
  return(encoder1Count+1);
 }
 else{
  return(encoder1Count);
 }
}
