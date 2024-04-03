#include <Wire.h>
#define MY_ADDR 8
// Global variables to be used for I2C communication
volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;
volatile uint8_t distanceBit = 0;
volatile uint8_t angleBit = 0;
volatile double distance = 0;
volatile double angle = 0;
volatile bool detection_flag = false;
volatile bool circle_flag = false;

void setup() {
  Serial.begin(115200);
  // We want to control the built-in LED (pin 13)
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize I2C
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts
  Wire.onReceive(receive);
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
    distance = 1;
  } else if (distanceBit == 2) {
    distance = 0.875;
  } else if (distanceBit == 3) {
    distance = 0.625;
  } else if (distanceBit == 4) {
    distance = 0.425;
  } else if (distanceBit == 5) {
    distance = 0.30;
  } else if (distanceBit == 6) {
    distance = 0.20;
  } else if (distanceBit == 7) {
    distance = 0.125;
  } else if (distanceBit == 8) {
    distance = 0.65;
  } else if (distanceBit == 9) {
    distance = 0.00;
    circle_flag = true;
  }

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println("");

  // ANGLE
  if (angleBit == 1) {
    angle = -22.5;
  } else if (angleBit == 2) {
    angle = -15;
  } else if (angleBit == 3) {
    angle = -7.5;
  } else if (angleBit == 4) {
    angle = -3.5;
  } else if (angleBit == 5) {
    angle = 0;
  } else if (angleBit == 6) {
    angle = 3.5;
  } else if (angleBit == 7) {
    angle = 7.5;
  } else if (angleBit == 8) {
    angle = 15;
  } else if (angleBit == 9) {
    angle = 22.5;
  }

  angle = angle * 0.0174533;

  Serial.print("Angle: ");
  Serial.print(angle);
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
