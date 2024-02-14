#include <Wire.h>
#define MY_ADDR 8
// Global variables to be used for I2C communication
volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;
volatile uint8_t wheel_1=0;
volatile uint8_t wheel_2=0;

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