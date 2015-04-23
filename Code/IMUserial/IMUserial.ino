/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

 This example code is in the public domain.
 */
#include "I2Cdev.h"
#include "MPU6050.h"
 
#include <Wire.h>

#define IMU_ADD 0x68

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Wire.begin();
  pinMode(13,OUTPUT);
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  
  Serial.println("Reading Sensor Values...");
  
  // read the input on analog pin 0:
  digitalWrite(13, HIGH);
  int sensorValue = analogRead(A0);
  // print out the value you read:
  Wire.requestFrom(IMU_ADD, 1);
  
  Serial.println(Wire.read());
  delay(500);        // delay in between reads for stability
  
  // read the input on analog pin 0:
  digitalWrite(13, LOW);
  // print out the value you read:
  Wire.requestFrom(IMU_ADD, 1);
  Serial.println(Wire.read());
  delay(500);        // delay in between reads for stability  
  
}
