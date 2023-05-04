
#include "Wire.h"
#include <MPU6050_light.h>
#include <NewPing.h> // to read the 

#define TRIGGER_PIN 25 // pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN 26 // pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 50 // Maximum distance we want to ping for (in centimeters). Maximum sensor
//////////////////////////////////////////////////////////////////////////////////////////
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum
int distance_ultrasonic; //to store the distance in cm

void setup() {
  Serial.begin(9600); 
  Wire.begin(); // set up as master
  }


void loop() {

// Wire.beginTransmission(8); // transmit to device #8
// Wire.write('a');        // sends five bytes
// Wire.write(raw_angle_MPU);   // sends one byte
// Wire.endTransmission();    // stop transmitting
	
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  delay(300);
  
  distance_ultrasonic = sonar.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.print("Distance: ");
  Serial.println(distance_ultrasonic);
  //Serial.println("cm");
  Wire.beginTransmission(8); // transmit to device adress 8 
  Wire.write('d');        // send char to allow the slave to store the following data to correct variable
  Wire.write(distance_ultrasonic);   // sends one byte
  Wire.endTransmission();    // stop transmitting
  }






