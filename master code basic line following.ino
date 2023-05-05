// basic line following ESP32 code. 
 #include "Wire.h"
 
 # define photodiode_1 A15 // the yallow wire
 # define photodiode_2 A2 // the green wire
 # define photodiode_3 A0 // the orange wire
 # define photodiode_4 A4 // the purple wire 


int sensor2; //the green wire
int sensor3; // the orange wire



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  Wire.begin(); // set up as master

  pinMode( photodiode_1, INPUT);
  pinMode( photodiode_2, INPUT);
  pinMode( photodiode_3, INPUT);
  pinMode( photodiode_4, INPUT);

  

}

void loop() {
  
// using the two middle sensors, photodiode_2 and  photodiode_3

int photodiode2_value = analogRead(A2);
int photodiode3_value = analogRead(A0);

map(photodiode2_value, 0, 4095, 0, 255); // convert the range of the sensors 2 to one byte
map(photodiode3_value, 0, 4095, 0, 255); // convert the range of the sensors 3 to one byte

Serial.print("the photodiode 2 value is");
Serial.println(photodiode2_value);

Serial.print("the photodiode 3 value is");
Serial.println(photodiode3_value);
delay(200);

// send the value of sensor 2, char b is used to 
Wire.beginTransmission(8); // transmit to device #8
Wire.write('b');        // sends five bytes
Wire.write( photodiode2_value);   // sends one byte
Wire.endTransmission();    // stop transmitting


// send the value of sensor 2, char c is used to 
Wire.beginTransmission(8); // transmit to device #8
Wire.write('c');        // sends five bytes
Wire.write( photodiode3_value);   // sends one byte
Wire.endTransmission();    // stop transmitting


}


