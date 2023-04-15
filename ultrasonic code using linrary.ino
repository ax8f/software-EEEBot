#include <NewPing.h>
//#include <TinyDebugSerial.h>
#define TRIGGER_PIN 2 // pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN 4 // pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 50 // Maximum distance we want to ping for (in centimeters). Maximum sensor


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum

//TinyDebugSerial mySerial = TinyDebugSerial();
void setup() {

Serial.begin(9600); // set baud rate at 9600 baud to see ping results
}
void loop()
{

 int time = sonar.ping_median(5); //median off 5 values_ and it returen the time in microseconds
 distance = sonar.convert_cm(time); //convert that to cm, replace "cm" with "in" for inches
 Serial.print("distance: ");
 Serial.print(distance); // //print value to screen so we can see it.
 Serial.println(" cm");


}




