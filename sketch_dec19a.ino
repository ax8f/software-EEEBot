#include <NewPing.h>

/*******Interrupt-based Rotary Encoder Sketch*******
by Simon Merrett, based on insight from Oleg Mazurov, Nick Gammon, rt, Steve Spence
*/

 int pinA = 2; // Our first hardware interrupt pin is digital pin 2
 int pinB = 3; // Our second hardware interrupt pin is digital pin 3
 byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
 byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
//volatile byte encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
int encoderPos = 0;
//volatile byte oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
int oldEncPos = 0;
byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
float angular = 0;

int count=0 ; // for each 6 count one complete cycle
float displacement =0; //in cm 

int distance_count;
float distance ;

void setup() {
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0,PinA,RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1,PinB,RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)
  Serial.begin(9600); // start the serial monitor link
}

void PinA(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    count --;
    distance_count ++;
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void PinB(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    count ++;
    distance_count ++;    
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void loop(){
  if(oldEncPos != encoderPos) {
    
    angular = 1.05*count;
    displacement = 3*angular;
    distance = 3*(1.05*distance_count);    
    Serial.print("angule: ");
    Serial.println(angular);
    Serial.print("count: ");
    Serial.println(count);
    Serial.print("dispalcement: ");
    Serial.println(displacement);
    Serial.print("distance: ");
    Serial.println(distance);
    oldEncPos = encoderPos;

        
  }
}