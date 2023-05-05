

#include "Wire.h"
#include "math.h"
#include <Servo.h>  //include the servo library

/////////////////////////////////////////////////////////////
#define servoPin 4
Servo myservo;        // create servo object to control a servo
int steeringAngle;  // variable to store the servo position
/////////////////////////////////////////////////////////////////////////

#define enA 5  //EnableA command line - should be a PWM pin
#define enB 6  //EnableB command line - should be a PWM pin
byte speedSetting_right = 0;  //initial speed = 0
byte speedSetting_lift = 0;  //initial speed = 0

//name the motor control pins - replace the CHANGEME with your pin number, digital pins do not need the 'D' prefix whereas analogue pins need the 'A' prefix
#define INa A0  //Channel A direction
#define INb A1  //Channel A direction
#define INc A2  //Channel B direction
#define INd A3  //Channel B direction


void setup() {
  // put your setup code here, to run once:
   
  Serial.begin(9600);
  
  //I2C comuncation
  Wire.begin(8);                 // join I2C bus with address #8
  Wire.onReceive(receiveEvent);  // register event

    myservo.attach(servoPin);  //attach our servo object to pin D4
  //the Servo library takes care of defining the PinMode declaration (libraries/Servo/src/avr/Servo.cpp line 240)

  //configure the motor control pins as outputs
  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

   Serial.println("Arduino Nano is Running");  //sanity check

  speedSetting_right = 150;
  speedSetting_lift = 150;
  //motors(speedSetting_right, speedSetting_lift);  //make a call to the "motors" function and provide it with a value for each of the 2 motors - can be different for each motor - using same value here for expedience
  //Serial.print("Motor Speeds: ");
  //Serial.println(speedSetting);  
}

void loop() {
  // put your main code here, to run repeatedly:
   motors(speedSetting_lift, speedSetting_right);
  moveSteering(steeringAngle);
  goForwards()
    

}


void motors(int leftSpeed, int rightSpeed) {
  //set individual motor speed
  //direction is set separately

  analogWrite(enA, leftSpeed);
  analogWrite(enB, rightSpeed);
}


void moveSteering(int servo_angle) {
  //you may need to change the maximum and minimum servo angle to have the largest steering motion
  int maxAngle = 115;
  int minAngle = 65;
  int midAngle = 84;
  myservo.write(servo_angle);

  if (minAngle > servo_angle > maxAngle) {
    Serial.print("servo error");
  }

  void goForwards() {
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
}


void goBackwards() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, HIGH);
  digitalWrite(INc, LOW);
  digitalWrite(INd, HIGH);
}

void stopMotors() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, LOW);
}


void receiveEvent(int howMany) {
  char c;
  while (1 < Wire.available()) { // loop through all but the last
   c = Wire.read(); // receive byte as a character
  Serial.print(c);         // print the character
  }
  if(c=='a')  // recive right motor speed
  {
  speedSetting_right = Wire.read();
  Serial.print("right motor speed :");
  Serial.println(speedSetting_right);
  Serial.print("\n");
  
  }

  else if(c=='b')  // recive right motor speed
  {
  speedSetting_lift = Wire.read();
  Serial.print("lift motor speed :");
  Serial.println(speedSetting_lift);
  Serial.print("\n");
  
  }

  else if(c=='c')  // recive right motor speed
  {
  steeringAngle = Wire.read();
  Serial.print("servo angel :");
  Serial.println(steeringAngle);
  Serial.print("\n");
  
  }  

 
}





