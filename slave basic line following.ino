#include "Wire.h"
#include "math.h"
#include <Servo.h>  //include the servo library


/////////////////////////////////////////////////////////////
#define servoPin 4
Servo myservo;        // create servo object to control a servo
float steeringAngle;  // variable to store the servo position
/////////////////////////////////////////////////////////////////////////
//name the motor control pins - replace the CHANGEME with your pin number, digital pins do not need the 'D' prefix whereas analogue pins need the 'A' prefix
#define INa A0  //Channel A direction
#define INb A1  //Channel A direction
#define INc A2  //Channel B direction
#define INd A3  //Channel B direction
#
// ASSUMPTION: Channel A is LEFT, Channel B is RIGHT
#define enA 5  //EnableA command line - should be a PWM pin
#define enB 6  //EnableB command line - should be a PWM pin
byte speedSetting = 0;  //initial speed = 0

// photodiode value in byte range from 0-255
int sensor2; // the right sensor 
int sensor3;  // the lift sensor
//////////////////////////////////////////////////////////

int white_value_sensor_2;  // the max value the sensor return in range 0-255 when it see white 
int black_value_sensor_2;  // the min value the sensor return in range 0-255 when it see black

int white_value_sensor_3;  // the max value the sensor return in range 0-255 when it see white 
int black_value_sensor_3;  // the min value the sensor return in range 0-255 when it see black

// two different variable becouse the two sensors might not give same values


void setup() {

  Serial.begin(9600);
  // put your setup code here, to run once:
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

  speedSetting = 150;
  motors(speedSetting, speedSetting);  //make a call to the "motors" function and provide it with a value for each of the 2 motors - can be different for each motor - using same value here for expedience
  Serial.print("Motor Speeds: ");
  Serial.println(speedSetting);

}

void loop() {
  // put your main code here, to run repeatedly:

  goForwards();
  if((sensor2 <= white_value_sensor_2)&&(sensor3 <= white_value_sensor_3)){ // if both sensor see white survice the bot go forword.
    
    moveSteering(84);
    
  }

  else if ((sensor2 >= black_value_sensor_2)&&(sensor3 <= white_value_sensor_3)){ // if right sensor see black and lift sensor see white, turn the Bot right
  moveSteering(65);
  } 
  else if ((sensor2 <= white_value_sensor_2)&&(sensor3 >= black_value_sensor_3)){ // if right sensor see white and lift sensor see black, turn the Bot lift
    
  moveSteering(115);
  }    

}


void receiveEvent(int howMany) {
  char c;
  while (1 < Wire.available()) { // loop through all but the last
   c = Wire.read(); // receive byte as a character
  Serial.print(c);         // print the character
  }
  if(c=='b')
  {
  sensor2 = Wire.read();
  Serial.print("sensor2 read :");
  Serial.println(sensor2 );
  Serial.print("\n");
  
  }

  else if(c=='c')
  {
  sensor3 = Wire.read();
  Serial.print("sensor3 read :");
  Serial.println(sensor3);
  Serial.print("\n");
  
  }  

}




































//#include <Arduino_BuiltIn.h>
#include <MPU6050_light.h>  // to read raw angle



///////////////////////////////////////////////////////////////
MPU6050 mpu(Wire);  /// MPU object
int yaw_angle;

/////////////////////////////////////////////////////////////
#define servoPin 4
Servo myservo;        // create servo object to control a servo
float steeringAngle;  // variable to store the servo position
/////////////////////////////////////////////////////////////////////////

// ASSUMPTION: Channel A is LEFT, Channel B is RIGHT

#define enA 5  //EnableA command line - should be a PWM pin
#define enB 6  //EnableB command line - should be a PWM pin
byte speedSetting = 0;  //initial speed = 0

//name the motor control pins - replace the CHANGEME with your pin number, digital pins do not need the 'D' prefix whereas analogue pins need the 'A' prefix
#define INa A0  //Channel A direction
#define INb A1  //Channel A direction
#define INc A2  //Channel B direction
#define INd A3  //Channel B direction

// encoder two input pins
int pinA = 2;  // Our first hardware interrupt pin is digital pin 2
int pinB = 3;  // Our second hardware interrupt pin is digital pin 3

int distance_ultrasonic;  // to store the ultrasonic data which send from master esp32


void setup() {


  //pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  //pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  // attachInterrupt(0,PinA,RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  //  attachInterrupt(1,PinB,RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)
  Serial.begin(9600);
  // put your setup code here, to run once:
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //I2C comuncation
  Wire.begin(8);                 // join I2C bus with address #8
  Wire.onReceive(receiveEvent);  // register event

  /////////////////////////////////////////////////////////////////////////////////////////////////
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}  // stop everything if could not connect to MPU6050
 ////////////////////////////////////////////////////////////////////////////////////


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

  speedSetting = 150;
  motors(speedSetting, speedSetting);  //make a call to the "motors" function and provide it with a value for each of the 2 motors - can be different for each motor - using same value here for expedience
  Serial.print("Motor Speeds: ");
  Serial.println(speedSetting);

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets();  // gyro and accelero
  Serial.println("Done!\n");
}


void read_yaw() {


  //if((millis()-timer)>10){ // print data every 10ms

  mpu.update();
  // Serial.print("X : ");
  // Serial.print(mpu.getAngleX());
  // Serial.print("\tY : ");
  // Serial.print(mpu.getAngleY());
  // Serial.print("\tf : ");
  // Serial.print(floor(mpu.getAngleZ()));

  // Serial.print("\tc : ");
  // Serial.print(ceil(mpu.getAngleZ()));

  // Serial.print("\tZ : ");
  // Serial.println(mpu.getAngleZ());
  delay(200);
  yaw_angle = floor(mpu.getAngleZ());
  Serial.print(yaw_angle);
  Serial.print("\n");


  //timer = millis();
  //}

  // Serial.print("Apin");
}


void loop() {

  //stop_and_calibrate_MPU(500);
  read_yaw();
  goForwards_MPU(90);
  goBackwards_Ultrasonci(20); 
  //stop_and_calibrate_MPU(200);  
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


  //  for (steeringAngle = minAngle; steeringAngle <= maxAngle; steeringAngle += 1) { //goes from minAngle to maxAngle (degrees)
  //    //in steps of 1 degree
  //    myservo.write(steeringAngle);   //tell servo to go to position in variable 'steeringAngle'
  //    delay(15);                      //waits 15ms for the servo to reach the position
  //  }
  //  for (steeringAngle = maxAngle; steeringAngle >= minAngle; steeringAngle -= 1) { // goes from maxAngle to minAngle (degrees)
  //  /     myservo.write(steeringAngle);   //tell servo to go to position in variable 'steeringAngle'
  //    delay(15);                      //waits 15 ms for the servo to reach the position
  //  }
}


//for each of the below function, two of the 'IN' variables must be HIGH, and two LOW in order to move the wheels - use a trial and error approach to determine the correct combination for your EEEBot
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

// void goClockwise() {
//   digitalWrite(INa, CHANGEME);
//   digitalWrite(INb, CHANGEME);
//   digitalWrite(INc, CHANGEME);
//   digitalWrite(INd, CHANGEME);
// }

// void goAntiClockwise() {
//   digitalWrite(INa, CHANGEME);
//   digitalWrite(INb, CHANGEME);
//   digitalWrite(INc, CHANGEME);
//   digitalWrite(INd, CHANGEME);
// }

void stopMotors() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, LOW);
}




void goForwards_MPU_staight(int desire_angle)  // desire angel will be set to zero. 

 {  
  


  while (yaw_angle != desire_angle) {
    
    Serial.print(yaw_angle);
    Serial.print("\n");
    read_yaw();
    
    if (desire_angle > yaw_angle) {

      float dego1 = (desire_angle - yaw_angle) ;
      float dego2 = dego1 / 360 ;
      float dego3 = dego2 * 26 ;
     
      int degreeo = 84 - dego3 ;
      
     moveSteering(degreeo);  




    } else if (desire_angle < yaw_angle) {

      float  dego_11 = (desire_angle - yaw_angle);
      float dego_21 = dego_11/360;
      float dego_31 = dego_21 * 14;
      
      degreeo = dego_31 - 84
      
      moveSteering(degreeo);  
      return;


    } else {

      moveSteering(84);
      
    
    }



    goForwards();

  }
}

void goBackward_MPU(int current_angle, int desire_angle) {





  while (current_angle != desire_angle) {
    read_yaw();

    if (desire_angle > current_angle) {

      int degree = (((desire_angle - current_angle) / 360) * 26) + 84;

      moveSteering(degree);  // max angle


    } else if (desire_angle < current_angle) {

      int degree = 84 - (((current_angle - desire_angle) / 360) * 14);

      moveSteering(degree);  // max angle



    } else {

      moveSteering(84);
    }

    goBackwards();
  }
}

// void goBackwards_Ultrasonci_MPU(int stop_distance, int distance_obstical, int current_angle, int desire_angle) {

//   if (distance_obstical > stop_distance) {
//     goBackward_MPU(current_angle, desire_angle);
//   } else {
//     stopMotors();
//     delay(5000);    
//   }
// }

void goBackwards_Ultrasonci(int stop_distance) {

  while (distance_ultrasonic > stop_distance || distance_ultrasonic == 0) {
    // goBackward_MPU(current_angle, desire_angle);
    goBackwards();

  } 
    stopMotors();
    //delay(5000);    
  }



void calibrate_MPU() {
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets();  // gyro and accelero
  Serial.println("Done!\n");
}

void stop_and_calibrate_MPU(int time) {
  stopMotors();
  calibrate_MPU();

  delay(time);
}


// void read_yaw_angle()
// {



//   if((millis()-timer)>10){ // print data every 10ms

//    mpu.update();
// 	// Serial.print("X : ");
// 	// Serial.print(mpu.getAngleX());
// 	// Serial.print("\tY : ");
// 	// Serial.print(mpu.getAngleY());
//     Serial.print("\tf : ");
// 	Serial.print(floor(mpu.getAngleZ()));

//   Serial.print("\tc : ");
// 	Serial.print(ceil(mpu.getAngleZ()));

// 	Serial.print("\tZ : ");
// 	Serial.println(mpu.getAngleZ());


// 	timer = millis();
//   }
// }

// // function that executes whenever data is received from master
// // this function is registered as an event, see setup()
// void receiveEvent(int howMany) {
//   while (1 < Wire.available()) { // loop through all but the last
//     char c = Wire.read(); // receive byte as a character
//     Serial.print(c);         // print the character
//   }
//   int x = Wire.read();    // receive byte as an integer
//   Serial.println(x);         // print the integer
// }

void receiveEvent(int howMany) {
  char c;
  while (1 < Wire.available()) { // loop through all but the last
   c = Wire.read(); // receive byte as a character
  Serial.print(c);         // print the character
  }
  if(c=='d')
  {
  distance_ultrasonic = Wire.read();
  Serial.print("distance is :");
  Serial.println(distance_ultrasonic);
  Serial.print("\n");
  
  }

}



void goForwards_MPU(int desire_angle) {


  while (yaw_angle != desire_angle) {
    
    Serial.print(yaw_angle);
    Serial.print("\n");
    read_yaw();
    if (desire_angle > yaw_angle) {


      
      int degree = 70 ;   // min angle turn to the lift 
      moveSteering(degree);  




    } else if (desire_angle < yaw_angle) {

      degree = 110; // max angle turn to the right
      moveSteering(degree);  // max angle
    


    } else {

      moveSteering(84);
      //delay(200);
      return;
    }



    goForwards();

  }

