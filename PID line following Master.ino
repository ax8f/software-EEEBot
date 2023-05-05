#include <QTRSensors.h>
#include "Wire.h"

// PID cantroller line following
QTRSensors qtr;

const uint8_t SensorCount = 4; // number of sensor used 
uint16_t sensorValues[SensorCount]; // to store the values of sensors

int centre_value; // can be found by placing the centre of Bot in balck line
int base_speed; // the base motor speed 

//motors speed
int right_motor_speed;
int lift_motor_speed;

// PID constant to cantrol the motors speed
// this values will be set by try and test
float Kp;  //propotional const
float Ki;  //Integral const              
float Kd;  // derivitive const

// PID constant to cantrol the servo angel 
// this values will be set by try and test
float Kp_servo;  //propotional const
float Ki_servo;  //Integral const
float Kd_servo;  // derivitive const

int mid_sevo_angel; // angel that allow Bot to move staright
int servo_angel;




void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  Wire.begin(); // set up as master
  
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A5, A0, A2, A4}, SensorCount); // pins of IR sensors.

  // take about 10 second, need to move sensors accross black line and white backgraoung
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  } 
}

void loop() {
  

  // send the motors speed and servo angel to ardino nano;
  PID(); 
  
  
  Serial.print("right motor speed: ");
  Serial.println(right_motor_speed);
  
  Serial.print("lift motor speed: ");
  Serial.println(lift_motor_speed);
  
  Serial.print("servo angel: ");
  Serial.println(servo_angel);
  
  Wire.beginTransmission(8); // transmit to device adress 8 
  Wire.write('a');        // send char to allow the slave to store the following data to correct variable
  Wire.write(right_motor_speed);   // sends one byte
  Wire.endTransmission();    // stop transmitting

  Wire.beginTransmission(8); // transmit to device adress 8 
  Wire.write('b');        // send char to allow the slave to store the following data to correct variable
  Wire.write(lift_motor_speed);   // sends one byte
  Wire.endTransmission();    // stop transmitting

   Wire.beginTransmission(8); // transmit to device adress 8 
  Wire.write('c');        // send char to allow the slave to store the following data to correct variable
  Wire.write(servo_angel);   // sends one byte
  Wire.endTransmission();    // stop transmitting

}


void PID(){
    
    uint16_t lineposition = qtr.readLineBlack(sensorValues); // calculate the postion of the line respect to the centre
    int current_error = centre_value - lineposition;
    
    int P = current_error;
    int I =  current_error+I;
    int D = current_error-lasterror;

    lasterror = current_error


    int speed_change = P*Kp + I*Ki + D*Kd;   // adjust Kp, Kd, and Ki to allow speed to change with range (0-255)  

     right_motor_speed = base_speed + speed_change;
     lift_motor_speed = base_speed - speed_change;  

    int servo_angel_change = P*Kp_servo + I*Ki_servo + D*Kd_servo;   // adjust Kp, Kd, and Ki to allow speed to change with range (0-255)  
    
    if (error < 0 ){   // turn the bot right

         servo_angel= mid_sevo_angel+servo_angel_change;

    } 

    if (error > 0 ){   // turn the bot lift

         servo_angel= mid_sevo_angel-servo_angel_change;

    }     
  
      

   // check to do not go after limit
    if (right_motor_speed> 255){
      
      right_motor_speed = 255;

    }
    if (lift_motor_speed> 255){
      
      right_motor_speed = 255;
      
    }

    if (right_motor_speed< 0){
      
      right_motor_speed = 0;

    }
    if (lift_motor_speed< 0){
      
      right_motor_speed = 0;
      
    }
   
}



















