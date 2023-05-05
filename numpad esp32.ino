// esp32 numpad 

/* @file HelloKeypad.pde
|| @version 1.0
|| @author Alexander Brevig
|| @contact alexanderbrevig@gmail.com
||
|| @description
|| | Demonstrates the simplest use of the matrix Keypad library.
|| #
*/
// #include <Keypad.h>

// const byte ROWS = 4; //four rows
// const byte COLS = 3; //three columns
// char keys[ROWS][COLS] = {
//   {'1','2','3'},
//   {'4','5','6'},
//   {'7','8','9'},
//   {'*','0','#'}
// };
// byte rowPins[ROWS] = {12, 11, 10, 9}; //connect to the row pinouts of the keypad
// byte colPins[COLS] = {8, 7, 6}; //connect to the column pinouts of the keypad

// Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

// void setup(){
//   Serial.begin(9600);
// }
  
// void loop(){
//   char key = keypad.getKey();
  
//   if (key){
//     Serial.println(key);
//   }
// }




#include <Keypad.h>
char key;
const byte ROWS = 4; //four rows
const byte COLS = 3; //three columns
int count =0;
// char key[20];
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

byte rowPins[ROWS] = {32,33, 25, 26}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {27, 14, 12}; //connect to the column pinouts of the keypad
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

void setup(){
  Serial.begin(9600);
}
  
void loop(){

  
  
   char key = keypad.getKey();
   if (key){
    
    Serial.println(key);
   }
  
 
  
  
 
}




void function ()
{
  
  if (key='2')
   {
     if(key='2')
     {
        Serial.print("go forward 10 cm");     
     }      
   }  

}
