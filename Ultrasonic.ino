



int echo_pin = 26;
int trig_pin = 25;

long duration; 
float distance;

void setup() {
  // put your setup code here, to run once:
 pinMode(trig_pin, OUTPUT);
 pinMode(echo_pin, INPUT);
 Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(5);

  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  
  pinMode(echo_pin, INPUT);
  duration = pulseIn(echo_pin, HIGH);   // PulseIN used to measure the time for High or low signal 
  distance = duration*0.034/2;

  Serial.print("distance in Cm: ");
  Serial.println(distance);
  delay(200);

  
}
