/* Rotary encoder with attachInterrupt
Counts pulses from an incremental encoder and put the result in variable counter. 
Taking also into account the direction and counts down when the rotor rotates in 
the other direction.
This code is used attachInterrupt 0 and 1 which are pins 2 and 3 moust Arduino.
For more information about attachInterrupt see:
http://arduino.cc/en/Reference/AttachInterrupt
 
created 2014
by Ben-Tommy Eriksen
https://github.com/BenTommyE/BenRotaryEncoder
 
*/

// Encoder connect to digitalpin 2 and 3 on the Arduino.
#define ENCODERA 2
#define ENCODERB 4

volatile unsigned int counter = 0;  //This variable will increase or decrease depending on the rotation of encoder
long timelastrev = millis();
float omega = 0;
//long timesecondlastrev = 0;

void setup() {
  Serial.begin (9600);
 
  pinMode(ENCODERA, INPUT);           // set pin to input
  pinMode(ENCODERB, INPUT);           // set pin to input
  
  digitalWrite(ENCODERA, HIGH);       // turn on pullup resistors
  digitalWrite(ENCODERB, HIGH);       // turn on pullup resistors
 
 
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);
  
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  //attachInterrupt(1, ai1, RISING);
}

void loop() {
  // Send the value of counter
  //Serial.println ("counter: " + String(counter) + "revolutions: " + String(counter/200));
  delay(10);
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(ENCODERB)==LOW) {
    counter++;
  }else{
    counter--;
  }
  if(counter%200==0){
    omega = 1.0*1000/((millis() - timelastrev));
    Serial.println("w: " + String(omega) + "; time: " + String(millis()) + "; last time: " + String(timelastrev));
    timelastrev = millis();
  }
}

