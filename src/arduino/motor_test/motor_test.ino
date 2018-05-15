
 const int lmotorpin = 5;
 const int rmotorpin = 6;
 const int servomotorpin = 7;
 
 int pwmsig = 150;
 int servopwm = 155;
 
 void setup() {
  // put your setup code here, to run once:
  pinMode(lmotorpin, OUTPUT);
  pinMode(rmotorpin, OUTPUT);
  pinMode(servomotorpin, OUTPUT);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(lmotorpin, pwmsig);
  analogWrite(rmotorpin, pwmsig);
  analogWrite(servomotorpin, servopwm);
  Serial.println(pwmsig);
  delay(100);

}
