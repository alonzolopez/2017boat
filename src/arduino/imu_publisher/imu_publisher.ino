/* ROS node for Arduino motor controller
 * and sensor readings
 * Publishes sensor data from IMU and sonar sensors
 * and subscribes to motor commands
 * and actuates the motors
 */
 
 /*  PINOUT!!!!
  *   
  *  SonarL:
  *  Short between Trig & Echo
  *  Trig & Echo - Digital 10
  *  Vcc - 5V
  *  SonarC:
  *  Short between Trig & Echo
  *  Trig & Echo - Digital 11
  *  Vcc - 5V
  *  SonarR:
  *  Short between Trig & Echo
  *  Trig & Echo - Digital 12
  *  Vcc - 5V
  *  
  *  IMU:
  *  Vin - 3.3V
  *  SDA - SDA
  *  SCL - SCL
 */
 #include <ros.h>
 #include <sensor_msgs/Imu.h>
 #include <std_msgs/Int32.h>
 #include <geometry_msgs/Twist.h>
 #include <Wire.h>
 #include <Adafruit_Sensor.h>
 #include <Adafruit_BNO055.h>
 #include <utility/imumaths.h>
 
 // sonar pin and variable setup code
 #define PING_PINL A0
 #define PING_PINC A1
 #define PING_PINR A2

 // IMU setup code
 Adafruit_BNO055 bno = Adafruit_BNO055();
 float x, y, z;

 // motor control setup code
 int lcmd, rcmd, servocmd;
 const int maxmotorspd = 200;
 const int lmotorpin = 5;
 const int rmotorpin = 6;
 const int servomotorpin = 7;

 // encoder setup
 const int leftEncoderPinA = 2;
 const int leftEncoderPinB = 3;
 const int rightEncoderPinA = 18;
 const int rightEncoderPinB = 19;
 volatile unsigned int lcounter = 0;
 volatile unsigned int rcounter = 0;
 volatile long timelastrevleft = millis();
 volatile long timelastrevright = millis();
 volatile float leftomega = 0;
 volatile float rightomega = 0;
 volatile int lefterror = 0;
 volatile int righterror = 0;
 
 ros::NodeHandle nh;
 
 void callback(const geometry_msgs::Twist &msg)
 {
   lcmd = msg.angular.x;
   rcmd = msg.angular.y;
   servocmd = msg.angular.z;
   lcmd = map(lcmd, 0,maxmotorspd, 0, 255);
   rcmd = map(rcmd, 0,maxmotorspd, 0, 255);
   servocmd = map(servocmd, -90,90, 0, 255);
   lcmd = constrain(lcmd, 0, 255);
   rcmd = constrain(rcmd, 0, 255);
   servocmd = constrain(servocmd, 0, 255);
   analogWrite(lmotorpin, lcmd);
   analogWrite(rmotorpin, rcmd);
   analogWrite(servomotorpin, servocmd);
   nh.spinOnce();
 }
 
 sensor_msgs::Imu sensor_arr_msg;
 std_msgs::Int32 rangemsgL;
 std_msgs::Int32 rangemsgC;
 std_msgs::Int32 rangemsgR;
 std_msgs::Int32 millisecsmsg;
 ros::Publisher pub("sensor_data", &sensor_arr_msg);
 ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &callback);
 ros::Publisher rangepubL("rangeL", &rangemsgL);
 ros::Publisher rangepubC("rangeC", &rangemsgC);
 ros::Publisher rangepubR("rangeR", &rangemsgR);
 ros::Publisher millisecspub("millisecs", &millisecsmsg);

 
 
 void setup(void)
 {
   // ROS initialization
   nh.initNode();
   
   nh.advertise(pub);
   nh.advertise(rangepubL);
   nh.advertise(rangepubC);
   nh.advertise(rangepubR);
   nh.advertise(millisecspub);
   nh.subscribe(sub);

   // Initialize encoder pinouts and interrupts
  pinMode(leftEncoderPinA, INPUT);           // set pin to input
  pinMode(leftEncoderPinB, INPUT);           // set pin to input
  pinMode(rightEncoderPinA, INPUT);           // set pin to input
  pinMode(rightEncoderPinB, INPUT);           // set pin to input
  
  digitalWrite(leftEncoderPinA, HIGH);       // turn on pullup resistors
  digitalWrite(leftEncoderPinB, HIGH);       // turn on pullup resistors
  digitalWrite(rightEncoderPinA, HIGH);       // turn on pullup resistors
  digitalWrite(rightEncoderPinB, HIGH);       // turn on pullup resistors

  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), leftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), rightEncoder, RISING);
   
   /* Initialise the sensor */
  if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_COMPASS))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
  
  bno.setExtCrystalUse(true);
   
 }
 
 void loop(void)
 {
   nh.spinOnce();

   // Gather and pub imu data
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
   imu::Vector<3> laccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
   
   sensor_arr_msg.orientation.x = euler.x();
   sensor_arr_msg.orientation.y = euler.y();
   sensor_arr_msg.orientation.z = euler.z();  
   sensor_arr_msg.orientation.w = millis();
   sensor_arr_msg.linear_acceleration.x = laccel.x();
   sensor_arr_msg.linear_acceleration.y = laccel.y();
   sensor_arr_msg.linear_acceleration.z = laccel.z();
   sensor_arr_msg.angular_velocity.x = leftomega; //encoder rotational vel
   sensor_arr_msg.angular_velocity.y = rightomega; //encoder rotational vel

   pub.publish(&sensor_arr_msg);
   
   // Gather and pub range data
   rangemsgL.data = analogRead(PING_PINL)/2;
   rangemsgC.data = analogRead(PING_PINC)/2;
   rangemsgR.data = analogRead(PING_PINR)/2;
   rangepubL.publish(&rangemsgL);
   rangepubC.publish(&rangemsgC);
   rangepubR.publish(&rangemsgR);

   // Pub time
   millisecsmsg.data = millis();
   millisecspub.publish(&millisecsmsg);
   
   
   nh.spinOnce();
   delay(20);
 }

void leftEncoder() {
  // leftEncoder is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(leftEncoderPinB)==LOW) {
    lcounter++;
  }else{
    lcounter--;
  }
  if(lcounter%200==0){
    leftomega = 1.0*1000/((millis() - timelastrevleft));
    lefterror = lcmd - leftomega;
    timelastrevleft = millis();
  }
}

void rightEncoder() {
  // rightEncoder is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(rightEncoderPinB)==LOW) {
    rcounter++;
  }else{
    rcounter--;
  }
  if(rcounter%200==0){
    rightomega = 1.0*1000/((millis() - timelastrevright));
    righterror = rcmd - rightomega;
    timelastrevright = millis();
  }
}
