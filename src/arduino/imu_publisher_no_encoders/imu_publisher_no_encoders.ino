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
 #include <Servo.h>
 
 // sonar pin and variable setup code
 #define PING_PINL A0
 #define PING_PINC A1
 #define PING_PINR A2

 // IMU setup code
 Adafruit_BNO055 bno = Adafruit_BNO055();
 float x, y, z;

 // motor control setup code
 int lcmd, rcmd, servocmd;
 int lpwm, rpwm, servopwm;
 const int maxmotorspd = 70;
 const int lmotorpin = 5;
 const int rmotorpin = 6;
 const int servomotorpin = 7;
 Servo servomotor; 
 
 ros::NodeHandle nh;
 
 void callback(const geometry_msgs::Twist &msg)
 {
   lcmd = msg.angular.x;
   rcmd = msg.angular.y;
   servocmd = msg.angular.z;

   lpwm = map(lcmd, 0, maxmotorspd, 0,255);
   lpwm = constrain(lpwm, 0, 255);
   rpwm = map(rcmd, 0, maxmotorspd, 0,255);
   rpwm = constrain(rpwm, 0, 255);
   servopwm = map(servocmd, -90,90, 0, 180);
   servopwm = 180; //servopwm + 45;
   //servopwm = constrain(servopwm, 0, 180);
   analogWrite(lmotorpin, lpwm);
   analogWrite(rmotorpin, rpwm);
   //servomotor.write(servopwm);
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

  // set up motor pwm output pins
  pinMode(lmotorpin, OUTPUT);
  pinMode(rmotorpin, OUTPUT);
  servomotor.attach(servomotorpin);
   
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
    // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
   imu::Vector<3> laccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
   
   sensor_arr_msg.orientation.x = euler.x();
   sensor_arr_msg.orientation.y = euler.y();
   sensor_arr_msg.orientation.z = euler.z();  
   sensor_arr_msg.orientation.w = millis();
   sensor_arr_msg.linear_acceleration.x = laccel.x();
   sensor_arr_msg.linear_acceleration.y = laccel.y();
   sensor_arr_msg.linear_acceleration.z = laccel.z();

   pub.publish(&sensor_arr_msg);

   servomotor.write(180);
   
   // Gather and pub range data
   rangemsgL.data = lpwm; //analogRead(PING_PINL)/2;
   rangemsgC.data = rpwm; //analogRead(PING_PINC)/2;
   rangemsgR.data = servopwm; //analogRead(PING_PINR)/2;
   rangepubL.publish(&rangemsgL);
   rangepubC.publish(&rangemsgC);
   rangepubR.publish(&rangemsgR);

   // Pub time
   millisecsmsg.data = millis();
   millisecspub.publish(&millisecsmsg);
   
   
   
   nh.spinOnce();
   delay(20);
 }
