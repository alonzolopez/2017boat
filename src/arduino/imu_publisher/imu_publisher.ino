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
 #include <NewPing.h>

 // sonar pin and variable setup code
 #define PING_PINL 10
 #define PING_PINC 11
 #define PING_PINR 12
 #define MAX_DISTANCE 200
 //int range_analog = 0;
 //int range_inches = 0;
 NewPing sonarL(PING_PINL, PING_PINL, MAX_DISTANCE);
 NewPing sonarC(PING_PINC, PING_PINC, MAX_DISTANCE);
 NewPing sonarR(PING_PINR, PING_PINR, MAX_DISTANCE);

 // IMU setup code
 Adafruit_BNO055 bno = Adafruit_BNO055();
 float x, y, z;

 // motor control setup code
 int lcmd, rcmd;
 const int maxmotorspd = 200;
 const int lmotorpin = 2;
 const int rmotorpin = 3;


 
 ros::NodeHandle nh;
 
 void callback(const geometry_msgs::Twist &msg)
 {
   lcmd = msg.angular.x;
   rcmd = msg.angular.y;
   lcmd = map(lcmd, 0,maxmotorspd, 0, 255);
   rcmd = map(rcmd, 0,maxmotorspd, 0, 255);
   lcmd = constrain(lcmd, 0, 255);
   rcmd = constrain(rcmd, 0, 255);
   analogWrite(lmotorpin, lcmd);
   analogWrite(rmotorpin, rcmd);
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
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
   imu::Vector<3> laccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
   
   sensor_arr_msg.orientation.x = euler.x();
   sensor_arr_msg.orientation.y = euler.y();
   sensor_arr_msg.orientation.z = euler.z();  
   sensor_arr_msg.orientation.w = lcmd;
   sensor_arr_msg.linear_acceleration.x = laccel.x();
   sensor_arr_msg.linear_acceleration.y = laccel.y();
   sensor_arr_msg.linear_acceleration.z = laccel.z();

   pub.publish(&sensor_arr_msg);
   
   /*range_analog = analogRead(A0);
   range_inches = map(range_analog, 0, 1023, 0, 5000);
   range_inches = range_inches/6.4*2;
   rangemsg.data = range_inches;
   rangepub.publish(&rangemsg);*/
   rangemsgL.data = sonarL.ping_cm();
   rangemsgC.data = sonarC.ping_cm();
   rangemsgR.data = sonarR.ping_cm();
   rangepubL.publish(&rangemsgL);
   rangepubC.publish(&rangemsgC);
   rangepubR.publish(&rangemsgR);
   
   millisecsmsg.data = millis();
   millisecspub.publish(&millisecsmsg);
   
   
   nh.spinOnce();
   delay(20);
 }
