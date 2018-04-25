/* ROS node for Arduino motor controller
 * and sensor readings
 * Publishes sensor data from IMU and sonar sensors
 * and subscribes to motor commands
 * and actuates the motors
 */
 
 #include <ros.h>
 #include <sensor_msgs/Imu.h>
 #include <geometry_msgs/Twist.h>
 #include <Wire.h>
 #include <Adafruit_Sensor.h>
 #include <Adafruit_BNO055.h>
 #include <utility/imumaths.h>
 
 Adafruit_BNO055 bno = Adafruit_BNO055();
 float x, y, z;
 
 int lmotor, rmotor;
 
 ros::NodeHandle nh;
 
 void callback(const geometry_msgs::Twist &msg)
 {
   lmotor = msg.angular.x;
   rmotor = msg.angular.y;
 }
 
 sensor_msgs::Imu sensor_arr_msg;
 ros::Publisher pub("sensor_data", &sensor_arr_msg);
 ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &callback);
 

 
 
 void setup(void)
 {
   // ROS initialization
   nh.initNode();
   
   nh.advertise(pub);
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
   sensor_arr_msg.orientation.w = lmotor;
   sensor_arr_msg.linear_acceleration.x = laccel.x();
   sensor_arr_msg.linear_acceleration.y = laccel.y();
   sensor_arr_msg.linear_acceleration.z = laccel.z();

   pub.publish(&sensor_arr_msg);
   
   
   nh.spinOnce();
   delay(10);
 }
