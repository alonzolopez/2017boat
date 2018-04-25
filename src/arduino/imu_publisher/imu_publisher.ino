/* ROS node for Arduino motor controller
 * and sensor readings
 * Publishes sensor data from IMU and sonar sensors
 * and subscribes to motor commands
 * and actuates the motors
 */
 
 #include <ros.h>
 #include <std_msgs/Float32MultiArray.h>
 #include <std_msgs/Int16MultiArray.h>
 #include <sensor_msgs/Imu.h>
 #include <Wire.h>
 #include <Adafruit_Sensor.h>
 #include <Adafruit_BNO055.h>
 #include <utility/imumaths.h>
 
 Adafruit_BNO055 bno = Adafruit_BNO055();
 float x, y, z;
 
 ros::NodeHandle nh;
 //std_msgs::Float32MultiArray sensor_arr_msg;
 //std_msgs::Int16MultiArray sensor_arr_msg;
 sensor_msgs::Imu sensor_arr_msg;
 ros::Publisher pub("sensor_data", &sensor_arr_msg);
 //std_msgs::String str_msg;
 //ros::Publisher chatter("chatter",&str_msg);
 
 
 void setup(void)
 {
   // ROS initialization
   nh.initNode();
   //sensor_arr_msg.data_length = 3;
   
   nh.advertise(pub);
   //nh.advertise(chatter);
   
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
   //imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
   
   sensor_arr_msg.orientation.x = euler.x();
   sensor_arr_msg.orientation.y = euler.y();
   sensor_arr_msg.orientation.z = euler.z();   
   sensor_arr_msg.linear_acceleration.x = laccel.x();
   sensor_arr_msg.linear_acceleration.y = laccel.y();
   sensor_arr_msg.linear_acceleration.z = laccel.z();

   pub.publish(&sensor_arr_msg);
   
   
   nh.spinOnce();
   delay(10);
 }
