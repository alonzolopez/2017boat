/* ROS node for Arduino motor controller
 * and sensor readings
 * Publishes sensor data from IMU and sonar sensors
 * and subscribes to motor commands
 * and actuates the motors
 */
 
 #include <ros.h>
 #include <std_msgs/Float32MultiArray.h>
 #include <Wire.h>
 #include <Adafruit_Sensor.h>
 #include <Adafruit_BNO055.h>
 #include <utility/imumaths.h>
 
 Adafruit_BNO055 bno = Adafruit_BNO055();
 float x, y, z;
 
 ros::NodeHandle nh;
 std_msgs::Float32MultiArray sensor_arr_msg;
 ros::Publisher pub("sensor_data", &sensor_arr_msg);
 //std_msgs::String str_msg;
 //ros::Publisher chatter("chatter",&str_msg);
 
 
 void setup(void)
 {
   // ROS initialization
   nh.initNode();
   sensor_arr_msg.data_length = 3;
   
   nh.advertise(pub);
   //nh.advertise(chatter);
   
   /* Initialise the sensor */
  if(!bno.begin())
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
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
   x = euler.x();
   y = euler.y();
   z = euler.z();
   
   sensor_arr_msg.data[0] = x;
   sensor_arr_msg.data[1] = y;
   sensor_arr_msg.data[2] = z;
   pub.publish(&sensor_arr_msg);
   
   
   nh.spinOnce();
   delay(100);
 }
