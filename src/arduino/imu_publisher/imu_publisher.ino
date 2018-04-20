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
 std_msgs::Float32MultiArray imu_arr;
 ros::Publisher imu("imu", &imu_arr);
 
 
 void setup(void)
 {
   // ROS initialization
   nh.initNode();
   nh.advertise(imu_arr);
   
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
