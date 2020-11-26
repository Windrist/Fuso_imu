#include "MPU9250.h"
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>

MPU9250 mpu;
ros::NodeHandle nh;

sensor_msgs::Imu imu;
ros::Publisher imuPub("/fuso/imu", &imu);

void setup()
{
    Serial.begin(57600);

    Wire.begin();

    delay(2000);
    mpu.setup();
    
    nh.initNode();
    nh.advertise(imuPub);
}

void loop()
{
    static uint32_t prev_ms = millis();
    if ((millis() - prev_ms) > 16)
    {
        mpu.update();
        
        imu.header.stamp = nh.now();
        imu.header.frame_id = "imu_link";

        imu.orientation.x = mpu.getQuaternionX();
        imu.orientation.y = mpu.getQuaternionY();
        imu.orientation.z = mpu.getQuaternionZ();
        imu.orientation.w = mpu.getQuaternionW();

        imu.angular_velocity.x = mpu.getGyroX();
        imu.angular_velocity.y = mpu.getGyroY();
        imu.angular_velocity.z = mpu.getGyroZ();
      
        imu.linear_acceleration.x = mpu.getAccX();
        imu.linear_acceleration.y = mpu.getAccY();
        imu.linear_acceleration.z = mpu.getAccZ();

        imuPub.publish(&imu);
        nh.spinOnce();
        
        prev_ms = millis();
    }
}
