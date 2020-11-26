#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define PI 3.141592654

ros::Publisher imu_pub;
sensor_msgs::Imu imu;

float my_yaw = 0, prev_yaw = 0, init_yaw = 1.571;
float roll, pitch, yaw;

float ax, ay, az, gx, gy, gz, mx, my, mz;

void getAccelcallback(const geometry_msgs::Vector3& msg)
{
    ax = msg.x;
    ay = msg.y;
    az = msg.z;
}

void getGyrocallback(const geometry_msgs::Vector3& msg)
{
    gx = msg.x;
    gy = msg.y;
    gz = msg.z;
}

void getMagnocallback(const geometry_msgs::Vector3& msg)
{
    mx = msg.x;
    my = msg.y;
    mz = msg.z;
}

void initialCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
   geometry_msgs::Quaternion odom_quat;
   odom_quat.x = msg->pose.pose.orientation.x;
   odom_quat.y = msg->pose.pose.orientation.y;
   odom_quat.z = msg->pose.pose.orientation.z;
   odom_quat.w = msg->pose.pose.orientation.w;
   init_yaw = tf::getYaw(odom_quat);
   prev_yaw = yaw;
   ROS_ERROR("INIT YAW: %f\n",init_yaw);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "imu_publisher");

    ros::NodeHandle n;
    imu_pub = n.advertise<sensor_msgs::Imu>("/fuso/imu", 50);
    ros::Subscriber subAcc = n.subscribe("/fuso/accel", 1000, getAccelcallback);
    ros::Subscriber subGyr = n.subscribe("/fuso/gyro", 1000, getGyrocallback);
    ros::Subscriber subMag = n.subscribe("/fuso/magno", 1000, getMagnocallback);
    ros::Subscriber sub_initialPose = n.subscribe("/initialpose", 1000, initialCallback);
    ros::Rate r(10);

    while(n.ok())
    {
        tf::Quaternion differential_rotation;
        tf::Quaternion plus_orientation;

        imu.header.stamp = ros::Time::now();
        imu.header.frame_id = "imu_link";
        imu.orientation_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
        imu.angular_velocity_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
        imu.linear_acceleration_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};

        imu.angular_velocity.x = gx;
        imu.angular_velocity.y = gy;
        imu.angular_velocity.z = gz;

        imu.linear_acceleration.x = ax;
        imu.linear_acceleration.y = ay;
        imu.linear_acceleration.z = az;

        roll = (float)atan2(ay, az);
        if ((ay * sin(roll) + az * cos(roll)) == 0)
        {
            if (ax > 0)
                pitch = PI / 2;
            else
                pitch = -PI / 2;
        }
        else
            pitch = (float)atan(-ax / (ay * sin(roll) + az * cos(roll)));
        yaw = (float)atan2(mz * sin(roll) - my * cos(roll), mx * cos(pitch) + my * sin(pitch) * sin(roll) + mz * sin(pitch) * cos(roll));
        // my_yaw = yaw - prev_yaw;
        // if (my_yaw < - PI)
        // my_yaw += 2 * PI;
        // else if (my_yaw > PI)
        // my_yaw -= 2 * PI;
        // my_yaw += init_yaw;
        plus_orientation = tf::createQuaternionFromRPY(roll, pitch, yaw);
        differential_rotation = plus_orientation.normalize();
        // differential_rotation = plus_orientation;

        quaternionTFToMsg(differential_rotation, imu.orientation);
        imu_pub.publish(imu);
        ros::spinOnce();
        r.sleep();
    }
}