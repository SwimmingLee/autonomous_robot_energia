#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <IMU.h>


ros::NodeHandle nh;

sensor_msgs::Imu  imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;

cIMU imu;

//fast inverse square root
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  const float threehalfs = 1.5f;

  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (threehalfs - (half * y * y));

  return y;
}


void setup() {
  // put your setup code here, to run once:
  nh.initMode();
  nh.advertise(imu_pub);
  tfbroadcaster.init(nh);

  imu.begin();
}

void loop() {
  // put your main code here, to run repeatedly: 
  static uint32_t pre_time;

  imu.update();

  if(millis() - pre_time >= 50)
  {
    pre_time = millis();

    imu_msg.header.stamp - nh.now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.angular_velocity.x = imu.gyroData[0];
    imu_msg.angular_velocity.y = imu.gyroData[1];
    imu_msg.angular_velocity.z = imu.gyroData[2];
    imu_msg.angular_velocity_convariance[0] = 0.02;
    imu_msg.angular_velocity_convariance[1] = 0;
    imu_msg.angular_velocity_convariance[2] = 0;
    imu_msg.angular_velocity_convariance[3] = 0;
    imu_msg.angular_velocity_convariance[4] = 0.02;
    imu_msg.angular_velocity_convariance[5] = 0;
    imu_msg.angular_velocity_convariance[6] = 0;
    imu_msg.angular_velocity_convariance[7] = 0;
    imu_msg.angular_velocity_convariance[8] = 0.02;

    imu_msg.linear_acceleration.x = imu.accData[0];
    imu_msg.linear_acceleration.y = imu.accData[1];
    imu_msg.linear_acceleration.z = imu.accData[2];
    imu_msg.linear_acceleration_convariance[0] = 0.04;
    imu_msg.linear_acceleration_convariance[1] = 0;
    imu_msg.linear_acceleration_convariance[2] = 0;
    imu_msg.linear_acceleration_convariance[3] = 0;
    imu_msg.linear_acceleration_convariance[4] = 0.04;
    imu_msg.linear_acceleration_convariance[5] = 0;
    imu_msg.linear_acceleration_convariance[6] = 0;
    imu_msg.linear_acceleration_convariance[7] = 0;
    imu_msg.linear_acceleration_convariance[8] = 0.04;

    imu_msg.orientation.w = imu.quat[0];
    imu_msg.orientation.x = imu.quat[1];
    imu_msg.orientation.y = imu.quat[2];
    imu_msg.orientation.z = imu.quat[3];

    imu_msg.orientation_covariance[0] = 0.0025;
    imu_msg.orientation_covariance[1] = 0;
    imu_msg.orientation_covariance[2] = 0;
    imu_msg.orientation_covariance[3] = 0;
    imu_msg.orientation_covariance[4] = 0.0025;
    imu_msg.orientation_covariance[5] = 0;
    imu_msg.orientation_covariance[6] = 0;
    imu_msg.orientation_covariance[7] = 0;
    imu_msg.orientation_covariance[8] = 0.0025;

    imu_pub.publish(&imu_msg);

    tfs_msg.header.stamp = nh.now();
    tfs_msg.header.frame_id = "base_link";
    tfs_msg.child_frame_id = "imu_link";
    tfs_msg.transform.rotation.w = imu.quat[0];
    tfs_msg.transform.rotation.x = imu.quat[1];
    tfs_msg.transform.rotation.y = imu.quat[2];
    tfs_msg.transform.rotation.z = imu.quat[3];

    tfs_msg.transform.translation.x = 0.0;
    tfs_msg.transform.translation.y = 0.0;
    tfs_msg.transform.translation.z = 0.0;

    tfbroadcaster.sendTransform(tfs_msg);
  }

  nh.spinOnce();
}
