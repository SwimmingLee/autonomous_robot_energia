#include "Wire.h"
#include "ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"

const int MPU6050_addr = 0x68;
volatile int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int cnt = 0;

ros::NodeHandle nh;
std_msgs::Int16MultiArray imu_msg;
ros::Publisher pub_imu("imu", &imu_msg);

void initMPU6050()
{
  Wire.begin();
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);       
}

void readAccelGyro()
{
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr, 14, true);
  AcX = (Wire.read() << 8) | (Wire.read());
  AcY = (Wire.read() << 8) | (Wire.read());
  AcZ = (Wire.read() << 8) | (Wire.read());
  Tmp = (Wire.read() << 8) | (Wire.read());
  GyX = (Wire.read() << 8) | (Wire.read());
  GyY = (Wire.read() << 8) | (Wire.read());
  GyZ = (Wire.read() << 8) | (Wire.read());
}

void setup() {
  cnt = 0;
  // put your setup code here, to run once:
  nh.initNode();
  imu_msg.data_length = 6;
  imu_msg.data = new int16_t[6];
  //imu_msg.layout.dim_length = 1;
  //imu_msg.layout.dim = (std_msgs::MultiArrayDimension *)malloc(sizeof(std_msgs::MultiArrayDimension));
  //imu_msg.layout.dim[0].label = "imu";
  //imu_msg.layout.dim[0].size = 6;
  //imu_msg.layout.dim[0].stride = 1 * 6;
  //imu_msg.layout.data_offset = 0;

  //imu_msg.data_length = 6;
  //imu_msg.data = (int16_t *)malloc(sizeof(int16_t)*6);
  
  nh.advertise(pub_imu);
  
  initMPU6050();
}

void loop() {
  // put your main code here, to run repeatedly: 

  readAccelGyro();
  imu_msg.data[0] = AcX;
  imu_msg.data[1] = AcY;
  imu_msg.data[2] = AcZ;
  imu_msg.data[3] = GyX;
  imu_msg.data[4] = GyY;
  imu_msg.data[5] = GyZ;

  pub_imu.publish(&imu_msg);
  
  nh.spinOnce();
}
