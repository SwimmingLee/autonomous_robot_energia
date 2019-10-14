#include <Wire.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/ColorRGBA.h>

//Right Motor
#define INA_1 12
#define PWM_1 PC_5

//Left Motor
#define INA_2 5
#define PWM_2 PC_6

const int MPU6050_addr = 0x68;
volatile int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int cnt = 0;

//ROS NodeHandler
ros::NodeHandle nh;

//IMU data publisher
std_msgs::Int16MultiArray imu_msg;
ros::Publisher pub_imu("imu", &imu_msg);

//TF data publisher
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

void Move_Forword()
{
  digitalWrite(INA_1, HIGH);
  analogWrite(PWM_1, 255);

  digitalWrite(INA_2, LOW);
  analogWrite(PWM_2, 255);
}


void Move_Right()
{
  digitalWrite(INA_1, LOW);
  analogWrite(PWM_1, 0);

  digitalWrite(INA_2, LOW);
  analogWrite(PWM_2, 255);
}


void Move_Left()
{
  digitalWrite(INA_1, HIGH);
  analogWrite(PWM_1, 255);

  digitalWrite(INA_2, HIGH);
  analogWrite(PWM_2, 0);
}


void Stop()
{
  digitalWrite(INA_1, LOW);
  analogWrite(PWM_1, 0);

  digitalWrite(INA_2, HIGH);
  analogWrite(PWM_2, 0);
}


void Move_Backword()
{
  digitalWrite(INA_1, LOW);
  analogWrite(PWM_1, 255);

  digitalWrite(INA_2, HIGH);
  analogWrite(PWM_2, 255);
}


void motor_CB(const std_msgs::Int16& cmd_msg)
{
  //motor speed control function
  switch(cmd_msg.data)
  {
  case 1:
    digitalWrite(RED_LED, HIGH);
    Move_Forword();
    break;
  case 2:
    digitalWrite(RED_LED, LOW);
    Move_Backword();
    break;
  case 3:
    digitalWrite(GREEN_LED, HIGH);
    Move_Right();
    break;
  case 4:
    digitalWrite(GREEN_LED, LOW);
    Move_Left();
    break;
  case 5:
    digitalWrite(BLUE_LED, HIGH);
    Stop();
    break;
  case 6:
    digitalWrite(BLUE_LED, LOW);
    break;
    
  }
}

//Motor Speed data subscriber
ros::Subscriber<std_msgs::Int16> sub_motor("motor", &motor_CB);

void initMotor()
{
  pinMode(INA_1, OUTPUT);
  pinMode(PWM_1, OUTPUT);
  digitalWrite(INA_1, HIGH);
  analogWrite(PWM_1, 0);
  pinMode(INA_2, OUTPUT);
  pinMode(PWM_2, OUTPUT);
  digitalWrite(INA_2, HIGH);
  analogWrite(PWM_2, 0);
}

void initLED()
{
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
}

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
  initLED();
  initMotor();
  nh.initNode();
  imu_msg.data_length = 6;
  imu_msg.data = new int16_t[6];
  
  //nh.advertise(pub_imu);

  nh.subscribe(sub_motor);
  
  broadcaster.init(nh);
  
  //initMPU6050();
 
}

void loop() {
  // put your main code here, to run repeatedly: 

  //readAccelGyro();
  //imu_msg.data[0] = AcX;
  //imu_msg.data[1] = AcY;
  //imu_msg.data[2] = AcZ;
  //imu_msg.data[3] = GyX;
  //imu_msg.data[4] = GyY;
  //imu_msg.data[5] = GyZ;

  //pub_imu.publish(&imu_msg);

  t.header.frame_id = "/odom";
  t.child_frame_id = "/base_link";
  t.transform.translation.x = 1.0;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;
  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);
  
  nh.spinOnce();
}
