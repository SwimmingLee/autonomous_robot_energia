#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Wire.h>

#include "MadgwickAHRS.h"

//motor
#define INA_R PA_4
#define PWM_R PC_5
#define INA_L PE_4
#define PWM_L PC_6
//encoder
#define ENCODER_LEFT_A PC_4
#define ENCODER_LEFT_B PB_3
#define ENCODER_RIGHT_A PF_3
#define ENCODER_RIGHT_B PF_2


volatile long Left_Encoder_Count =0;
volatile long Right_Encoder_Count =0;
volatile bool Left_Encoder_Pulse;
volatile bool Right_Encoder_Pulse;


ros::NodeHandle nh;

sensor_msgs::Imu  imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

std_msgs::Float32MultiArray rpy_msg;
ros::Publisher rpy_pub("rpy", &rpy_msg);

std_msgs::Int64 lwheel_msg;
ros::Publisher lwheel_pub("lwheel", &lwheel_msg);
std_msgs::Int64 rwheel_msg;
ros::Publisher rwheel_pub("rwheel", &rwheel_msg);

geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;

Madgwick filter;
const int MPU6050_addr = 0x68;
volatile int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, MagX, MagY, MagZ;
volatile float ax, ay, az, gx, gy, gz;
volatile float quat[4];
float aRes, gRes, mRes;
float motor_left_speed = 0;
float motor_right_speed = 0;

//
//////////////MOTOR////////////////////////////
void Move_Forword(){ 
  digitalWrite(INA_L, LOW);
  analogWrite(PWM_L, 120);
  digitalWrite(INA_R, HIGH);
  analogWrite(PWM_R, 120); 
}
void Move_Right(){
  digitalWrite(INA_R, LOW);
  analogWrite(PWM_R, 120);
  digitalWrite(INA_L, LOW);
  analogWrite(PWM_L, 120);
}
void Move_Left(){
  digitalWrite(INA_R, HIGH);
  digitalWrite(INA_L, HIGH); 
  analogWrite(PWM_L, 120);
  analogWrite(PWM_R, 120); 
}
void Stop(){
  digitalWrite(INA_R, LOW);
  analogWrite(PWM_R, 0);
  digitalWrite(INA_L, HIGH);
  analogWrite(PWM_L, 0);
}
void Move_Backword(){
  digitalWrite(INA_R, LOW);
  analogWrite(PWM_R, 120);
  digitalWrite(INA_L, HIGH);
  analogWrite(PWM_L, 120);
}
void initMotor(){
  pinMode(INA_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  digitalWrite(INA_R, HIGH);
  analogWrite(PWM_R, 0);
  pinMode(INA_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  digitalWrite(INA_L, LOW);
  analogWrite(PWM_L, 0);
}

void moveRightMotor(float rightServoValue)
{
  
  if(rightServoValue > 0)
  {
    digitalWrite(INA_R, HIGH);
    analogWrite(PWM_R, rightServoValue); 
  }
  else if(rightServoValue < 0)
  {
    digitalWrite(INA_R, LOW);
    analogWrite(PWM_R, abs(rightServoValue));  
  }
  else if(rightServoValue == 0)
  {
    analogWrite(PWM_R, 0);
  }
}

void moveLeftMotor(float leftSerboValue)
{
  
  if(leftSerboValue > 0)
  {
    digitalWrite(INA_L, LOW);
    analogWrite(PWM_L, leftSerboValue);
  }
  else if(leftSerboValue < 0)
  {
    digitalWrite(INA_L, HIGH);
    analogWrite(PWM_L, abs(leftSerboValue)); 
  }
  else if(leftSerboValue == 0)
  {
    analogWrite(PWM_L, 0);
  }
}

////////////ENCODER///////////////////////////////////////
void SetupEncoders(){
  pinMode(ENCODER_LEFT_A,INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B,INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A,INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B,INPUT_PULLUP);  
  attachInterrupt(ENCODER_LEFT_A,Count_Left_Encoder,RISING);
  attachInterrupt(ENCODER_RIGHT_A,Count_Right_Encoder,RISING);
}
void Count_Left_Encoder(){
  Left_Encoder_Pulse = digitalRead(ENCODER_LEFT_B);
  Left_Encoder_Count -= Left_Encoder_Pulse ? -1: +1;   
}
 
void Count_Right_Encoder(){
  Right_Encoder_Pulse = digitalRead(ENCODER_RIGHT_B);
  Right_Encoder_Count += Right_Encoder_Pulse ? -1:+1;
  
}

/////////////////////////////////////////////////////////
void initMPU6050()
{
  Wire.begin();
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);   

  //filter.begin(update_hz);
  //update_hz는 uint32_t type이다. 수정할 것

  aRes = 2.0f / 32768.0f;
  gRes = 250.0f / 32768.0f; 
}


void readAccelGyro()
{
  static uint32_t prev_process_time = micros();
  static uint32_t cur_process_time = 0;
  static uint32_t process_time = 0;

  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr, 14, true);
  AcX = ((Wire.read() << 8) | (Wire.read())) - 350 ;         // separately test - 1500
  AcY = ((Wire.read() << 8) | (Wire.read())) - 450;                 // separately test 
  AcZ = ((Wire.read() << 8) | (Wire.read()));          // separately test + 3000
  Tmp = ((Wire.read() << 8) | (Wire.read()));                 // separately test 
  GyX = ((Wire.read() << 8) | (Wire.read())) - 260;           // separately test - 250
  GyY = ((Wire.read() << 8) | (Wire.read())) - 190;           // separately test + 260
  GyZ = ((Wire.read() << 8) | (Wire.read())) +  80;           // separately test + 110

  cur_process_time = micros();
  process_time     = cur_process_time - prev_process_time;
  prev_process_time = cur_process_time;

  filter.invSampleFreq = (float)process_time / 1000000.0f;

  ax = AcX * aRes;
  ay = AcY * aRes;
  az = AcZ * aRes;

  gx = GyX * gRes;
  gy = GyY * gRes;
  gz = GyZ * gRes;
  
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  quat[0] = filter.q0;
  quat[1] = filter.q1;
  quat[2] = filter.q2;
  quat[3] = filter.q3;

}

/********************************************************************************************
Odometry + TF + JointStates
********************************************************************************************/
/*
void initOdom(void)
{
  init_encoder = true;

  for(int index = 0; index < 3; index++)
  {
    odom_pos[index] = 0.0;
    odom_vel[index] = 0.0;
  }

  odom.pose.pose.posistion.x = 0.0;
  odom.pose.pose.posistion.x = 0.0;/home/lsy/catkin_ws/src/AR/AR_bringup/src/pid_velocity.cpp
  odom.pose.pose.posistion.x = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.x = 0.0;

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.angualr.z = 0.0;
}


void initJointStates(void)
{
  static char *joint_states_name = {"wheel_left_joint", "wheel_right_joint"};

  joint_states.header.frame_id = "base_link";
  joint_states.name = joint_states_name;

  joint_states.name_length = WHEEL_NUM;
  joint_states.postion_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length = WHEEL_NUM;
}

void updateOdometry(void)
{
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::ceateQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];
}

void updateJointStates(void)
{
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

  joint_states_pos[LEFT]  = last_rad[LEFT];
  joint_states_pos[RIGHT] = last_ras[RIGHT];

  joint_states_vel[LEFT]  = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}


void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.transform.translation.x = odom.pose.pose.positoin.x;
  odom_tf.transform.translation.y = odom.pose.pose.positoin.y;
  odom_tf.transform.translation.z = odom.pose.pose.positoin.z;
  odom_tf.transform.roation       = odom.pose.pose.orientation;
}

bool calcOdometry(double diff_time)
{
  float* orientation;
  double wheel_l, wheel_r; // roatation value of wheel [rad]
  double delta_s, theta, delta__theta;

  static double last_theta = 0.0;
  //v = translational velocity [m/s], w = roatational velocityy [rad/s]
  double v, w;
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if(step_time = 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

  if(isnan(wheel_l))
    wheel_l = 0.0;

  if(isnan(wheel_r))
    wheel_r = 0.0;

  delta_s = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  //theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
  odrientation = sensor.getOrientation();
  theta = atan2(orientation[1]*oriention[2] + orientaion[0]*orientation[3], 
                0.5f - orienatation[2]*orientation[2] - orientation[3]*orientation[3]);

  delta_theta = theta - last_theta;

  v = delta_s / step_time;
  w = delta_theta / step_time;

  last_velocity[LEFT]  = wheel_l / step_time;
  last_velocity[RIGHT] = wheel_r / step_time;


  //compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0 ));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0 ));
  odom_pose[2] += delta_theta;

  //compute odometric instantaneuouos velocity
  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_theta = theta;

  return true;
  
}


void updateMotorInfo(int32_t left_ticks, int32_t right_tick)
{
  int32_t current_tick = 0;
  static int32_t last_tick[WHEEL_NUM] = {0.0, 0.0};

  if(init_encoder)
  {
    for(int index = 0; index < WHEEL_NUM; index++)
    {
      last_diff_tick[index] = 0.0;
      last_tick[index] = 0.0;
      last_rad[index] = 0.0;
      last_velocity[index]= 0.0;
 
    }

    last_tick[LEFT]  = left_tick;
    last_tick[RIGHT] = right_tick;

     init_encoder = false;
     
     return;
  }

  current_tick = left_tick;

  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
  last_tick[LEFT]      = current_tick;
  last_rad[LEFT]      += TICK2RAD * (double)last_diff_tick[LEFT];

  current_tick = right_tick;

  last_diff_tick[RIGHT] = current_tick - last_tick[RIHGT];
  last_tick[RIHGT]      = current_tick;
  last_rad[RIGHT]      += TICK2RAD * (double)last_diff_tick[RIGHT];
   
}
*/



/********************************************************************************************
  Call back function for wheel velocity
********************************************************************************************/
void lwheel_vtargetCB(const std_msgs::Float32& msg)
{
  motor_left_speed = msg.data;
}

void rwheel_vtargetCB(const std_msgs::Float32& msg)
{
  motor_right_speed = msg.data;  
}






ros::Subscriber<std_msgs::Float32> lwheel_vtarget_sub("/lwheel_vtarget", lwheel_vtargetCB);
ros::Subscriber<std_msgs::Float32> rwheel_vtarget_sub("/rwheel_vtarget", rwheel_vtargetCB);

void setup() {
  // put your setup code here, to run once:
  initMotor();
  SetupEncoders();
  
  nh.initNode();
  rpy_msg.data_length = 3;
  rpy_msg.data = new float[3];
  nh.advertise(imu_pub);
  nh.advertise(rpy_pub);
  nh.advertise(lwheel_pub);
  nh.advertise(rwheel_pub);
  
  tfbroadcaster.init(nh);

  nh.subscribe(lwheel_vtarget_sub);
  nh.subscribe(rwheel_vtarget_sub);

  
  initMPU6050();

}

void loop() {
  // put your main code here, to run repeatedly: 
  static uint32_t pre_time;

  readAccelGyro();

  if(millis() - pre_time >= 50)
  {
    pre_time = millis();

    imu_msg.header.stamp = nh.now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.angular_velocity.x = GyX;
    imu_msg.angular_velocity.y = GyY;
    imu_msg.angular_velocity.z = GyZ;
    imu_msg.angular_velocity_covariance[0] = 0.02;
    imu_msg.angular_velocity_covariance[1] = 0;
    imu_msg.angular_velocity_covariance[2] = 0;
    imu_msg.angular_velocity_covariance[3] = 0;
    imu_msg.angular_velocity_covariance[4] = 0.02;
    imu_msg.angular_velocity_covariance[5] = 0;
    imu_msg.angular_velocity_covariance[6] = 0;
    imu_msg.angular_velocity_covariance[7] = 0;
    imu_msg.angular_velocity_covariance[8] = 0.02;

    imu_msg.linear_acceleration.x = AcX;
    imu_msg.linear_acceleration.y = AcY;
    imu_msg.linear_acceleration.z = AcZ;
    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[1] = 0;
    imu_msg.linear_acceleration_covariance[2] = 0;
    imu_msg.linear_acceleration_covariance[3] = 0;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[5] = 0;
    imu_msg.linear_acceleration_covariance[6] = 0;
    imu_msg.linear_acceleration_covariance[7] = 0;
    imu_msg.linear_acceleration_covariance[8] = 0.04;

    imu_msg.orientation.w = quat[0];
    imu_msg.orientation.x = quat[1];
    imu_msg.orientation.y = quat[2];
    imu_msg.orientation.z = quat[3];

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
    tfs_msg.transform.rotation.w = quat[0];
    tfs_msg.transform.rotation.x = quat[1];
    tfs_msg.transform.rotation.y = quat[2];
    tfs_msg.transform.rotation.z = quat[3];

    tfs_msg.transform.translation.x = 0.0;
    tfs_msg.transform.translation.y = 0.0;
    tfs_msg.transform.translation.z = 0.0;

    tfbroadcaster.sendTransform(tfs_msg);

    rpy_msg.data[0] = 0.0f;
    //rpy_msg.data[0] = filter.getRoll();
    rpy_msg.data[1] = filter.getPitch();
    rpy_msg.data[2] = filter.getYaw() - 180;

    rpy_pub.publish(&rpy_msg);

    lwheel_msg.data = Left_Encoder_Count;
    lwheel_pub.publish(&lwheel_msg);

    
    rwheel_msg.data = Right_Encoder_Count;
    rwheel_pub.publish(&rwheel_msg);
  }
  moveRightMotor(motor_right_speed);
  moveLeftMotor(motor_left_speed);
  
  nh.spinOnce();
}
