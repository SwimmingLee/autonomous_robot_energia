#ifndef AR_CORE_CONFIG_H_
#define AR_CORE_CONFIG_H_

#include <ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <Wire.h>

#include "MadgwickAHRS.h"

#include <math.h>



#define INA_R           PA_4
#define PWM_R           PC_5
#define INA_L           PE_4
#define PWM_L           PC_6

#define ENCODER_LEFT_A      PC_4
#define ENCODER_LEFT_B      PB_3
#define ENCODER_RIGHT_A     PF_3
#define ENCODER_RIGHT_B     PF_2

#define WHEEL_NUM           2
#define WHEEL_RADIUS        0.045
#define WHEEL_SEPARATION    0.293
#define ENCODER_MIN         -2147483648
#define ENCODER_MAX         2147483648


#define LEFT            0
#define RIGHT           1

#define LINEAR          0
#define ANGULAR         1

#define TICK2RAD        0.0001530617243

#define DEG2RAD(x)      (x * 0.01745329252)  // * PI/180
#define RAD2DEG(x)      (x * 57.2957795131)  // * 180/PI


#define IMU_PUBLISH_PERIOD                  5 // msec
#define WHEEL_ENCODER_PUBLISH_PERIOD        3 // msec
#define DRIVE_INFORMATION_PUBLISH_PERIOD    30 // msec







//Callback function prototypes
void lwheel_vtargetCB(const std_msgs::Float32& msg);
void rwheel_vtargetCB(const std_msgs::Float32& msg);

//Funtion prototypes
void Move_Forword();
void Move_Right();
void Move_Left();
void Stop();
void Move_Backword();

void initMotor();
void moveRightMotor(float rightServoValue);
void moveLeftMotor(float leftSerboValue);

void SetupEncoders();
void Count_Left_Encoder();
void Count_Right_Encoder();

void initMPU6050();
void readAccelGyro();

void initOdom();
void initJointStates();

void updateMotorInfo(int32_t left_ticks, int32_t right_tick);
void updateOdometry(void);
void updateJointStates(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateIMU();

bool calcOdometry(double diff_time);


void publishDriveInformation();


ros::Time addMircos(ros::Time &t, uint32_t _micros);
/**************************************************
* ROS NodeHandle
***************************************************/
ros::NodeHandle nh;
ros::Time current_time; //current_time
uint32_t current_offset; //current_offset


/**************************************************
* SoftwareTimer of AR
***************************************************/
static uint32_t tTime[5] = { 0, };


/**************************************************
* Subscriber
***************************************************/
ros::Subscriber<std_msgs::Float32> lwheel_vtarget_sub("/lwheel_vtarget", lwheel_vtargetCB);

ros::Subscriber<std_msgs::Float32> rwheel_vtarget_sub("/rwheel_vtarget", rwheel_vtargetCB);


/**************************************************
* Publisher
***************************************************/
//IMU of AR
sensor_msgs::Imu  imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

//roll pitch yaw of AR
std_msgs::Float32MultiArray rpy_msg;
ros::Publisher rpy_pub("rpy", &rpy_msg);

//left wheel encoder of AR
std_msgs::Int64 lwheel_msg;
ros::Publisher lwheel_pub("lwheel", &lwheel_msg);

//right wheel encoder of AR
std_msgs::Int64 rwheel_msg;
ros::Publisher rwheel_pub("rwheel", &rwheel_msg);

// Odometry of AR
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

// Joint State of AR
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);




/**************************************************
* Transform Broadcaster
***************************************************/
//TF of AR
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;





/**************************************************
* IMU
***************************************************/
Madgwick filter;
const int MPU6050_addr = 0x68;
volatile int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, MagX, MagY, MagZ;
volatile float ax, ay, az, gx, gy, gz;
volatile float quat[4];
float aRes, gRes, mRes;


/**************************************************
* Motor Speed & Motor Encoder
***************************************************/
float motor_left_speed = 0;
float motor_right_speed = 0;

volatile long Left_Encoder_Count =0;
volatile long Right_Encoder_Count =0;
volatile bool Left_Encoder_Pulse;
volatile bool Right_Encoder_Pulse;



/**************************************************
* Calulation for odomerty
***************************************************/
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
double last_rad[WHEEL_NUM]        = {0.0, 0.0};


/**************************************************
* Update Joint States
***************************************************/
double last_velocity[WHEEL_NUM] = {0.0, 0.0};

/**************************************************
* Delaration for SLAM and navigation
***************************************************/
float odom_pose[3];
double odom_vel[3];
unsigned long prev_update_time;



#endif // AR_CORE_CONFIG_H_void
