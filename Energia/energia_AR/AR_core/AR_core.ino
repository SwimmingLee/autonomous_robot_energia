//AR_core

#include "AR_core_config.h"


/**************************************************
* Setup function
***************************************************/
void setup()
{
  initMotor();
  SetupEncoders();

  rpy_msg.data_length = 3;
  rpy_msg.data = new float[3];


  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(rpy_pub);
  nh.advertise(lwheel_pub);
  nh.advertise(rwheel_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  
  tf_broadcaster.init(nh);

  nh.subscribe(lwheel_vtarget_sub);
  nh.subscribe(rwheel_vtarget_sub);

  
  initMPU6050();
  initOdom();
  initJointStates();

}


/**************************************************
* Loop function
***************************************************/
void loop()
{
  // put your main code here, to run repeatedly: 
  static uint32_t t_imu;
  static uint32_t t_encoder;
  static uint32_t t_drive;
  
  //t = millis();
  current_offset = micros();
  current_time = nh.now();

  readAccelGyro();

  if((millis() - t_imu) >= IMU_PUBLISH_PERIOD)
  {
    t_imu = millis();
    updateIMU();

    rpy_msg.data[0] = filter.getRoll();
    rpy_msg.data[1] = filter.getPitch();
    rpy_msg.data[2] = filter.getYaw() - 180;

    imu_pub.publish(&imu_msg);
    rpy_pub.publish(&rpy_msg);


    
  }

  if((millis() - t_encoder) >= WHEEL_ENCODER_PUBLISH_PERIOD)
  {
    t_encoder = millis();
    lwheel_msg.data = Left_Encoder_Count;
    lwheel_pub.publish(&lwheel_msg);
    
    rwheel_msg.data = Right_Encoder_Count;
    rwheel_pub.publish(&rwheel_msg);

  }

  if((millis() - t_drive) >= 50)
  {
    t_drive = millis();
    updateMotorInfo(Left_Encoder_Count, Right_Encoder_Count);

    publishDriveInformation();
    
  }


  moveRightMotor(motor_right_speed);
  moveLeftMotor(motor_left_speed);
  
  nh.spinOnce();
}





/************************************************
  AR motor direction Test
*************************************************/
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




/****************************************************
  Initial Motor
****************************************************/
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



/*******************************************************
  Moving the AR Motor 
********************************************************/
void moveRightMotor(float rightServoValue)
{
  //rightServoValue *= 300;
  
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
  //leftSerboValue *= 300;
  
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

/***************************************************************
  inital encoder  +  count_encoder(Call back function)
***************************************************************/
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




/************************************************
  MPU6050 initialation
*************************************************/
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


/***********************************************************************
  read MPU6050 Data ( Accelometer, Gryoscope)
************************************************************************/
void readAccelGyro()
{
  static uint32_t prev_process_time = micros();
  static uint32_t cur_process_time = 0;
  static uint32_t process_time = 0;

  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr, 14, true);
  AcX = ((Wire.read() << 8) | (Wire.read())) - 350;
  AcY = ((Wire.read() << 8) | (Wire.read())) - 450;
  AcZ = ((Wire.read() << 8) | (Wire.read()));
  Tmp = ((Wire.read() << 8) | (Wire.read()));
  GyX = ((Wire.read() << 8) | (Wire.read())) - 260;
  GyY = ((Wire.read() << 8) | (Wire.read())) - 190;
  GyZ = ((Wire.read() << 8) | (Wire.read())) +  80;

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


/**************************************************************************
  Initial Odometry for SLAM
**************************************************************************/
void initOdom(void)
{
  init_encoder = true;

  for(int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index] = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.x = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.x = 0.0;

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.angular.z = 0.0;
}


/********************************************************************************************
  Initial JointStates for SLAM
********************************************************************************************/
void initJointStates(void)
{
  static char* joint_states_name[2] = {"wheel_left_joint", "wheel_right_joint"};

  joint_states.header.frame_id = "base_link";
  joint_states.name = joint_states_name;

  joint_states.name_length = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length = WHEEL_NUM;
}


/********************************************************************************************
  Update Odometry for SLAM
********************************************************************************************/
void updateOdometry(void)
{
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];
}


/********************************************************************************************
  update JointStates for SLAM
********************************************************************************************/
void updateJointStates(void)
{
  static double joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
  static double joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
  static double joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

  joint_states_pos[LEFT]  = last_rad[LEFT];
  joint_states_pos[RIGHT] = last_rad[RIGHT];

  joint_states_vel[LEFT]  = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}


/********************************************************************************************
  update TF for SLAM
********************************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}


/********************************************************************************************
  Calculation Odometry for SLAM
********************************************************************************************/
bool calcOdometry(double diff_time)
{
  double wheel_l, wheel_r; // roatation value of wheel [rad]
  double delta_s, theta, delta_theta;

  static double last_theta = 0.0;
  //v = translational velocity [m/s], w = roatational velocityy [rad/s]
  double v, w;
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if(step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

  if(isnan(wheel_l))
    wheel_l = 0.0;

  if(isnan(wheel_r))
    wheel_r = 0.0;

  delta_s = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  //theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
  theta = atan2(quat[1]*quat[2] + quat[0]*quat[3], 
                0.5f - quat[2]*quat[2] - quat[3]*quat[3]);

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


/********************************************************************************************
  Update Motor information for SLAM
********************************************************************************************/
void updateMotorInfo(int32_t left_tick, int32_t right_tick)
{
  int32_t current_tick = 0;
  static int32_t last_tick[WHEEL_NUM] = {0, 0};

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

  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
  last_tick[RIGHT]      = current_tick;
  last_rad[RIGHT]      += TICK2RAD * (double)last_diff_tick[RIGHT];
   
}

/********************************************************************************************
  Update IMU
********************************************************************************************/
void updateIMU()
{
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
}



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


/********************************************************************************************
  Publish Drive Information of AR
********************************************************************************************/
void publishDriveInformation()
{
    unsigned long time_now = millis();
    unsigned long step_time = time_now - prev_update_time;

    prev_update_time = time_now;
    ros::Time stamp_now = addMircos(current_time, micros() - current_offset);

    //calculate Odometry
    calcOdometry((double)(step_time * 0.001));
    
    // Odometry
    updateOdometry();
    odom.header.stamp = stamp_now;
    odom_pub.publish(&odom);

    // odomerty tf
    updateTF(odom_tf);
    odom_tf.header.stamp = stamp_now;
    tf_broadcaster.sendTransform(odom_tf);

    // joint states
    updateJointStates();
    joint_states.header.stamp = stamp_now;
    joint_states_pub.publish(&joint_states);
    

}

/********************************************************************************************
  ros Time
********************************************************************************************/
ros::Time addMircos(ros::Time &t, uint32_t _micros)
{
  uint32_t sec, nsec;

  sec = _micros / 1000000 + t.sec;
  nsec = _micros % 1000000 + 1000 * (t.nsec / 1000);

  if(nsec >= 1e9)
  {
    sec++;
    nsec--;
  }
  return ros::Time(sec, nsec);
}

