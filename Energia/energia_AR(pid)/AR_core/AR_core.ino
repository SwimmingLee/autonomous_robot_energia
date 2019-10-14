//AR_core(PID)

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
  
  quat_msg.data_length = 4;
  quat_msg.data = new float[4];

  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(rpy_pub);
  nh.advertise(quat_pub);
  nh.advertise(lwheel_pub);
  nh.advertise(rwheel_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  nh.advertise(lwheel_vel_pub);
  nh.advertise(rwheel_vel_pub);
  //debuggingi publisher
  nh.advertise(debugging_pub);

  tf_broadcaster.init(nh);

  nh.subscribe(lwheel_vtarget_sub);
  nh.subscribe(rwheel_vtarget_sub);

  nh.subscribe(lvel_Kp_sub);
  nh.subscribe(lvel_Ki_sub);
  nh.subscribe(lvel_Kd_sub);


  nh.subscribe(rvel_Kp_sub);
  nh.subscribe(rvel_Ki_sub);
  nh.subscribe(rvel_Kd_sub);
  
  initMPU6050();
  initOdom();
  initJointStates();
  initPIDvalue();
/*
  nh.setParam("lKp", Kp[LEFT]);
  nh.setParam("lKi", Ki[LEFT]);
  nh.setParam("lKd", Kd[LEFT]);

  nh.setParam("rKp", Kp[RIGHT]);
  nh.setParam("rKi", Ki[RIGHT]);
  nh.setParam("rKd", Kd);
 */ 
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

    quat_msg.data[0] = quat[0];
    quat_msg.data[1] = quat[1];
    quat_msg.data[2] = quat[2];
    quat_msg.data[3] = quat[3];

    imu_pub.publish(&imu_msg);
    rpy_pub.publish(&rpy_msg);
    quat_pub.publish(&quat_msg);
  
  }

  if((millis() - t_encoder) >= WHEEL_ENCODER_PUBLISH_PERIOD)
  {
    t_encoder = millis();
    lwheel_msg.data = Left_Encoder_Count;
    lwheel_pub.publish(&lwheel_msg);
    
    rwheel_msg.data = Right_Encoder_Count;
    rwheel_pub.publish(&rwheel_msg);

  }

  if((millis() - t_drive) >= 5)
  {
    t_drive = millis();
    updateMotorInfo(Left_Encoder_Count, Right_Encoder_Count);

    publishDriveInformation();
    
    lwheel_vel.data = cur_vel[LEFT];
    rwheel_vel.data = cur_vel[RIGHT];

    lwheel_vel_pub.publish(&lwheel_vel);
    rwheel_vel_pub.publish(&rwheel_vel);
    
  }
  /*
  nh.getParam("lKp", Kp[LEFT]);
  nh.getParam("lKi", Ki[LEFT]);
  nh.getParam("lKd", Kd[LEFT]);

  nh.getParam("rKp", Kp[RIGHT]);
  nh.getParam("rKi", Ki[RIGHT]);
  nh.getParam("rKd", Ki[RIGHT]);
  */

  
  calcVelocity();
  doPID();
 
  moveLeftMotor(vel_output[LEFT]);
  moveRightMotor(vel_output[RIGHT]);
  
  debugging_pub.publish(&debugging_msgs);
  
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
  Left_Encoder_Count = 0;
  Right_Encoder_Count = 0;

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
  initial PID control
************************************************/
void initPIDvalue(){
  encoder_low_wrap = ((long long)ENCODER_MAX - ENCODER_MIN) * 0.3f + ENCODER_MIN;
  encoder_high_wrap = ((long long)ENCODER_MAX - ENCODER_MIN) * 0.7f + ENCODER_MIN;

  for(int i = 0; i < WHEEL_NUM; i++)
  {
    target_vel[i] = 0.f;
    cur_vel[i] = 0.0;
    vel_output[i] = 0.f;

    vel_error[i] = 0.f;
    previous_vel_error[i] = 0.f;
    vel_intergral[i] = 0.f;
    vel_derivative[i] = 0.f;

    wheel_prev[i] = 0.0;
    wheel_latest[i] = 0.0;
    wheel_mult[i] = 0;

  }

  Kp[LEFT] = 550.f;
  Ki[LEFT] = 200.f;
  Kd[LEFT] = 40.f;

  Kp[RIGHT] = 550.f;
  Ki[RIGHT] = 200.f;
  Kd[RIGHT] = 40.f;
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

  //이것도 나눗셈으로 하지 말고 곱셈으로 바꾸자. 어차피 한번만 괴화하는거라서 크게 상고나은 없다.
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

  filter.invSampleFreq = (float)process_time * 0.000001f;

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
  static char * joint_states_name[2] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};

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

  //굳이 상관은 없지만 2.0나눗셈보단는 0.5곱하기가 좋지않을까?
  delta_s = WHEEL_RADIUS * (wheel_r + wheel_l) * 0.5;
  //theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
  theta = atan2(quat[1]*quat[2] + quat[0]*quat[3], 
                0.5f - quat[2]*quat[2] - quat[3]*quat[3]);

  delta_theta = theta - last_theta;

  v = delta_s / step_time;
  w = delta_theta / step_time;

  last_velocity[LEFT]  = wheel_l / step_time;
  last_velocity[RIGHT] = wheel_r / step_time;


  //compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta * 0.5 ));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta * 0.5 ));
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
  target_vel[LEFT] = msg.data;
}

void rwheel_vtargetCB(const std_msgs::Float32& msg)
{
  target_vel[RIGHT] = msg.data;  
}

void lvel_KpCB(const std_msgs::Float32& msgs){
  Kp[LEFT] = msgs.data;  
}
void lvel_KiCB(const std_msgs::Float32& msgs){
  Ki[LEFT] = msgs.data;  
}
void lvel_KdCB(const std_msgs::Float32& msgs){
  Kd[LEFT] = msgs.data;  
}

void rvel_KpCB(const std_msgs::Float32& msgs){
  Kp[RIGHT] = msgs.data;  
}
void rvel_KiCB(const std_msgs::Float32& msgs){
  Ki[RIGHT] = msgs.data;  
}
void rvel_KdCB(const std_msgs::Float32& msgs){
  Kd[RIGHT] = msgs.data;  
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

/********************************************************************************************
  motor velocity PID control
********************************************************************************************/
void doPID()
{
  static uint32_t prev_PID_time[WHEEL_NUM] = {micros(), micros()};
  static uint32_t cur_PID_time[WHEEL_NUM] = {0, 0};
  static uint32_t PID_time[WHEEL_NUM] = {0 , 0};

  //pid_dt를 해결해야 한다.
  for(int i = 0; i< WHEEL_NUM; i++)
  {
    cur_PID_time[i] = micros();
    PID_time[i]     = cur_PID_time[i] - prev_PID_time[i];
    prev_PID_time[i] = cur_PID_time[i];

    float pid_dt = (float)PID_time[i] * 0.000001f;

    vel_error[i] = target_vel[i] - cur_vel[i];
    vel_intergral[i] = vel_intergral[i] + (vel_error[i] * pid_dt);
    vel_derivative[i] = -(cur_vel[i] - previous_vel_error[i]) / pid_dt;
    previous_vel_error[i] = cur_vel[i];

    vel_output[i] = Kp[i] * vel_error[i] + Ki[i] * vel_intergral[i] + Kd[i] * vel_derivative[i];
    

    if(vel_output[i] > VEL_OUTPUT_MAX)
    {
      vel_output[i] = VEL_OUTPUT_MAX;
      vel_intergral[i] = vel_intergral[i] - (vel_error[i] * pid_dt);
    }
    if(vel_output[i] < VEL_OUTPUT_MIN)
    {
      vel_output[i] = VEL_OUTPUT_MIN;
      vel_intergral[i] = vel_intergral[i] - (vel_error[i] * pid_dt);
    }
    if(target_vel[i] == 0){
      vel_output[i] = 0;
    }
  }
}


/********************************************************************************************
  motor velocity calculate for PID control
********************************************************************************************/
void calcVelocity()
{
  static uint32_t prev_calcVel_time[WHEEL_NUM] = {micros(), micros()};
  static uint32_t cur_calcVel_time[WHEEL_NUM] = {0, 0};
  static uint32_t calcVel_time[WHEEL_NUM] = {0 , 0};

  enc[LEFT] = (double)Left_Encoder_Count;
  enc[RIGHT] = (double)Right_Encoder_Count;

  for(int i = 0; i < WHEEL_NUM ; i++)
  {
    cur_calcVel_time[i] = micros();
    calcVel_time[i]     = cur_calcVel_time[i] - prev_calcVel_time[i];
    prev_calcVel_time[i] = cur_calcVel_time[i];
 
    double dt = (double)calcVel_time[i] * 0.000001;

    if(enc[i] < encoder_low_wrap && prev_encoder[i] > encoder_high_wrap)
    {
      wheel_mult[i] = wheel_mult[i] + 1;
    }

    if(enc[i] > encoder_high_wrap && prev_encoder[i] < encoder_low_wrap)
    {
      wheel_mult[i] = wheel_mult[i] - 1;
    }//오버플로우를 막으려고 한 것
 
    wheel_latest[i] = (double)(enc[i] + wheel_mult[i] * ((long long)ENCODER_MAX - ENCODER_MIN)) * METER_PER_TICKS;
    //wheel_latest[i] = enc[i] * METER_PER_TICKS;
    cur_vel[i] = (double)(wheel_latest[i] - wheel_prev[i]) / dt;
    wheel_prev[i] = wheel_latest[i];

    prev_encoder[i] = enc[i];

    debugging_msgs.data = dt;
  }
}
