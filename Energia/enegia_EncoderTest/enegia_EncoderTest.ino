#define INA_R           PA_4
#define PWM_R           PC_5
#define INA_L           PE_4
#define PWM_L           PC_6

#define ENCODER_LEFT_A      PC_4
#define ENCODER_LEFT_B      PB_3
#define ENCODER_RIGHT_A     PF_3
#define ENCODER_RIGHT_B     PF_2

/**************************************************
* Motor Speed & Motor Encoder
***************************************************/

volatile long Left_Encoder_Count = 0;
volatile long Right_Encoder_Count = 0;
volatile bool Left_Encoder_Pulse;
volatile bool Right_Encoder_Pulse;




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

/***************************************************************
  inital encoder  +  count_encoder(Call back function)
***************************************************************/

void Count_Left_Encoder(){
  Left_Encoder_Pulse = digitalRead(ENCODER_LEFT_B);
  Left_Encoder_Count -= Left_Encoder_Pulse ? -1: +1;   
}
 
void Count_Right_Encoder(){
  Right_Encoder_Pulse = digitalRead(ENCODER_RIGHT_B);
  Right_Encoder_Count += Right_Encoder_Pulse ? -1:+1;
  
}

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



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  initMotor();
  SetupEncoders();
}

void loop() {
  // put your main code here, to run repeatedly: 
  Serial.print("e");
  Serial.print("\t");
  Serial.print(Left_Encoder_Count);
  Serial.print("\n");
  
}
