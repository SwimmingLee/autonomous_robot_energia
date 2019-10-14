//Right Motor
//#define INB_1 12
#define INA_1 13 //PA_4
#define PWM_1 PC_5

//Left Motor
#define INA_2 5  //PE_4
//#define INB_2 6
#define PWM_2 PC_6 


void Move_Backword()
{
  digitalWrite(INA_1, HIGH);
  analogWrite(PWM_1, 255);

  digitalWrite(INA_2, LOW);
  analogWrite(PWM_2, 255);
}


void Move_Left()
{
  digitalWrite(INA_1, LOW);
  analogWrite(PWM_1, 0);

  digitalWrite(INA_2, LOW);
  analogWrite(PWM_2, 255);
}


void Move_Right()
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


void Move_Forword()
{
  digitalWrite(INA_1, LOW);
  analogWrite(PWM_1, 255);

  digitalWrite(INA_2, HIGH);
  analogWrite(PWM_2, 255);
}


void setup() {
  // put your setup code here, to run once:
  pinMode(INA_1, OUTPUT);
  pinMode(PWM_1, OUTPUT);
  digitalWrite(INA_1, HIGH);
  analogWrite(PWM_1, 0);
  pinMode(INA_2, OUTPUT);
  pinMode(PWM_2, OUTPUT);
  digitalWrite(INA_2, HIGH);
  analogWrite(PWM_2, 0);
}

void loop() {
  // put your main code here, to run repeatedly: 

  Move_Forword(); //back
  delay(3000);
  Stop();
  delay(3000);
  Move_Left();  // right
  delay(3000);
  Stop();
  delay(3000);
  
}
