
#include "ros.h"
#include "std_msgs/Bool.h"

#define PUSHBUTTON1   31
#define PUSHBUTTON2   17

#define LED_GREEN     39
#define LED_RED       30
#define LED_BLUDE     40

ros::NodeHandle nh;
std_msgs::Bool pushed_msg;
ros::Publisher pub_button("pushed", &pushed_msg);

bool last_reading;
long last_debounce_time = 0;
long debounce_delay = 50;
bool published = true;

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(pub_button);
  
  pinMode(LED_GREEN, OUTPUT);
  pinMode(PUSHBUTTON1, INPUT_PULLUP);

  digitalWrite(PUSHBUTTON1, HIGH);

  last_reading = !digitalRead(PUSHBUTTON1);
}

void loop() {
  // put your main code here, to run repeatedly: 
  bool reading = ! digitalRead(PUSHBUTTON1);

  if(last_reading != reading)
  {
    last_debounce_time = millis();
    published = false;
  }

  if(!published && (millis() - last_debounce_time) > debounce_delay)
  {
    digitalWrite(LED_GREEN, reading);
    pushed_msg.data = reading;
    pub_button.publish(&pushed_msg);
    published = true;
  }

  last_reading = reading;

  nh.spinOnce();
  
 }

