/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

// Use the following line if you have a Leonardo or MKR1000
//#define USE_USBCON

#include <avr/io.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

const uint16_t frq = 50;   // frequency
const float min_ms = 0.5;   // ms(servo min) 
const float max_ms = 2.5;   // ms(servo max) 
float high_ms = 1.5;    // ms(servo now)
uint8_t message_flag = 0; //for timeout

const uint16_t delaytime = 10; //ms
const uint32_t timeout = 30000; //ms

#define SERVO_PT 1
#define SERVO_SW 9
#define SERVO_PWM 10

ros::NodeHandle nh;

std_msgs::Float64 AD_data;
ros::Publisher sendData("float_data", &AD_data);

float AD_potentiometer = 0;

void messageCb(const std_msgs::Float64& msg){
  if(0 <= msg.data && msg.data <= 1.0){
    //servo position (from 0 to 1)
    high_ms = min_ms * (1.0 - msg.data) + max_ms * msg.data;
    message_flag = 1;
  }
}

ros::Subscriber<std_msgs::Float64> sub("your_topic", &messageCb);

void setup()
{
  pinMode(SERVO_SW, OUTPUT);
  pinMode(SERVO_PWM, OUTPUT);

  nh.initNode();
  nh.advertise(sendData);
  nh.subscribe(sub);
}

void loop()
{
  static uint32_t cnt = 0;
  //get potentiometer data
  AD_potentiometer = analogRead(SERVO_PT) / 1024.0;
  AD_data.data = AD_potentiometer;
  sendData.publish( &AD_data );
  //check timeout
  message_flag = 0;
  nh.spinOnce();
  if(message_flag == 0){
    if(cnt < timeout){
      cnt += delaytime;
    }
  }
  else{
    cnt = 0;
  }
  //PWM setting
  TCCR1A = 0b00100001;
  TCCR1B = 0b00010010;
  // TOP
  OCR1A = (unsigned int)(1000000 / frq);
  // Duty
  OCR1B = (unsigned int)(1000 * high_ms);
  //select
  if(timeout <= cnt){
    digitalWrite(SERVO_SW, LOW);
  }
  else{
    digitalWrite(SERVO_SW, HIGH);
  }
  delay(delaytime);
}
