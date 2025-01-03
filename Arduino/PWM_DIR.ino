
  #include <ros.h>
  #include <std_msgs/Int16MultiArray.h>
  #include "CytronMotorDriver.h"
  #include <Servo.h>
  
  Servo T200M5;
  Servo T200M6;

  // Configure the motor driver.
  CytronMD motor1(PWM_DIR, 3, 2);  // PWM = Pin 3, DIR = Pin 2.
  CytronMD motor2(PWM_DIR, 5, 4);  // PWM = Pin 5, DIR = Pin 4.
  CytronMD motor3(PWM_DIR, 6, 7);  // PWM = Pin 6, DIR = Pin 7.
  CytronMD motor4(PWM_DIR, 9, 8);  // PWM = Pin 9, DIR = Pin 8.

  //pins of T200 motors
  byte servoPinM5 = 10; // T200 motor5, Pin10
  byte servoPinM6 = 11; // T200 motor6, Pin11  
  
  //NodeHandle
  ros::NodeHandle  nh;

  //Callback of sub
  void pwm_input( const std_msgs::Int16MultiArray& pwm_value){
    T200M5.writeMicroseconds(pwm_value.data[4]);
    T200M6.writeMicroseconds(pwm_value.data[4]);
    motor1.setSpeed(pwm_value.data[0]); 
    motor2.setSpeed(pwm_value.data[1]); 
    motor3.setSpeed(pwm_value.data[2]); 
    motor4.setSpeed(pwm_value.data[3]);
    

  }
  

  //Subscriber
  ros::Subscriber<std_msgs::Int16MultiArray> sub("PWM_motor", &pwm_input);

  
  
  
  // The setup routine runs once when you press reset.
  void setup() {
  T200M5.attach(servoPinM5);
  T200M6.attach(servoPinM6);
  T200M5.writeMicroseconds(1500);
  T200M6.writeMicroseconds(1500);

  
  nh.initNode();
  nh.subscribe(sub);

  delay(1000);
  }
  
  
  // The loop routine runs over and over again forever.
  void loop() {
    

    nh.spinOnce();
  
  }
