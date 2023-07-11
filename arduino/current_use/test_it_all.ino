#include <Arduino.h>
#include <SPI.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <DHT.h>
#include <ArduinoHardware.h>

#define RCPinY 2
#define RCPinX 3

#define DHTPIN 4
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

// L298N setup, single pwm in and set pin for direction
// IBT-2 sends pwm to the different pins

// Left Wheel
#define EN_LL 6
#define EN_RL 7
#define IN1_L 10
#define IN2_L 11
#define hsla 18 // hall sensors
#define hslb 19
// Right Wheel
#define EN_LR 8
#define EN_RR 9
#define IN1_R 12
#define IN2_R 13
#define hsra 20 // hall sensors
#define hsrb 21

double w_r=0, w_l=0;

//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.02, wheel_sep = 0.34;

volatile unsigned int templ, counterl = 0; //This variable will increase or decrease depending on the rotation of encoder
volatile unsigned int tempr, counterr = 0; //This variable will increase or decrease depending on the rotation of encoder


int PulsesY;
int PulsesX;
int PulseWidthY;
int PulseWidthX;

volatile long StartTimeY = 0;
volatile long CurrentTimeY = 0;
volatile long StartTimeX = 0;
volatile long CurrentTimeX = 0;

int throttle = 0;
int steering = 0;
int rc_pwm_L = 0;
int rc_pwm_R = 0;

float temperature;
float humidity;

unsigned long currentMillis;
unsigned long previousMillis;

int loopTime = 10;

ros::NodeHandle  nh;

int lowSpeed = 100;
int highSpeed = 50;
double speed_ang=0, speed_lin=0;

void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}

void rcPWM(){
  int speed_angRC = map(PulseWidthX, 1000, 2000, -30, 30);
  int speed_linRC = map(PulseWidthY, 1000, 2000, -20, 20);
  rc_pwm_L = (speed_linRC/wheel_rad) - ((speed_angRC*wheel_sep)/(2.0*wheel_rad));
  rc_pwm_R = (speed_linRC/wheel_rad) + ((speed_angRC*wheel_sep)/(2.0*wheel_rad));
}

std_msgs::Int16 str_msg;
std_msgs::Int16 str_msgt;
std_msgs::Int16 str_msgs;
std_msgs::Int16 str_msgf;
std_msgs::Int16 str_msgh;
std_msgs::Float32 a;
std_msgs::Float32 b;
//Publish steps of left and right stepper motors
ros::Publisher pub1("lwheel", &a);
ros::Publisher pub2("rwheel", &b);
ros::Publisher rc_throttle_pub("rc/throttle", &str_msgt);
ros::Publisher rc_steering_pub("rc/steering", &str_msgs);
ros::Publisher temperature_pub("base_sensors/temperature", &str_msgf);
ros::Publisher humidity_pub("base_sensors/humidity", &str_msgh);
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);


void setup() {

  dht.begin();
  pinMode(RCPinY, INPUT_PULLUP);
  pinMode(RCPinX, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RCPinY),PulseTimerY,CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCPinX),PulseTimerX,CHANGE);
  Motors_init();
  //Left Hall
  pinMode(hsla, INPUT_PULLUP); 
  pinMode(hslb, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(hsla), ai0, RISING);
  attachInterrupt(digitalPinToInterrupt(hslb), ai1, RISING);
  //Right Hall
  pinMode(hsra, INPUT_PULLUP); 
  pinMode(hsrb, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(hsra), ai2, RISING);
  attachInterrupt(digitalPinToInterrupt(hsrb), ai3, RISING);
  nh.initNode();              // init ROS
  nh.advertise(rc_throttle_pub);     
  nh.advertise(rc_steering_pub);
  nh.advertise(temperature_pub);
  nh.advertise(humidity_pub);
  nh.advertise(pub1);
  nh.advertise(pub2);
  nh.subscribe(sub);
  
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(hslb)==LOW) {
  counterl++;
  }else{
  counterl--;
  }
  }
   
  void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(hsla)==LOW) {
  counterl--;
  }else{
  counterl++;
  }
  }


void ai2() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(hsrb)==LOW) {
  counterr++;
  }else{
  counterr--;
  }
  }
   
  void ai3() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(hsra)==LOW) {
  counterr--;
  }else{
  counterr++;
  }
  }


void PulseTimerY(){
  CurrentTimeY = micros();
   if (CurrentTimeY > StartTimeY){
    PulsesY = CurrentTimeY - StartTimeY;
    StartTimeY = CurrentTimeY;
   }
  }

void PulseTimerX(){
  CurrentTimeX = micros();
   if (CurrentTimeX > StartTimeX){
    PulsesX = CurrentTimeX - StartTimeX;
    StartTimeX = CurrentTimeX;
   }
  }

float deadzone(float value) {
     if (value > 50) {
        value = value - 50;
     }
     else if (value < -50) {
      value = value +50;
     }
     else {
      value = 0;
     }
     value = value / 500;   // scale so that we get 0.0 ~ 1.0
     return value;  
}


void loop() {
      
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  if (PulsesY < 2000){
    PulseWidthY = PulsesY;
  }  
  if (PulsesX < 2000){
    PulseWidthX = PulsesX;
  }

// if switch set to cmd_vel
  MotorL(w_l*10);
  MotorR(w_r*10);
// if switch set to rc. Will enable just battery pack to mega to just move the robot
//     without having to boot up the whole shebang
//  rcPWM();
//  MotorL(rc_pwm_L);
//  MotorR(rc_pwm_R);

  // PulseWidthY = (PulseWidthY) - 0;
  // PulseWidthX = (PulseWidthX) + 0;

  str_msgt.data = PulseWidthY;
  str_msgs.data = PulseWidthX;
  str_msgf.data = temperature;
  str_msgh.data = humidity;

  templ = counterl;
  tempr = counterr;
  a.data = counterl;
  b.data = counterr;

  pub1.publish(&a);
  pub2.publish(&b);

  rc_throttle_pub.publish(&str_msgt);
  rc_steering_pub.publish(&str_msgs);
  temperature_pub.publish(&str_msgf);
  humidity_pub.publish(&str_msgh);

  nh.spinOnce();

  delay(10);
}


void Motors_init(){
 pinMode(EN_LL, OUTPUT);
 pinMode(EN_RL, OUTPUT);
 pinMode(EN_LR, OUTPUT);
 pinMode(EN_RR, OUTPUT);
 pinMode(IN1_L, OUTPUT);
 pinMode(IN2_L, OUTPUT);
 pinMode(IN1_R, OUTPUT);
 pinMode(IN2_R, OUTPUT);
 digitalWrite(EN_LL, LOW);
 digitalWrite(EN_RL, LOW);
 digitalWrite(EN_LR, LOW);
 digitalWrite(EN_RR, LOW);
 digitalWrite(IN1_L, HIGH);
 digitalWrite(IN2_L, HIGH);
 digitalWrite(IN1_R, HIGH);
 digitalWrite(IN2_R, HIGH);
}


void MotorL(int Pulse_Width1){
 if (Pulse_Width1 > 0){
    digitalWrite(EN_LL, LOW);    
    analogWrite(EN_RL, Pulse_Width1);
 }

 if (Pulse_Width1 < 0){
    Pulse_Width1=abs(Pulse_Width1);
    digitalWrite(EN_RL, LOW);    
    analogWrite(EN_LL, Pulse_Width1);
 }

 if (Pulse_Width1 == 0){
    digitalWrite(EN_RL, LOW);
    digitalWrite(EN_LL, LOW);  
 }
}


void MotorR(int Pulse_Width2){
 if (Pulse_Width2 > 0){
    digitalWrite(EN_RR, LOW);
    analogWrite(EN_LR, Pulse_Width2);
 }

 if (Pulse_Width2 < 0){
    Pulse_Width2=abs(Pulse_Width2);
    digitalWrite(EN_LR, LOW);    
    analogWrite(EN_RR, Pulse_Width2);
 }

 if (Pulse_Width2 == 0){
    digitalWrite(EN_RR, LOW);
    digitalWrite(EN_LR, LOW);
 }
}