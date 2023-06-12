// CODE "Borrowed" from James Bruton's RUR project If you see this James,
// love the vids, sorry I removed most of your code, just that I only want
// /base_link and /odom with the odrive v3.6 firmware 0.5.5 diffbot
#include <HardwareSerial.h>

//ODrive
#include <ODriveArduino.h>

//ROS
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include <ros/time.h>

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

//ODrive Objects
ODriveArduino odrive1(Serial1);

ros::NodeHandle nh;

// cmd_vel variables to be received to drive with
float demandx;
float demandz;

// timers for the sub-main loop
unsigned long currentMillis;
long previousMillis = 0;    // set up timers
float loopTime = 10;

// ODrive init stuff
int requested_state;
float leftVel;
float rightVel;

// output variables to drive the ODrive
float forward0;
float forward1;
float turn0;
float turn1;

// ** ROS callback & subscriber **

void velCallback(  const geometry_msgs::Twist& vel)
{
     demandx = vel.linear.x;
     demandz = vel.angular.z;
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);

// ** Setup **

void setup() {

  nh.getHardware()->setBaud(115200);      // set baud rate to 115200
  nh.initNode();              // init ROS
  nh.subscribe(sub);          // subscribe to cmd_vel
  
  //nh.advertise(odom_pub);
  //broadcaster.init(nh);       // set up broadcaster

  // pinMode(2, INPUT_PULLUP);   // ODrive init switch

  Serial1.begin(115200);    // ODrive
  Serial6.begin(115200);    // debug port using a USB-serial adapter (Serial-zero is in use by ros_serial

  requested_state = AXIS_STATE_MOTOR_CALIBRATION;
  Serial1 << "Axis" << "0" << ": Requesting state " << requested_state << '\n';
  if(!odrive1.run_state(0, requested_state, true)) return;

  requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  Serial1 << "Axis" << "0" << ": Requesting state " << requested_state << '\n';
  if(!odrive1.run_state(0, requested_state, true, 25.0f)) return;

  requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  Serial1 << "Axis" << "0" << ": Requesting state " << requested_state << '\n';
  if(!odrive1.run_state(0, requested_state, false /*don't wait*/)) return;

  requested_state = AXIS_STATE_MOTOR_CALIBRATION;
  Serial1 << "Axis" << "1" << ": Requesting state " << requested_state << '\n';
  if(!odrive1.run_state(1, requested_state, true)) return;

  requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  Serial1 << "Axis" << "1" << ": Requesting state " << requested_state << '\n';
  if(!odrive1.run_state(1, requested_state, true, 25.0f)) return;

  requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  Serial1 << "Axis" << "1" << ": Requesting state " << requested_state << '\n';
  if(!odrive1.run_state(1, requested_state, false /*don't wait*/)) return;

}

// ** Main loop **

void loop() {

  nh.spinOnce();        // make sure we listen for ROS messages and activate the callback if there is one

  currentMillis = millis();
        if (currentMillis - previousMillis >= loopTime) {  // run a loop every 10ms          
            previousMillis = currentMillis;          // reset the clock to time it

            // deal with driving wheels

            forward0 = demandx * 10; // convert m/s into counts/s
            forward1 = demandx * 10; // convert m/s into counts/s

            turn0 = demandz * 10;    // convert rads/s into counts/s
            turn1 = demandz * 10;    // convert rads/s into counts/s

            forward1 = forward1*-1;      // one motor and encoder is mounted facing the other way

            leftVel = forward0 - turn0;
            rightVel = forward1 - turn1;

            odrive1.SetVelocity(0, leftVel); 
            odrive1.SetVelocity(1, rightVel);

        } // end of timed loop

} // end of main loop