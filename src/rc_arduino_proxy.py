#!/usr/bin/env python3

"""
Class for arduino RC interpreter to teleom twist messages
"""
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

#  ros_pub_twist = ''

class RcArduinoInput():
    def __init__(self): 
        rospy.init_node('rc_arduino_proxy')
        #  self.ros_pub_twist = ros_pub_twist
        self.ros_pub_twist1 = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.ros_pub_twist2 = rospy.Publisher("rc/cmd_vel", Twist, queue_size=1)
        self.ros_pub_twist = ''
        #  if self.ros_pub_twist == '':
        #     self.ros_pub_twist = self.ros_pub_twist2
        #--- Create the Subscriber to RC Throttle commands
        self.ros_sub_rc_throttle  = rospy.Subscriber("rc/throttle", Int16, self.update_throttle)
        self.ros_sub_rc_steering  = rospy.Subscriber("rc/steering", Int16, self.update_steering)
        self.ros_control_sub  = rospy.Subscriber("raptor/control", String, self.update_control)
        rospy.loginfo("> Subscriber corrrectly initialized")
        
        self._pwm_min = 1000
        self._pwm_max = 2000
        
        self._sign_throttle = 1
        self._sign_steering = 1
        
        self._offset_throttle = 0
        self._offset_steering = 0
        
        self._values_received = 0
        
        self._timeout_s = 2;
        self._last_time_cmd_rcv = time.time() - self._timeout_s;
        
        self.ros_twist_msg = Twist();

    def update_control(self, data):
        if str(data) == 'data: "rc"':
            self.ros_pub_twist = self.ros_pub_twist1
        else:
            self.ros_pub_twist = self.ros_pub_twist2
        print(self.ros_pub_twist)

    def update_throttle(self, message):
        self.ros_twist_msg.linear.x = self.pwm_to_adimensional(message.data + self._offset_throttle) * self._sign_throttle
        self._values_received += 1
        self._last_time_cmd_rcv = time.time()
        
    def update_steering(self, message):
        self.ros_twist_msg.angular.z = self.pwm_to_adimensional(message.data + self._offset_steering) * self._sign_steering
        self._values_received += 1
        self._last_time_cmd_rcv = time.time()
        
    def pwm_to_adimensional(self, pwm):
        pwm = max(pwm, self._pwm_min)
        pwm = min(pwm, self._pwm_max)
        pwm = pwm - ((self._pwm_max + self._pwm_min) * 0.5)
        pwm = pwm / ((self._pwm_max - self._pwm_min) * 0.5)
        return pwm
        
    @property
    def is_rc_connected(self):
        return(self._values_received >= 2)        
        
    def run(self):
        #--- Set the control rate
        #rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.is_rc_connected:
                if self.ros_pub_twist == '':
                    self.ros_pub_twist = self.ros_pub_twist2
                self.ros_pub_twist.publish(self.ros_twist_msg)
                self._values_received = 0

            #rate.sleep()

if __name__ == "__main__":
    rc_proxy  = RcArduinoInput()
    rc_proxy.run()