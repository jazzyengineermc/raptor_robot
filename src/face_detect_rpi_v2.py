#!/usr/bin/env python3
# MIT License
# Copyright (c) 2019-2022 JetsonHacks
# See LICENSE for OpenCV license and additional information

# https://docs.opencv.org/3.3.1/d7/d8b/tutorial_py_face_detection.html
# On the Jetson Nano, OpenCV comes preinstalled
# Data files are in /usr/sharc/OpenCV

import sys
import cv2
from geometry_msgs.msg import Twist
import sys
import rospy
from std_msgs.msg import String

# gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
# Defaults to 1920x1080 @ 30fps
# Flip the image by setting the flip_method (most common values: 0 and 2)
# display_width and display_height determine the size of the window on the screen
# Notice that we drop frames if we fall outside the processing time in the appsink element
#def gstreamer_pipeline(
#    capture_width=1920,
#    capture_height=1080,
#    display_width=960,
#    display_height=540,
#    framerate=30,
#    flip_method=0,
#):
#--- Define our Class
class face_detect:

    def __init__(self):
        self.rcvel_pub2 = rospy.Publisher("raptor/cmd_vel", Twist, queue_size=1)
        self.rcvel_pub1 = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.face_cam_sub = rospy.Subscriber("raptor/control", String, self.doidoit)
        #--- Twist stuffs
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.angular.z = 0  
        self.face_tracking()  


    def gstreamer_pipeline(
        capture_width=640,
        capture_height=480,
        display_width=640,
        display_height=480,
        framerate=30,
        flip_method=0,
    ):
        return (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink drop=True"
            % (
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
        )   


    def doidoit(self,izzy):
        if str(izzy) == 'data: "face"':
            self.rcvel_pub = self.rcvel_pub1
        else:
            self.rcvel_pub = self.rcvel_pub2

        print(izzy)


    def face_tracking(self):
        def gstreamer_pipeline(
            capture_width=640,
            capture_height=480,
            display_width=640,
            display_height=480,
            framerate=30,
            flip_method=0,
        ):
            return (
                "nvarguscamerasrc ! "
                "video/x-raw(memory:NVMM), "
                "width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
                "nvvidconv flip-method=%d ! "
                "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
                "videoconvert ! "
                "video/x-raw, format=(string)BGR ! appsink drop=True"
                % (
                    capture_width,
                    capture_height,
                    framerate,
                    flip_method,
                    display_width,
                    display_height,
                )
            )
        
        window_title = "Face Tracking"
        face_cascade = cv2.CascadeClassifier(
            "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
        )
        #eye_cascade = cv2.CascadeClassifier(
        #   "/usr/share/opencv4/haarcascades/haarcascade_eye.xml"
        #)
        video_capture = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        if video_capture.isOpened():
            try:
                cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
                while True:
                    ret, frame = video_capture.read()
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

                    roi = ''
                    max_area = 0
                    area = 0
                    for (x, y, w, h) in faces:
                        cv2.rectangle(frame, (x, y), (x+w,y+h), (255, 0, 255), 2)  #  I put a box around your head ;P
                        #  now find the biggest face and report that one
                        area=h*w
                        if area>max_area:
                            max_area=area
                        # roi=img[y:y+h, x:x+w]
                        roi = [x, y, w, h]
                        
                    if roi != '':
                        x, y, w, h = int(roi[0]), int(roi[1]), int(roi[2]), int(roi[3])
                        
                        
                        center_box_w = int(x+(w/2)) 
                        roi_box_area = int(w*h)        

                        cv2.line(frame, (320, 0), (320, 480), (255, 0, 0), 2) #  blue vertical centerline
                        cv2.line(frame, (290, 0), (290, 480), (0, 255, 0), 2) #  left green vertical centerline
                        cv2.line(frame, (350, 0), (350, 480), (0, 255, 0), 2) #  right green vertical centerline
                
                        imH, imW, imC = frame.shape
                        imW = int(imW)
                        imH = int(imH)
                        imC =int(imC)
                        img_area = int(imW*imH)

                        roi_scale = img_area/roi_box_area
                        decision_lr = "HOLD"
                        decision_gonogo = "NOGO"
                        #twist.angular.z = float(0.00)
                        #twist.linear.x = float(0.00)

                        if roi_scale < 55:
                        
                        # send Twist vel.angular.z +Left -Right 0.24-ish
                            tolerance = 30    
                            if (int(center_box_w) < (int(imW/2) - int(tolerance))):
                                decision_lr = "LEFT"
                            #self.twist.angular.z = float(0.30)
                            if (int(center_box_w) > (int(imW/2) + int(tolerance))):
                                decision_lr = "RIGHT"
                            #self.twist.angular.z = float(-0.30)
                        
                        # send Twist vel.linuar.x +Fwd -Rev 0.24-ish
                            if ((roi_scale) < 30):
                                if ((roi_scale) > 25):
                                    decision_gonogo = "GO FWD"
                                #self.twist.linear.x = float(0.00)
                                if ((roi_scale) < 7):
                                    decision_gonogo = "MV 2 CLOSE"
                                #self.twist.linear.x = float(0.15)
                            
                        # tracking = "yes"                   
                        # print(x, y, w, h, imW, imH, imC, self.twist.linear.x, self.twist.angular.z,
                        # roi_box_area, img_area, roi_scale)
                        #print(self.twist.linear.x, self.twist.angular.z, roi_scale)    
                            print(roi_scale)
                        else:
                    # print("NO ROI DETECTED")
                    # tracking = "no"
                            decision_lr = "HOLD"
                            decision_gonogo = "NOGO"
                    #self.twist.linear.x = 0
                    #self.twist.angular.z = 0

                        cv2.line(frame, (0, 460), (640, 460), (0, 0, 0), 40) #  black horizontal background
                        cv2.putText(frame, "Choices " + decision_lr + " " + decision_gonogo, (40, 462), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

                    #for (x, y, w, h) in faces:
                    #    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    #    roi_gray = gray[y : y + h, x : x + w]
                    #    roi_color = frame[y : y + h, x : x + w]
                        #eyes = eye_cascade.detectMultiScale(roi_gray)
                        #for (ex, ey, ew, eh) in eyes:
                        #    cv2.rectangle(
                        #       roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 2
                        #   )
                    # Check to see if the user closed the window
                    # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
                    # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
                    if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                        cv2.imshow(window_title, frame)
                    else:
                        break
                    keyCode = cv2.waitKey(10) & 0xFF
                    # Stop the program on the ESC key or 'q'
                    if keyCode == 27 or keyCode == ord('q'):
                        break
            finally:
                video_capture.release()
                cv2.destroyAllWindows()
        else:
            print("Unable to open camera")


#--------------- MAIN LOOP
def main(args):
    #--- Create the object from the class we defined before
    fc = face_detect()
    
    #--- Initialize the ROS node
    rospy.init_node('face_detect', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        
    #--- In the end remember to close all cv windows
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)
    # face_detect()
