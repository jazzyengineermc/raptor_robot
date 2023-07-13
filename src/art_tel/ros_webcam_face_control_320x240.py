#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

import os


#--- Define our Class
class face_control:

    def __init__(self):
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
        #--- Publishers
        # self.image_pub = rospy.Publisher("image_topic",Image,queue_size=1)
        self.rcvel_pub2 = rospy.Publisher("raptor/cmd_vel", Twist, queue_size=1)
        self.rcvel_pub1 = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.rcvel_pub = ''
        # self.tracking_pub = rospy.Publisher("raptor/tracking", String, queue_size=1)

        #--- Subscribers
        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        self.face_cam_sub = rospy.Subscriber("raptor/control", String, self.doidoit)

        #--- Twist stuffs
        self.twist = Twist()
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0

        self.cam = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        success, img = self.cam.read()

        if success:
            self.callback(img)


    def doidoit(self,izzy):
        if str(izzy) == 'data: "face"':
            self.rcvel_pub = self.rcvel_pub1
        else:
            self.rcvel_pub = self.rcvel_pub2

        print(izzy)



    # def callback(self,data):  #--- Callback function
    def callback(self,img):
        #--- Read the frame and convert it using bridge
    #    try:
    #        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
     #   except CvBridgeError as e:
    #        print(e)
        ret, cv_image = self.cam.read()

        #--- If a valid frame is received, do stuffs
        (rows,cols,channels) = cv_image.shape
        if cols > 20 and rows > 20:
            # cv2_base_dir = os.path.dirname(os.path.abspath(cv2.__file__))
            # haar_face_model = os.path.join(cv2_base_dir, 'data/haarcascade_frontalface_default.xml')
            # Follow this to make jazzy face.xml file :D
            # https://medium.com/@vipulgote4/guide-to-make-custom-haar-cascade-xml-file-for-object-detection-with-opencv-6932e22c3f0e
            haar_face_model = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
            face_model = cv2.CascadeClassifier(haar_face_model)
            # haar_person_model = os.path.join(cv2_base_dir, 'data/haarcascade_upperbody.xml')
            # person_model = cv2.CascadeClassifier(haar_person_model)
            img = cv_image

            imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            faces = face_model.detectMultiScale(imgGray, 1.1, 4, minSize=(20, 20))
            #  people = person_model.detectMultiScale(imgGray, 1.1, 4)

            roi = ''
            max_area = 0
            area = 0
            for (x, y, w, h) in faces:
                cv2.rectangle(img, (x, y), (x+w,y+h), (255, 0, 255), 2)  #  I put a box around your head ;P
                #  now find the biggest face and report that one
                area=h*w
                if area>max_area:
                    max_area=area
                    # roi=img[y:y+h, x:x+w]
                    roi = [x, y, w, h]

            #  for (x, y, w, h) in people:
            #      cv2.rectangle(img, (x, y), (x+w,y+h), (255, 255, 255), 2)  #  I put a box around your body ;P
            #      #  Assign that face to a tracked person, use person box for variable decisions

            if roi != '':
                x, y, w, h = int(roi[0]), int(roi[1]), int(roi[2]), int(roi[3])
                    
                    
                center_box_w = int(x+(w/2)) 
                roi_box_area = int(w*h)        

                cv2.line(img, (160, 0), (160, 240), (255, 0, 0), 2) #  blue vertical centerline
                cv2.line(img, (140, 0), (140, 240), (0, 255, 0), 2) #  left green vertical centerline
                cv2.line(img, (180, 0), (180, 240), (0, 255, 0), 2) #  right green vertical centerline
            
                imH, imW, imC = img.shape
                imW = int(imW)
                imH = int(imH)
                imC =int(imC)
                img_area = int(imW*imH)

                roi_scale = img_area/roi_box_area
                decision_lr = "HOLD"
                decision_gonogo = "NOGO"
                self.twist.angular.z = float(0.00)
                self.twist.linear.x = float(0.00)

                if roi_scale < 55:
                    
                    # send Twist vel.angular.z +Left -Right 0.24-ish
                    tolerance = 30    
                    if (int(center_box_w) < (int(imW/2) - int(tolerance))):
                        decision_lr = "LEFT"
                        self.twist.angular.z = float(0.30)
                    if (int(center_box_w) > (int(imW/2) + int(tolerance))):
                        decision_lr = "RIGHT"
                        self.twist.angular.z = float(-0.30)
                    
                    # send Twist vel.linuar.x +Fwd -Rev 0.24-ish
                    if ((roi_scale) < 30):
                        if ((roi_scale) > 25):
                            decision_gonogo = "GO FWD"
                            self.twist.linear.x = float(0.00)
                        if ((roi_scale) < 7):
                            decision_gonogo = "MV 2 CLOSE"
                            self.twist.linear.x = float(0.15)
                        
                    # tracking = "yes"                   
                    # print(x, y, w, h, imW, imH, imC, self.twist.linear.x, self.twist.angular.z,
                    # roi_box_area, img_area, roi_scale)
                    print(self.twist.linear.x, self.twist.angular.z, roi_scale)    
            else:
                # print("NO ROI DETECTED")
                # tracking = "no"
                decision_lr = "HOLD"
                decision_gonogo = "NOGO"
                self.twist.linear.x = 0
                self.twist.angular.z = 0

        cv2.line(img, (0, 230), (320, 230), (0, 0, 0), 20) #  black horizontal background
        cv2.putText(img, "Choices " + decision_lr + " " + decision_gonogo, (20, 232), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

        #--- Optional: show the image on a window (comment this for the Raspberry Pi)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        #--- Publish the modified frame to a new topic
        # try:
        #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        # except CvBridgeError as e:
        #     print(e)

        #--- Publish to raptor/cmd_vel
        try:
            # self.doidoit('no')
            if self.rcvel_pub == '':
                self.rcvel_pub = self.rcvel_pub2
            #self.rcvel_pub.publish( self.twist )
        except CvBridgeError as e:
            print(e)

        #--- Publish to raptor/tracking
        # try:
        #     self.tracking_pub.publish( tracking )
        # except CvBridgeError as e:
        #     print(e)

#--------------- MAIN LOOP
def main(args):
    #--- Create the object from the class we defined before
    fc = face_control()
    
    #--- Initialize the ROS node
    rospy.init_node('face_control', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        
    #--- In the end remember to close all cv windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
        main(sys.argv)
