#!usr/bin/env python
import rospy
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetection
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from math import cos, sin, tan, radians, degrees
import numpy as np
#import RPi.GPIO as GPIO
#import time

right_forward_pin=13

right_backward_pin=15

left_forward_pin=16
left_backward_pin=18

#GPIO.setmode(GPIO.BOARD)
#GPIO.setwarnings(False)
#GPIO.setup(right_forward_pin, GPIO.OUT, initial=0)
#GPIO.setup(right_backward_pin, GPIO.OUT, initial=0)
#GPIO.setup(left_forward_pin, GPIO.OUT, initial=0)
#GPIO.setup(left_backward_pin, GPIO.OUT, initial=0)
tz = 0.7
f1, px,f2, py =949.328667,351.693784, 954.021101, 201.703805
 
     

def callback(data):

    if len(data.detections)>0:
    	x = data.detections[0].pose.pose.pose.position.x
        y = data.detections[0].pose.pose.pose.position.y
        rot=data.detections[0].pose.pose.pose.orientation
        
        angles=euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        yaw = angles[2]
        r = degrees(yaw)
        tagID = data.detections[0].id
        print(r)
    	#print("value of x,y,r,id = %s%s%s%s", x,y,r,tagID)
    	#print(tagID)
        
    	if tagID == (7,):
    	   tx = 0
    	   ty = 0
           print("detecting tag 7")
    	elif tagID == (1,):
    	   tx = 0.1
    	   ty = 0.1
           print("detecting tag 1")
    	elif tagID == (24,):
    	   tx = 0.1
    	   ty = -0.1
    	   print("detecting tag 24")

    	elif tagID == (25,):
    	   tx = -0.1
    	   ty = 0.1  
           print("detecting tag 25")
    	elif tagID == (0,):
    	   tx = -0.1
    	   ty = -0.1        
           print("detecting tag 0")
        
        cx = (x-tx*f1*cos(r)+ty*sin(r)*f1-px*tz)/f1
        cy = (y-tx*f2*sin(r)-ty*f2*cos(r)-tz*py)/f2
        print("tx, ty= %s%s",cx,cy)
        
        #print(lx,ly)
          
    else:
	print("No tag detected")


def listner():
    rospy.init_node('listner',anonymous=True)
    rospy.Subscriber("/tag_detections",AprilTagDetectionArray, callback)
   
    rospy.spin()
    rospy.loginfo("top loop %s", )


    
if __name__ == '__main__':
    listner()


