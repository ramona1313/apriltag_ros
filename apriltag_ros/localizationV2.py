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
roll = pitch = yaw = 0.0
 
     

def callback(data):

    if len(data.detections)>0:
    	x = data.detections[0].pose.pose.pose.position.x
        y = data.detections[0].pose.pose.pose.position.y
        rot=data.detections[0].pose.pose.pose.orientation
        
        (roll, pitch, yaw)=euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        #r = angles[2]
        tagID = data.detections[0].id
        print("yaw_angle = %s",degrees(yaw))
        #print(degrees(yaw))
    	print("value of x,y = %s%s", x*10,y*10)
    	#print(tagID)
        
    	
          
    else:
	print("No tag detected")


def listner():
    rospy.init_node('listner',anonymous=True)
    rospy.Subscriber("/tag_detections",AprilTagDetectionArray, callback)
   
    rospy.spin()
    rospy.loginfo("top loop %s", )


    
if __name__ == '__main__':
    listner()


