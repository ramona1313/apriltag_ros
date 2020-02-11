#!usr/bin/env python
import rospy
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetection
from geometry_msgs.msg import PoseWithCovarianceStamped
import RPi.GPIO as GPIO
import time

right_forward_pin=13
right_backward_pin=15

left_forward_pin=16
left_backward_pin=18

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(right_forward_pin, GPIO.OUT, initial=0)
GPIO.setup(right_backward_pin, GPIO.OUT, initial=0)
GPIO.setup(left_forward_pin, GPIO.OUT, initial=0)
GPIO.setup(left_backward_pin, GPIO.OUT, initial=0)

def callback1(data):

    if len(data.detections)>0:
    	x = data.detections[0].pose.pose.pose.position.x

    	print("value of x = %s", x)
        
        go_forward()

        if x > 0.2:
           turn_left()
           if y > 0.2:
              turn_left()
              if x < 0:
                 stop()    
    else:
	print("No tag detected")

def callback2(data):

    if len(data.detections)>0:
    	x = data.detections[0].pose.pose.pose.position.x 

    	print("value of x = %s", x)
        
        go_forward()

        if x > 0.2:
           turn_left()
           if y > 0.2:
              turn_left()
              if x < 0:
                 stop()    
    else:
	print("No tag detected")	

def listner():
    rospy.init_node('listner',anonymous=True)
    rospy.Subscriber("/detection1/tag_detections",AprilTagDetectionArray, callback1)
    rospy.Subscriber("/detection2/tag_detections",AprilTagDetectionArray, callback2)
    rospy.spin()
    rospy.loginfo("top loop %s", )

def go_forward():
    GPIO.output(right_forward_pin, 1)
    GPIO.output(left_forward_pin, 1)
    GPIO.output(right_backward_pin, 0)
    GPIO.output(left_backward_pin, 0)
    print("forward")
def turn_left():
    GPIO.output(right_forward_pin, 1)
    GPIO.output(left_forward_pin, 0)
    GPIO.output(right_backward_pin, 0)
    GPIO.output(left_backward_pin, 1)
    print("Left")
def stop():
    GPIO.output(right_forward_pin, 0)
    GPIO.output(left_forward_pin, 0)
    GPIO.output(right_backward_pin, 0)
    GPIO.output(left_backward_pin, 0)
    print("stop")

    
if __name__ == '__main__':
    listner()


