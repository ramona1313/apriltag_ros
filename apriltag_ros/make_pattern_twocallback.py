#!usr/bin/env python
import rospy
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetection
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from math import degrees, sqrt, atan2, pi
#import RPi.GPIO as GPIO
import time
#import robot

#robot = robot.Robot()
xy = [(0.35,0.35),(0.35,-0.35),(-0.70,0.35),(-0.70,-0.30),(0,0)]
#x_goal,y_goal = (0,0)

global i
i = [0]

def callback(data):
    
    if len(data.detections)>0:
    	x = data.detections[0].pose.pose.pose.position.x
        y = data.detections[0].pose.pose.pose.position.y
        rot = data.detections[0].pose.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        
        if i[0] < len(xy):
           x = data.detections[0].pose.pose.pose.position.x
           y = data.detections[0].pose.pose.pose.position.y
           rot = data.detections[0].pose.pose.pose.orientation
           (roll, pitch, theta) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
           x_goal,y_goal=xy[i[0]]
           print("first",x,y)
           #print (x_goal, y_goal, i[0])
           x_diff=x_goal-x
           y_diff=y_goal-y
           alpha=atan2((y_diff),(x_diff))
   
           distance=sqrt((x_diff)**2+(y_diff)**2)
      
           alpha = alpha % (2*pi)
           theta = theta % (2*pi)
           error_rad = alpha - theta
        
           #print ("dist= %s ,  rad= %s", distance, error_rad)
           if distance > 0.1:
              if abs(error_rad)>0.2:
                  #print (degrees(theta))
                  #print("pwm=",abs(error_rad)*2+10)  
               
                  if alpha > theta:
                      robot.right(((abs(error_rad)-0.2)/73.5)+0.09)
                     # print(((abs(error_rad)*2-20)/50))
                     # print("theta=%s, alpha=%s, turning right", degrees(theta), degrees(alpha))
                  else: 
                      robot.left(((abs(error_rad)-0.2)/73.5)+0.09)
                      #print("theta=%s, alpha=%s,turning left",degrees(theta),degrees(alpha))

              else: 
                   #print ("going forward")
                   robot.forward(min((distance+0.3)/4,0.1))
           else:
               #print distance
               robot.stop()
               i[0]=i[0]+1
        print(x,y,degrees(theta))
    else:
	print("No tag detected")
        
        robot.stop()

def callback1(data):
    
    if len(data.detections)>0:
    	x = data.detections[0].pose.pose.pose.position.x
        y = data.detections[0].pose.pose.pose.position.y
        rot = data.detections[0].pose.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        
        if i[0] < len(xy):
           x = data.detections[0].pose.pose.pose.position.x
           y = data.detections[0].pose.pose.pose.position.y+0.3653
           rot = data.detections[0].pose.pose.pose.orientation
           (roll, pitch, theta) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
           x_goal,y_goal=xy[i[0]]
           print("second",x,y)
           #print (x_goal, y_goal, i[0])
           x_diff=x_goal-x
           y_diff=y_goal-y
           alpha=atan2((y_diff),(x_diff))
   
           distance=sqrt((x_diff)**2+(y_diff)**2)
      
           alpha = alpha % (2*pi)
           theta = theta % (2*pi)
           error_rad = alpha - theta
        
           #print ("dist= %s ,  rad= %s", distance, error_rad)
           if distance > 0.1:
              if abs(error_rad)>0.2:
                  #print (degrees(theta))
                  #print("pwm=",abs(error_rad)*2+10)  
               
                  if alpha > theta:
                      robot.right(((abs(error_rad)-0.2)/73.5)+0.09)
                     # print(((abs(error_rad)*2-20)/50))
                     # print("theta=%s, alpha=%s, turning right", degrees(theta), degrees(alpha))
                  else: 
                      robot.left(((abs(error_rad)-0.2)/73.5)+0.09)
                      #print("theta=%s, alpha=%s,turning left",degrees(theta),degrees(alpha))

              else: 
                   #print ("going forward")
                   robot.forward(min((distance+0.3)/4,0.1))
           else:
               #print distance
               robot.stop()
               i[0]=i[0]+1
        print(x,y,degrees(theta))
    else:
	print("No tag detected")
        
        robot.stop()




         
        
         
def listner():
    rospy.init_node('listner',anonymous=True)
    rospy.Subscriber("/detection1/tag_detections",AprilTagDetectionArray, callback)
    rospy.Subscriber("/detection2/tag_detections",AprilTagDetectionArray, callback1)
    rospy.spin()
    rospy.loginfo("top loop %s", )


if __name__ == '__main__':
    listner()


