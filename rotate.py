#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math

import cv2
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from stereo_msgs.msg import DisparityImage
from std_msgs.msg import Header

obstacle_distance = 100
roll = pitch = yaw = 0.0
target = 180
kp=0.07 # for wine bottle
x=[]
cnt = 0
cmd_flag=True
scan_distance = 0
def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    #print yaw

def res_callback (msg):
    global x
    x = msg.data.split(", ")
    print x


def disparityCallback(msg):
    global obstacle_distance
    f = msg.f
    b = msg.T
    disparityImage = bridge.imgmsg_to_cv2(msg.image, "32FC1")
    obstacleWindow = disparityImage[200:600,200:]
    closePixelCount = 0
    closeLeftCount = 0
    leftX = 1000000 # keep track of the x index closest to the left that's below clearance
    rightX = 0 # keep track of the x index closest to the right that's below clearance
    distances = []
    #print(f,b,"here")
        #x, y flipped?
        
    for idxX, row in enumerate(obstacleWindow):
        for idxY, d in enumerate(row):
            if d != 0:
                #print(b*f/np.amax(obstacleWindow))
                D = b*f/d
                #print(idxX,idxY, D,"D values!!!!")
                disparityImage[idxX][idxY] = D
                #print(distances,"Distance values!!!!")
                if D < obstacleClearance and D > 0:
                    print(idxX,idxY, D,"D values!!!!")
                    distances.append(D)
                    #print(distances, "distances!!!")
                    closePixelCount += 1
                    if idxY < 200:
                        closeLeftCount += 1
                    if idxX < leftX:
                        leftX = idxX
                    elif idxX > rightX:
                        rightX = idxX
        """
        cv2.imshow("image", obstacleWindow)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        """

        # print("number of close pixels: " + str(closePixelCount))
        #print(closePixelCount, "pixelcount!!!!!!!")
    if closePixelCount > pixelCountThreshold:
        print("here!!!!!!!!!!")
        obstacle_distance = calculate_obstacle_width(rightX, sum(distances)/float(len(distances)))
    #obstaclePub.publish(obstacleMessage)
        #print('distance')
        #print(obstacle_distance)    

def calculate_obstacle_width(rightX, obstacle_distance):
    targetX = rightX + 100 # TODO: for now, we have clearance of 100 pixels. this can be improved.
    print("target X value: " + str(targetX))
    centerX = 200 # TODO: this is hardcoded value; our camera yields 800x800 pixel img and we have an obstacle window that starts at 200
    if targetX <= centerX:
           # no avoidance necessary
        return 0
    width_in_pixels = targetX - centerX
    width_in_meters = obstacle_distance * (width_in_pixels/float(800)) * 1.3962634 #TODO: hardcoded values: camera resolution, horizontal FOV
    print("width in meters: " + str(width_in_meters))
    print("obstacle distance: " + str(obstacle_distance))
    return width_in_meters

def scan_callback(msg):
    d1 = []
    d2 = []
    global scan_distance
    scan_distance1 = 0
    scan_distance2 = 0
    for i in msg.ranges[-50:]:
        if i < 1000:
            d1.append(i)
    for i in msg.ranges[:50]:
        if i < 1000:
            d2.append(i)    
    if len(d1) != 0:
        scan_distance1 = sum(d1)/len(d1)
    if len(d2) != 0:
        scan_distance2 = sum(d2)/len(d2)
    scan_distance = scan_distance1+scan_distance2
    print(scan_distance, "laser_distances!!!")

pixelCountThreshold = 5
obstacleClearance = 0.2
bridge = CvBridge()
rospy.init_node('rotate_robot')

sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
sub_res = rospy.Subscriber('/result', String, res_callback)
sub_laser = rospy.Subscriber('/scan', LaserScan, scan_callback)
disparitySub = rospy.Subscriber('/turtlebot3/stereo/disparity', DisparityImage, disparityCallback)

pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

r = rospy.Rate(10)
command =Twist()

while not rospy.is_shutdown():
    target_rad = target*math.pi/180
    if cmd_flag:
        for item in x:
            if item != 'wine bottle':
                command.angular.z = kp * (target_rad-yaw)
            else:
                cnt = cnt + 1
                command.angular.z = 0
                if cnt > 3:
                    cmd_flag=False
        print("ang_z={} taeget={} current:{}".format(command.angular.z,target_rad,yaw))
    pub.publish(command)
    if not cmd_flag:
        print("Found target!")
        if scan_distance > 0.5: 
            command.linear.x = 0.1
            command.angular.z = 0
        else:
            command.linear.x = 0
    r.sleep()
