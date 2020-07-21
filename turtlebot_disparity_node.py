#!/usr/bin/env python


import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class DisparityNode:

    def __init__(self):
        rospy.init_node('disparity', anonymous=True)
        self.loop_rate = rospy.Rate(1)
        self.leftCameraSub = rospy.Subscriber('/turtlebot3/left_camera/left_camera_info_raw', Image, self.leftCameraCallback, queue_size=10)
        self.rightCameraSub = rospy.Subscriber('/turtlebot3/right_camera/right_camera_info_raw', Image, self.rightCameraCallback, queue_size=10)
        self.disparityPub = rospy.Publisher('/disparity_image', Image, queue_size=10)
        self.leftCameraTimeStamp = rospy.Time()
        self.latestLeftCameraImage = None
        self.rightCameraTimeStamp = rospy.Time()
        self.latestRightCameraImage = None
        self.bridge = CvBridge()
        self.stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
        self.disparity = None

    def leftCameraCallback(self, msg):
        self.leftCameraTimeStamp = msg.header.stamp
        self.latestLeftCameraImage = self.rosImgToCV(msg)
        if abs(self.leftCameraTimeStamp.to_nsec() - self.rightCameraTimeStamp.to_nsec()) < 5000000:
            # if the camera images were taken within 5ms of each other, create disparity map
            self.disparity = self.stereo.compute(self.latestLeftCameraImage, self.latestRightCameraImage)
            self.publishDisparity()

    def rightCameraCallback(self, msg):
        self.rightCameraTimeStamp = msg.header.stamp
        self.latestRightCameraImage = self.rosImgToCV(msg)
        if abs(self.leftCameraTimeStamp.to_nsec() - self.rightCameraTimeStamp.to_nsec()) < 5000000:
            # if the camera images were taken within 5ms of each other, create disparity map
            self.disparity = self.stereo.compute(self.latestLeftCameraImage, self.latestRightCameraImage)
            self.publishDisparity()

    def rosImgToCV(self, image):
        cvImage = self.bridge.imgmsg_to_cv2(image, "bgr8")
        return cv2.cvtColor(cvImage, cv2.COLOR_BGR2GRAY)

    def publishDisparity(self):
        # bgrImage = cv2.cvtColor(self.disparity, cv2.COLOR_GRAY2BGR)
        image = self.bridge.cv2_to_imgmsg(self.disparity)
        self.disparityPub.publish(image)

    def start(self):
        try: 
            rospy.spin()
            while not rospy.is_shutdown():
                rate.sleep()
        except KeyboardInterrupt:
            print("shutting down")


if __name__ == '__main__':
    disparityNode = DisparityNode()
    disparityNode.start()
        
