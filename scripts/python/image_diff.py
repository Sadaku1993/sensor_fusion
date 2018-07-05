#!/usr/bin/env python
#coding:utf-8

import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import cv2
import numpy as np
import math

class DiffImage(object):
    def __init__(self):
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image   , self.ImageCallback)
        self.bool_sub  = rospy.Subscriber("/stop" , Bool    , self.BoolCallback)
        self.odom_sub  = rospy.Subscriber("/odom" , Odometry, self.OdomCallback)

        self.pub = rospy.Publisher('/move', Bool, queue_size=10)
        self.image_pub_compress = rospy.Publisher('/diff_image/compressed', CompressedImage)
        self.move = Bool()
       
        self.stop_flag = False
        self.first_flag = True
        
        self.velocity = 0
        self.cv_image = np.zeros((1,1,3), np.uint8)
        
        self.count = 0

    
    def ImageCallback(self, msg):
        try:
            self.cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
            
            if self.stop_flag and self.velocity < 0.01:
                print("Arrive Save Point ---> Start Diff Image")
                self.DiffImage()
            else:
                print("Waiting Stop")
                self.Reset()
           
        except:
            print(CvBridgeerror)

    def BoolCallback(self, msg):
        self.stop_flag = msg

    def OdomCallback(self, msg):
        self.velocity = ( msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2 )**0.5

    def DiffImage(self):
        if self.first_flag:
            print("----first Image")
            self.back_img = np.zeros_like(self.cv_image, np.float32)
            self.first_flag = False
            
        print("----Count : {}".format(self.count))
        f_img = self.cv_image.astype(np.float32)
        diff_img = cv2.absdiff(f_img, self.back_img)

        diff_img_sum = np.sum(diff_img, axis=2)
        pixcel_diff = diff_img_sum > 100
 
        rate = float(np.sum(pixcel_diff))/(f_img.shape[0]*f_img.shape[1])
        print("----Diff Pixcel:{:>4} Image Pixcel:{:>4} Rate:{:>.4}".format(np.sum(pixcel_diff), f_img.shape[0]*f_img.shape[1], rate))
 
        booling_img = cv2.cvtColor(diff_img, cv2.COLOR_RGB2GRAY) > 50
        tmp_img = np.ones_like(booling_img, np.float32) * booling_img
        moving_img = self.cv_image * cv2.cvtColor(tmp_img, cv2.COLOR_GRAY2RGB)
        cv2.accumulateWeighted(f_img, self.back_img, 0.1)
 
 
        if self.count < self.diff_count:
            print("----CalcTime")
            self.move.data = False
            self.pub.publish(self.move)
        elif self.diff_count < self.count and self.threshold < rate:
            print("----World is Modeing")
            self.move.data = True
            self.pub.publish(self.move)
            self.Reset()
        else:
            print("----World is Stop")
            self.move.data = False
            self.pub.publish(self.move)

        compress = CompressedImage()
        compress.header.stamp = rospy.Time.now()
        compress.format = "jpeg"
        compress.data = np.array(cv2.imencode('.jpg', moving_img)[1]).tostring()
        self.image_pub_compress.publish(compress)
 
        self.count+=1


    def Reset(self):
        self.count = 0
        self.back_img = np.zeros_like(self.cv_image, np.float32)
        self.first_flag = True
        self.move.data = False


    def main(self):
        print("start")
        
        rospy.init_node("diff_image")
        self.diff_count = rospy.get_param("~diff_count", 20)
        self.threshold = rospy.get_param("`threshold", 0.3)

        rospy.spin()

        return 0


if __name__ == "__main__":

    diff_image = DiffImage()
    diff_image.main()
