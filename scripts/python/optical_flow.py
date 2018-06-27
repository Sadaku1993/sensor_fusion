#!/usr/bin/env python
#coding:utf-8

"""
author : Yudai Sadakuni

optical flow for ros

"""

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import math

class OpticalFlow(object):
    def __init__(self):
        self.image_sub = rospy.Subscriber("/image", Image, self.ImageCallback)
        self.pub = rospy.Publisher('/move', Bool, queue_size=10)
        self.image_pub = rospy.Publisher('/optical_flow', Image, queue_size=10)
        self.image_flag = False
        self.first_frame = True
        self.stop = Bool()
        self.bridge = CvBridge()

        # Shi-Tomasi法のパラメータ（コーナー検出用）
        self.ft_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7) 
        # Lucas-Kanade法のパラメータ（追跡用）
        self.lk_params = dict(winSize=(15,15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    def ImageCallback(self, msg):
        try:
            self.cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeerror as e:
            print (e)
        self.image_flag = True

    def first_processing(self):
        self.frame = self.cv_image
        self.gray1 = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        self.ft1 = cv2.goodFeaturesToTrack(self.gray1, mask=None, **self.ft_params)
        self.mask = np.zeros_like(self.frame)
        self.first_frame = False
        self.stop.data = False
        self.move = 0

    def optical_flow(self, threshold):
        # グレイスケールに変換
        gray2 = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        # Lucas-Kanade法でオプティカルフローの検出
        ft2, status, err = cv2.calcOpticalFlowPyrLK(self.gray1, gray2, self.ft1, None, **self.lk_params)
        # オプティカルフローを検出した特徴点を取得
        if np.sum(ft2) == None:
            print("Fail to find Optical Flow")
            self.first_processing()
            return 0;
            
        good1 = self.ft1[status == 1]
        good2 = ft2[status == 1]
        
        # 特徴点とオプティカルフローをフレーム・マスクに描画
        for i, (pt2, pt1) in enumerate(zip(good2, good1)):
            x1, y1 = pt1.ravel()
            x2, y2 = pt2.ravel()
            self.mask = cv2.line(self.mask, (x2, y2), (x1, y1), [0, 0, 200], 2)
            self.frame = cv2.circle(self.frame, (x2, y2), 5, [0, 0, 200], -1)
            if 4 < math.hypot(x2-x1, y2-y1):
                self.move += math.hypot(x2-x1, y2-y1)

        if threshold < self.move:
            print("moved:{:>4} threshold:{}".format(int(self.move), threshold))
            self.first_processing()
        else:
            print("stop :{:>4} threshold:{}".format(int(self.move), threshold))
            self.stop.data = True

        # フレームとマスクの論理積(合成)
        img = cv2.add(self.frame, self.mask)
        
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print(e)

        # 次のフレーム、ポイントの準備
        self.gray1 = gray2.copy()
        self.ft1 = good2.reshape(-1, 1, 2)
        self.frame = self.cv_image


    def main(self):
        rospy.init_node("optical_flow")
        threshold = rospy.get_param("~threshold", 500)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.image_flag:
                # print("Callback")
                if self.first_frame:
                    self.first_processing()
                else:
                    self.optical_flow(threshold)
            else:
                print("Waiting...")
            self.pub.publish(self.stop)
            rate.sleep()
        return 0

flow = OpticalFlow()
flow.main()
