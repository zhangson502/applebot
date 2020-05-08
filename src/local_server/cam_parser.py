#!/usr/bin/env python
# -*- coding=utf-8 -*-
import rospy
import cv2
import tf
import numpy as np
from sensor_msgs.msg import Image
from get_tf import Get_TF
from cv_bridge import CvBridge, CvBridgeError
class Cam_Parser:
    def __init__(self):
        self.img=[]
        self.cv_Bridge=CvBridge()
        #编码图像质量的参数
        self.img_quality = [int(cv2.IMWRITE_JPEG_QUALITY), rospy.get_param('~trans_quality',95)]
        self.img_Recv=rospy.Subscriber('/cam/monitor',Image,callback=self.Img_Callback)
    def Img_Callback(self,dat):
        self.img=dat
    def GetJpeg(self,cam_Name='',suffix='.jpg'):
        '''
            对收到的图像数据进行编码，返回jpeg数据
        '''
        if self.img==[]:#首先检验是否收到地图
            return 'NO MAP'
        else:
            img=self.cv_Bridge.imgmsg_to_cv2(self.img, "bgr8")
            img_jpeg=cv2.imencode(suffix,img)[1]
            encoded = np.array(img_jpeg)
            return encoded.tostring()
  
