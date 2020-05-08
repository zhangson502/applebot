#!/usr/bin/env python
# -*- coding=utf-8 -*-
import rospy
import cv2
import tf
import numpy as np
from nav_msgs.msg import OccupancyGrid
from get_tf import Get_TF
class Map_Parser:
    def __init__(self):
        self.gotten_Map=False
        self.ros_Map=[]
        self.map=None
        self.ros_Map_Meta=None
        self.pos_listener=tf.TransformListener()
        #编码图像质量的参数
        self.img_quality = [int(cv2.IMWRITE_JPEG_QUALITY), rospy.get_param('~trans_quality',95)]
        self.map_Recv=rospy.Subscriber('/map',OccupancyGrid,callback=self.Map_Callback)
    def Map_Callback(self,dat):
        self.ros_Map=dat
        self.ros_Map_Meta=dat.info
    def DrawRobot(self):
        '''
            在地图上画机器人
        '''
        self.posList=Get_TF(listener=self.pos_listener)
        #计算像素位置
        real_x=self.posList[0]-self.ros_Map.info.origin.position.x
        real_y=self.posList[1]-self.ros_Map.info.origin.position.y
        pix_x=int(real_x/self.ros_Map.info.resolution)
        pix_y=int(real_y/self.ros_Map.info.resolution)
        self.map=cv2.circle(self.map,(pix_x,pix_y),5,(0,255,0),-1)      
    def Coding2Jpeg(self,suffix='.jpg'):
        '''
            对收到的地图数据进行编码，返回jpeg数据
        '''
        try:
            if self.ros_Map==[]:#首先检验是否收到地图
                return ''
            else:
                height=self.ros_Map_Meta.height
                width=self.ros_Map_Meta.width
                print height,width
                self.map=np.zeros((height,width,3),dtype=np.uint8)
                for h in range(height):
                    self.map[h,:,0]=self.ros_Map.data[h*width:(h+1)*width]
                self.map[:,:,1]=self.map[:,:,2]=self.map[:,:,0]
                self.map=255-self.map
                self.DrawRobot()
                img_jpeg=cv2.imencode(suffix,self.map)[1]
                encoded = np.array(img_jpeg)
                return encoded.tostring()
        except:
            rospy.logerr('local server unable to transfer http map!')
  
