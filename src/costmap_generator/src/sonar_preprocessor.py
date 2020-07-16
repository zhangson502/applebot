#!/usr/bin/env python
# -*- coding=utf-8 -*-
'''声呐数据处理（原始回波数据转换为基于机器人基底的栅格数据）'''
# license removed for brevity

import rospy
import cv2
import tf
import numpy as np
from sensor_msgs.msg import LaserEcho
import matrix_processor
class SonarPtns2Matrix:
    '''
        雷达极坐标矩阵数据转化为2D笛卡尔坐标系中的矩阵数据
    '''
    def __init__(self,height=80,width=80,resolution=0.05,sonar_nums=3,
                sonar_filter=[0.05,3.0],sonar_Topic='sonar',sonar_frame_prefix='sonar',base_frame='base_link',weight=0.3):
        '''
            @height: 地图高度（像素）
            @width： 地图宽度
            @resolution： 地图分辨率
        '''
        self.height=height
        self.width=width
        self.sonar_Nums=sonar_nums
        self.sonar_frame_prefix=sonar_frame_prefix
        self.base_frame=base_frame
        self.sonarEcho=LaserEcho()
        self.maxtrix=np.zeros((height,width),dtype=np.uint8)
        self.listener=tf.TransformListener()
        self.filter=laser_filter
        rospy.Subscriber(scan_Topic,LaserEcho,callback=self.Sonar_Cb)
    def Sonar_Cb(self,msg):
        self.sonarEcho=msg
    def Process_Sonar_Frame(self,inflation=0.5):
        '''
            处理一祯雷达回波数据
        '''
        #重置costmap矩阵
        x,y,yaw=0.0,0.0,0.0
        self.maxtrix=np.zeros((self.height,self.width),dtype=np.uint8)
        #超声波硬件位置配置列表
        #   1. 获得所有超声波
        for i in range(self.sonar_Nums):
            #循环读取所有超声波硬件的排布位置
            try:
                (trans,rot)=self.listener.lookupTransform(self.base_frame,self.sonar_frame_prefix+str(i),rospy.Time(0))
                x=trans[0];y=trans[1]
                _r,_p,yaw=tf.transformations.euler_from_quaternion(rot)
                #   2. 扫描超声波回波点，映射到矩阵，并将矩阵栅格内指定元素置为100
                sonarCoordBPos=[self.sonarEcho.echoes[i],0]
                #   3. 超声波坐标变换
                sonarCoordAPos=matrix_processor.CoordinationTransfer2D([x,y,yaw],sonarCoordBPos)
                #   4. 映射到矩阵栅格坐标系
                matrix_X,matrix_Y=matrix_processor.RealXY2MatrixXY2D(realPos=sonarCoordAPos)
                #   5. 贴图
                if 0<matrix_X<self.height and 0<matrix_Y<self.width:
                    self.maxtrix[matrix_Y][matrix_X]=100
            except:
                '''
                    坐标变换读取失败
                    返回空的雷达数据矩阵
                '''
                rospy.logerr('[costmap-generator][laser-preprocessor]ubable to convert tf from base to laser')
                continue
        return self.maxtrix
            
        
