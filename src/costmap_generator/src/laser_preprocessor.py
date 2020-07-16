#!/usr/bin/env python
# -*- coding=utf-8 -*-
'''雷达数据处理（原始回波数据转换为基于机器人基底的栅格数据）'''
# license removed for brevity
import rospy
import cv2
import tf
import numpy as np
from sensor_msgs.msg import LaserScan
import matrix_processor
class LaserPtns2Matrix:
    '''
        雷达极坐标矩阵数据转化为2D笛卡尔坐标系中的矩阵数据
    '''
    def __init__(self,height=80,width=80,resolution=0.05,laser_frame='laser',base_frame='base_link',
                laser_filter=[0.05,3.0],scan_Topic='scan',weight=0.5):
        '''
            @height: 地图高度（像素）
            @width： 地图宽度
            @resolution： 地图分辨率
            @laser_frame:   激光雷达的坐标系名称
            @base_frame:    机器人基座的坐标系名称
            @laser_filter_intv: 激光雷达有效数据区间
            @weight:    雷达数据在生成地图中所占的权重
        '''
        self.height=height
        self.width=width
        self.laser_frame=laser_frame
        self.base_frame=base_frame
        self.laserScan=LaserScan()
        self.maxtrix=np.zeros((height,width),dtype=np.uint8)
        self.listener=tf.TransformListener()
        self.filter=laser_filter
        self.weight=weight
        rospy.Subscriber(scan_Topic,LaserScan,callback=self.Laser_Cb)
    def Laser_Cb(self,msg):
        self.laserScan=msg
    def Process_Laser_Frame(self,inflation=0.5):
        '''
            处理一祯雷达回波数据
        '''
        #重置costmap矩阵
        x,y,yaw=0.0,0.0,0.0
        self.maxtrix=np.zeros((self.height,self.width),dtype=np.uint8)
        #   1. 获得雷达与机器人基底的坐标变换关系
        try:
            (trans,rot)=self.listener.lookupTransform(self.base_frame,self.laser_frame,rospy.Time(0))
            x=trans[0];y=trans[1]
            _r,_p,yaw=tf.transformations.euler_from_quaternion(rot)
        except:
            '''
                坐标变换读取失败
                返回空的雷达数据矩阵
            '''
            rospy.logerr('[costmap-generator][laser-preprocessor]ubable to convert tf from base to laser')
            return self.maxtrix
        #   2. 扫描全部雷达回波点，映射到矩阵，并将矩阵栅格内指定元素置为100
        for i in range(len(self.laserScan.ranges)):
            #循环扫描
            if self.laserScan.ranges[i]>self.filter[1] or self.laserScan.ranges[i]<self.filter[0]:
                #丢弃阈值范围外的激光束
                continue
            #计算有效激光点映射到2D矩阵上的坐标数据
            #首先计算theta值，也就是某一条激光束与机器人激光雷达之间的夹角
            laser_Theta=self.laserScan.angle_min+i*self.laserScan.angle_increment
            #基于Theta值以及回波距离，计算相对于回波点相对于雷达的笛卡尔坐标（RealPosition）
            laser_Ptn_Real=[self.laserScan.ranges[i]*np.cos(laser_Theta),self.laserScan.ranges[i]*np.sin(laser_Theta)]
            #坐标转换为相对于机器人base_link的坐标
            base_Ptn_Real=matrix_processor.CoordinationTransfer2D(tf_Trans=[x,y,yaw],realPos=laser_Ptn_Real)
            #基于Realosition计算应当投射到矩阵的坐标
            matrix_X,matrix_Y=matrix_processor.RealXY2MatrixXY2D(realPos=base_Ptn_Real)
            #矩阵覆盖
            if 0<matrix_X<self.height and 0<matrix_Y<self.width:
                self.maxtrix[matrix_Y][matrix_X]=100
        #   3. 返回雷达矩阵
        return self.maxtrix
