#!/usr/bin/env python
# -*- coding=utf-8 -*-
import struct
import rospy
import tf
import math
from nav_msgs.msg import Odometry
class Encoder:
    def __init__(self):
        self.l_Data=0
        self.r_Data=0
        self.v=0.0
        self.vth=0.0
        self.odom=Odometry()
        self.odom.header.frame_id='/odom'
        self.odom.child_frame_id='/base_link'
        self.rotation=0.0
        self.STATIC_POS_COEF=[0.00001,0,0,0,0,0,
							0,0.00001,0,0,0,0,
							0,0,0.00001,0,0,0,
							0,0,0,0.0001,0,0,
							0,0,0,0,0.0001,0,
							0,0,0,0,0,0.0001]
        self.DYNAMIC_POS_COEF=[0.01,0,0,0,0,0,
							0,0.01,0,0,0,0,
							0,0,0.01,0,0,0,
							0,0,0,0.01,0,0,
							0,0,0,0,0.01,0,
							0,0,0,0,0,0.01]
        self.STATIC_TWIST_COEF=[0.00001,0,0,0,0,0,
                            0,0.00001,0,0,0,0,
                            0,0,0.00001,0,0,0,
                            0,0,0,0.0001,0,0,
                            0,0,0,0,0.0001,0,
                            0,0,0,0,0,0.0001]
        self.DYNAMIC_TWIST_COEF=[0.05,0,0,0,0,0,
                            0,0.01,0,0,0,0,
                            0,0,0.01,0,0,0,
                            0,0,0,0.01,0,0,
                            0,0,0,0,0.01,0,
                            0,0,0,0,0,0.01]
        self.odom_Pub=rospy.Publisher('/odom',Odometry,queue_size=5,latch=True)
    def Refresh_Encoder(self,enc_L,enc_R,pulse_per_meter=3800.0,width=0.29):
        tmp_l=(enc_L-self.l_Data)/pulse_per_meter
        tmp_r=(enc_R-self.r_Data)/pulse_per_meter
        self.odom.header.stamp=rospy.Time.now()
        self.l_Data=enc_L
        self.r_Data=enc_R
        self.rotation+=(tmp_r-tmp_l)/width
        #设置odometry的orientation
        [self.odom.pose.pose.orientation.x,self.odom.pose.pose.orientation.y,self.odom.pose.pose.orientation.z,self.odom.pose.pose.orientation.w]\
                                =tf.transformations.quaternion_from_euler(0,0,self.rotation)
        self.odom.pose.pose.position.x+=(tmp_l+tmp_r)*math.cos(self.rotation)/2
        self.odom.pose.pose.position.y+=(tmp_l+tmp_r)*math.sin(self.rotation)/2
        if tmp_l==0 and tmp_r==0: self.odom.pose.covariance=self.STATIC_POS_COEF
        else: self.odom.pose.covariance=self.DYNAMIC_POS_COEF
        self.odom_Pub.publish(self.odom)

        
        