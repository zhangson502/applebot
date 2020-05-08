#!/usr/bin/env python
#coding=utf-8
import rospy
import tf
import cv2
import thread as thread
import numpy as np
from basic_math_2D import Point2D
from sys import exit
from nav_msgs.msg import OccupancyGrid as Map
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal
from sensor_msgs.msg import LaserScan as Laser
from nav_msgs.msg import Odometry


class ROS_Parser:
    def __init__(self,map_size=(1,1)):
        '''
            ROS消息接口
        '''
        self.manual_Pub=rospy.Publisher('cmd_vel',Twist,queue_size=5)
        self.path_Pub=rospy.Publisher('user_path',Path,queue_size=5)
        self.map_Sub=rospy.Subscriber('/slamware_ros_sdk_server_node/map',Map,callback=self.Map_C)
        self.local_Map=rospy.Subscriber('/move_base/local_costmap/costmap',Map,callback=self.LMap_C)
        self.odom_Sub=rospy.Subscriber('/slamware_ros_sdk_server_node/odom',Odometry,callback=self.Odom_Cb)

        '''
            数据
        '''
        self.map=np.zeros(map_size)
        self.buffer=None
        self.calib=Point2D()
        self.origin=Point2D()
        self.size=1.0
        self.W=int(0)
        self.H=int(0)
        self.resolution=0.05
        self.robot_Odometry=[0,0,0]
        self.ctrl=Twist()
    def Manual(self,v=0,vth=0):
        print v,vth
        self.ctrl.angular.z=vth
        self.ctrl.linear.x=v
        self.manual_Pub.publish(self.ctrl)
    def Map_C(self,data):
        self.H=data.info.height
        self.W=data.info.width
        self.resolution=data.info.resolution
        self.origin.x=int(data.info.origin.position.x/self.resolution)
        self.origin.y=int(data.info.origin.position.y/self.resolution)
        #载入地图  
        buffer=np.zeros((self.H,self.W),dtype=np.uint8)
        for y in range(self.H):
            for x in range(self.W):
                if data.data[y*self.W+x]==-1: buffer[y][x]=10
                elif data.data[y*self.W+x]==0: buffer[y][x]=255
                else: buffer[y][x]=0
        self.map=buffer
        del buffer
    def LMap_C(self,data):
        H=data.info.height
        W=data.info.width
        resolution=data.info.resolution
        #origin.x=int(data.info.origin.position.x/resolution)
        #origin.y=int(data.info.origin.position.y/resolution)
    def Odom_Cb(self,data):
        self.robot_Odometry=[data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.orientation.z]

                
        
