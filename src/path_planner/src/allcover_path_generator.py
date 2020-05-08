#!/usr/bin/env python
# -*- coding=utf-8 -*-
import rospy
import json
import tf
import cv2
import time
import math
import numpy as np
from std_msgs.msg import String,UInt64
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose,PoseArray,PoseStamped,Point
from nav_msgs.msg import OccupancyGrid
from slamware_ros_sdk.msg import MoveToLocationsRequest as MoveTo
import thread as thread
import calculation


class AllCover_Path_Planner:
    def __init__(self):
        rospy.Subscriber('/clean_area',PoseArray,callback=self.Start_Clean)
        rospy.Subscriber('/map',OccupancyGrid,callback=self.Load_Map)
        self.path_Pub=rospy.Publisher('area_path',Path,queue_size=1,latch=True)
        self.slamware_path_Pub=rospy.Publisher('move_to_locations',MoveTo,queue_size=5,latch=True)
        #配置参数
        self.vehicle_max_Diameter=rospy.get_param('vehicle_diameter',0.7)
        self.vehicle_cover_Width=rospy.get_param('vehicle_cover_width',0.6)
        self.step_length=rospy.get_param('step_length',0.25)
        self.min_Cover_Percentage=rospy.get_param('min_allcover_percentage',0.8)

        self.ros_Map=OccupancyGrid()
        self.clean_Matrix=None  #全覆盖区域的矩阵标注
        self.blank_Size=0       #未覆盖区域的面积
        self.p0=[0,0]           #全覆盖区域在地图上所处的角点1
        self.p1=[0,0]           #全覆盖区域在地图上所处的角点2
        self.erode_Pixel=0  #预处理腐蚀像素值
        self.width_Pixel=0  #机器人宽度（像素值）
        self.step_Pixel=0   #路径计算步长（像素值）
        self.X=0    #当前计算中的焦点
        self.Y=0    #当前计算中的焦点
        self.dir='e'    #前进方向
        self.path=[]    #生成的路线
    def Load_Map(self,dat):
        '''
            更新地图
        '''
        self.ros_Map=dat
    def Resolve_Map(self):
        '''
            解析全局地图
            将全局ROS地图转换为Matrix（self.clean_Matrix）
        '''
        H=self.ros_Map.info.height
        W=self.ros_Map.info.width
        #载入地图  
        buffer=np.zeros((H,W),dtype=np.uint8)
        for y in range(H):
            for x in range(W):
                if self.ros_Map.data[y*W+x]==0: buffer[y][x]=255
                else: buffer[y][x]=0
        return buffer
    def Get_Clear_Area(self,pos):
        '''
            可用作回调函数
            获得可清扫区域(区域图像以及相关坐标)
        '''
        p0_X,p0_Y=calculation.Env2Pix(self.ros_Map.info.origin.position.x,\
                                    self.ros_Map.info.origin.position.y,\
                                    self.ros_Map.info.resolution,\
                                    pos.poses[0].position.x,\
                                    pos.poses[0].position.y)
        p1_X,p1_Y=calculation.Env2Pix(self.ros_Map.info.origin.position.x,\
                                    self.ros_Map.info.origin.position.y,\
                                    self.ros_Map.info.resolution,\
                                    pos.poses[1].position.x,\
                                    pos.poses[1].position.y)
        self.p0=[p0_X,p0_Y]
        self.clean_Matrix=self.Resolve_Map()[p0_Y:p1_Y,p0_X:p1_X]
        rospy.loginfo('[path-server][area-server]received all-cover area:[%f %f][%f %f]'%\
                                            (pos.poses[0].position.x,
                                            pos.poses[0].position.y,
                                            pos.poses[1].position.x,
                                            pos.poses[1].position.y,))
        rospy.loginfo('[path-server][area-server]area converted into:[%d %d][%d %d]'%(p0_X,p0_Y,p1_X,p1_Y))
    def Pre_Process_Img(self):
        '''
            区域图像预处理
        '''
        #按机器人半径对区域进行腐蚀
        self.erode_Pixel=(int)(self.vehicle_max_Diameter/self.ros_Map.info.resolution)
        #机器人宽度转换为像素宽度
        self.width_Pixel=(int)(self.vehicle_cover_Width/2/self.ros_Map.info.resolution)
        #计算步长转换为像素步长
        self.step_Pixel=(int)(self.step_length/self.ros_Map.info.resolution)
        #全覆盖区域图片进行膨胀处理
        kernel=np.ones((self.erode_Pixel,self.erode_Pixel),np.uint8)
        self.clean_Matrix=cv2.erode(self.clean_Matrix,kernel,iterations=1)
            
    def Calc_Blank(self):
        '''
            计算未清扫区域的面积(像素面积，无单位)
        '''
        a=0
        for h in range(self.clean_Matrix.shape[0]):
            for w in range(self.clean_Matrix.shape[1]):
                if self.clean_Matrix[h][w]==255:
                    a+=1
        return a
    def Choose_Start_Point(self,Initial=True):
        '''
            计算清扫开始的起始点
        '''
        if not Initial:
            for h in range(self.Y+self.step_Pixel,self.clean_Matrix.shape[0]-self.erode_Pixel):
                for w in range(self.erode_Pixel,self.clean_Matrix.shape[1]-self.erode_Pixel):
                    if self.clean_Matrix[h][w]!=0:
                        self.X=w;self.Y=h
                        return
        else:
            for h in range(self.erode_Pixel,self.clean_Matrix.shape[0]-self.erode_Pixel):
                for w in range(self.erode_Pixel,self.clean_Matrix.shape[1]-self.erode_Pixel):
                    if self.clean_Matrix[h][w]!=0:
                        self.X=w;self.Y=h
                        return
    def Chk_Border(self):
        '''
            检测是否已经触碰区域边界
        '''
        pass
    def Chk_Covered(self):
        '''
            检测下一目标点是否已经被覆盖
        '''
        pass
    def Chk_Cover_Percentage(self):
        '''
            检测已经清扫的比例
        '''
        return 1-self.Calc_Blank()*1.0/self.blank_Size
    def Gen_Path(self):
        '''
            生成清扫路线
        '''
        while True:
            #首先验证’East‘方向
            if self.clean_Matrix[self.Y][self.X+self.step_Pixel]==255 and self.X<(self.clean_Matrix.shape[1]-self.erode_Pixel):
                self.X+=self.step_Pixel
                self.clean_Matrix[self.Y-self.width_Pixel+1:self.Y+self.width_Pixel,self.X-self.step_Pixel:self.X]=100
                
            #验证'West'方向
            elif self.clean_Matrix[self.Y][self.X-self.step_Pixel]==255 and self.X>self.erode_Pixel:
                self.X-=self.step_Pixel
                self.clean_Matrix[self.Y-self.width_Pixel+1:self.Y+self.width_Pixel,self.X+1:self.step_Pixel+self.X+1]=100
            #验证'North'方向
            elif self.clean_Matrix[self.Y+self.step_Pixel][self.X]==255 and self.Y<(self.clean_Matrix.shape[0]-self.erode_Pixel):
                self.Y+=self.step_Pixel
                self.clean_Matrix[self.Y-self.step_Pixel:self.Y,self.X-self.width_Pixel+1:self.X+self.width_Pixel]=100
            #验证'South'方向
            elif self.clean_Matrix[self.Y-self.width_Pixel][self.X]==255 and self.Y>self.erode_Pixel:
                self.clean_Matrix[self.Y-self.step_Pixel+1:self.Y+1,self.X-self.width_Pixel+1:self.X+self.width_Pixel]=100
                self.Y-=self.step_Pixel
            else: 
                rospy.loginfo('Reinitializing blocks@%f %f'%(self.X,self.Y))
                return
            try:
                cv2.imshow('Path vision',self.clean_Matrix)
                cv2.waitKey(20)
            except:
                pass
            self.path.append([self.X,self.Y])
    def Start_Clean(self,dat):
        '''
            首先，获取清扫区域图片，存入自己的clean_Matrix矩阵中
        '''
        self.Get_Clear_Area(dat)
        '''
            第二步，对图片进行预处理（边缘膨胀）
        '''
        self.Pre_Process_Img()
        '''
            计算应当清扫区域的整体面积
        '''
        self.blank_Size=self.Calc_Blank()
        rospy.loginfo('clean area size:%f m^2'%(self.blank_Size*0.05*0.05))
        if self.blank_Size<10:
            self.path=[]
            return []
        '''
            开始进行整体清扫
        '''
        self.path=[]
        for i in range(10):
            if self.Chk_Cover_Percentage()>self.min_Cover_Percentage or rospy.is_shutdown():
                break
            self.Choose_Start_Point(Initial=False)
            self.Gen_Path()
        self.Choose_Start_Point(Initial=True)
        self.Gen_Path()
        pp=Path()
        go_Out=MoveTo()
        pp.header.frame_id='/map'
        
        rospy.loginfo('covered percentage:%f '%(self.Chk_Cover_Percentage()*100.0))
        for p in range(len(self.path)):
            self.path[p][0],self.path[p][1]=self.path[p][0]+self.p0[0],self.path[p][1]+self.p0[1]
            self.path[p][0],self.path[p][1]=calculation.Pix2Env(self.ros_Map.info.origin.position.x,\
                                    self.ros_Map.info.origin.position.y,\
                                    self.ros_Map.info.resolution,\
                                    self.path[p][0],\
                                    self.path[p][1])
        for p in self.path:
            ptn=PoseStamped()
            spp=Point()
            ptn.header.frame_id='/map'
            ptn.pose.position.x=p[0]
            ptn.pose.position.y=p[1]
            spp.x=p[0]
            spp.y=p[1]
            pp.poses.append(ptn)
            go_Out.locations.append(spp)
        self.path_Pub.publish(pp)
        self.slamware_path_Pub.publish(go_Out)
        
        

if __name__=='__main__':
    rospy.init_node("area_path_server")
    planner=AllCover_Path_Planner()
    rospy.spin()