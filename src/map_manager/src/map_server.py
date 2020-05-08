#!/usr/bin/env python
# -*- coding=utf-8 -*-
import sys
import rospy
import genpy
import cv2
import numpy as np
import os
import json
from map_manager.srv import *
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String,UInt8
class Map_Server:
    def __init__(self):
        self.path=rospy.get_param('map_database','/home/zhangdeyu/slamware/src/cfg/map_database')
        rospy.Service('ask_map_list',AskMapList,self.Map_List)
        rospy.Service('load_map_service',LoadMap,self.Load_Map)
        self.map_Pub=rospy.Publisher('/map',OccupancyGrid,queue_size=1,latch=True)
        self.map=OccupancyGrid()
        self.origin=[0.0,0.0]
        self.resolution=0.5
        self.occu_th=200
        self.free_th=50
        self.homes=[]
        self.areas=[]
        self.routines=[]
    def Map_List(self,req):
        if not os.path.isdir(self.path):
            return AskMapListResponse('{"list":["Congifure Error"]}')
        ret={};ret['list']=os.listdir(self.path)
        return AskMapListResponse(json.dumps(ret))
    def Load_Map(self,req):
        fileDir=os.path.join(self.path,req.name)
        if not os.path.isdir(fileDir): return LoadMapResponse(-1,'map do not exist')
        mapName=os.path.join(fileDir,req.name+'.jpg')
        cfgName=os.path.join(fileDir,req.name+'.cfg')
        paths=os.path.join(fileDir,req.name+'.path')
        user_Config=os.path.join(fileDir,req.name+'.ucfg')
        try:
            f=open(cfgName,'r')
            j=json.loads(f.read())
            self.origin=[float(j['origin_x']),float(j['origin_y'])]
            self.resolution=float(j['resolution'])
            f.close()
            map_Matrix=cv2.imread(mapName,cv2.IMREAD_GRAYSCALE)
            self.map.header.frame_id='/map'
            self.map.header.seq+=1
            self.map.header.stamp=genpy.Time()
            #   设定地图参数数据
            self.map.info.height=len(map_Matrix)
            self.map.info.width=len(map_Matrix[0])
            self.map.info.resolution=self.resolution
            self.map.info.origin.position.x=self.origin[0]
            self.map.info.origin.position.y=self.origin[1]
            self.map.info.map_load_time=genpy.Time()
            self.map.data=np.zeros((self.map.info.height*self.map.info.width),dtype=UInt8)
            for row in range(self.map.info.height):
                for col in range(self.map.info.width):
                    index=row*self.map.info.width+col
                    if map_Matrix[row][col]<(255-self.occu_th):
                        self.map.data[index]=100
                    elif map_Matrix[row][col]>(255-self.free_th):
                        self.map.data[index]=0
                    else:
                        self.map.data[index]=-1
            self.map_Pub.publish(self.map)
            return LoadMapResponse(0,'map %s loaded'%req.name)
        except Exception,err:
            return LoadMapResponse(-1,str(err))
if __name__=='__main__':
    rospy.init_node('map_server')
    m=Map_Server()
    rospy.spin()