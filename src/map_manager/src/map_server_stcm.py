#!/usr/bin/env python
# -*- coding=utf-8 -*-
import os
import rospy
import json
import tf
import cache_manager
from std_msgs.msg import String
from geometry_msgs.msg import Pose

from slamware_ros_sdk.srv import *
from slamware_ros_sdk.msg import *
from map_manager.srv import *
'''

    Map Load Interface

'''
class Map_Loader:
    def __init__(self):
        '''
            初始化地图加载程序
            加载Service
        '''
        rospy.loginfo('Stcm Loader Initiating...')
        rospy.wait_for_service('/sync_set_stcm')
        self.stcm_load_client=rospy.ServiceProxy('/sync_set_stcm',SyncSetStcm)
        #地图存储位置配置文件
        self.map_load_path=rospy.get_param('map_database','')
        rospy.loginfo('Stcm Loader Initiated!')
    def Load_Map(self,name):
        try:
            if name=='': return -1,'no map data'
            if not os.path.exists(os.path.join(self.map_load_path,name)):
                return -1,'no map in that name'
            o=self.Load_Map_Cfg(name)
            f=open(os.path.join(self.map_load_path,name,('stcm_binaries')),'rb')
            dat=f.read()
            pose=Pose()
            pose.position.x=o[0]
            pose.position.y=o[1]
            [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]=tf.transformations.quaternion_from_euler(0,0,o[2])
            ret=self.stcm_load_client(dat,pose)
            return 0,'map successfully loaded'
        except rospy.ServiceException, e:
           rospy.logerr("[map-server][load-error]%s"%e) 
           return -1,'%s'%e
    def Load_Map_Cfg(self,name):
        origin=[0,0,0]
        try:
            f=open(os.path.join(self.map_load_path,name,('origincfg')),'r')
            dat=json.loads(f.read())
            origin=[dat['x'],dat['y'],dat['yaw']]
        except rospy.ServiceException, e:
            rospy.logerr("[stcm-mapserver][loader]"%e) 
        return origin
'''

    Map Save Interface

'''

class Map_Saver:
    '''
        地图保存类
        接口：
            @rosparam/stcm_map_path：地图存储配置文件路径
            @Save_Map: name/输入地图存储位置
    '''
    def __init__(self):
        '''
            初始化地图保存程序
            加载Service
        '''
        rospy.loginfo('Stcm Saver Initiating...')
        rospy.wait_for_service('/sync_get_stcm')
        self.stcm_save_client=rospy.ServiceProxy('/sync_get_stcm',SyncGetStcm)
        #地图存储位置配置文件
        self.map_save_path=rospy.get_param('map_database','')
        self.listener=tf.TransformListener()
        rospy.loginfo('Stcm Saver Initiated!')
    def Save_Map(self,name,cfg):
        '''
            保存地图
        '''
        rospy.loginfo('Trying to Save STCM Map...')
        try:
            if os.path.exists(os.path.join(self.map_save_path,name)):
                return -1,'map already exists'
            os.mkdir(os.path.join(self.map_save_path,name))
            f=open(os.path.join(self.map_save_path,name,('origincfg')),'w+')
            o_x,o_y,o_yaw=self.Get_TF()
            j={}
            j['x']=o_x
            j['y']=o_y
            j['yaw']=o_yaw
            f.write(json.dumps(j,indent=4))
            f.close()
            ret=self.stcm_save_client()
            ret_dat=ret.raw_stcm
            f=open(os.path.join(self.map_save_path,name,('stcm_binaries')),'wb+')
            f.write(ret_dat)
            f.close()
            f=open(os.path.join(self.map_save_path,name,('configures')),'w+')
            f.write(cfg)
            f.close()
            return 0,'map successfully saved!'
        except Exception, e:
            rospy.logerr("[stcm-mapserver][saver]: %s"%e)
            return -1,'%s'%e

    def Get_TF(self):
        for i in range(10):
            try:
                time.sleep(0.05)
                (trans,rot)=self.listener.lookupTransform('/map', '/base_link', rospy.Time(0.0))
                (roll,pitch,yaw)=tf.transformations.euler_from_quaternion(rot)
                print trans[0],trans[1],yaw
                return trans[0],trans[1],yaw
            except Exception,err:
                pass
        return 0,0,0
class Map_Drawer:
    def __init__(self):
        self.start_Pub=rospy.Publisher('/set_map_update',SetMapUpdateRequest,queue_size=1)
        self.sync_Pub=rospy.Publisher('sync_map',SyncMapRequest,queue_size=1)
        self.clear_Pub=rospy.Publisher('clear_map',ClearMapRequest,queue_size=1)
    def Start_Draw(self):
        self.clear_Pub.publish(ClearMapRequest())
        self.start_Pub.publish(SetMapUpdateRequest(enabled=True))
        self.sync_Pub.publish(SyncMapRequest())
    def Stop_Draw(self):
        pass
class Map_Server:
    def __init__(self):
        self.path=rospy.get_param('map_database','')
        rospy.Service('ask_map_list',AskMapList,self.Map_List)
        rospy.Service('load_map_service',LoadMap,self.Load_Map)
        rospy.Service('start_draw',StartDraw,self.Start_Draw_Map)
        rospy.Service('stop_dump_map',SaveMap,self.Save_Map)
        rospy.Service('get_scene_info',GetSceneInfo,self.Get_Scene_Info)
        self.saver=Map_Saver()
        self.loader=Map_Loader()
        self.drawer=Map_Drawer()
        self.cache=cache_manager.Env_Cache()
        self.Init_Env()
    def Init_Env(self):
        self.cache.Cache_Init()
        self.loader.Load_Map(self.cache.scene)
    def Map_List(self,req):
        if not os.path.isdir(self.path):
            return AskMapListResponse('{"list":["Congifure Error"]}')
        ret={};ret['list']=os.listdir(self.path)
        return AskMapListResponse(json.dumps(ret))
    def Load_Map(self,req):
        retCd,retStr=self.loader.Load_Map(req.name)
        if retCd==0:
            self.cache.scene=req.name
        return LoadMapResponse(retCd,retStr)
    def Start_Draw_Map(self,req):
        self.cache.scene=''
        self.drawer.Start_Draw()
        return StartDrawResponse(0,'started!')
    def Save_Map(self,req):
        ret,retStr=self.saver.Save_Map(req.name,req.config)
        if ret!=0:
            return SaveMapResponse(-1,retStr)
        else:
            self.cache.scene=req.name
            return SaveMapResponse(0,retStr)
    def Get_Scene_Info(self,req):
        return GetSceneInfoResponse(self.cache.scene,'','')

if __name__=='__main__':
    rospy.init_node('map_server')
    server=Map_Server()
    rospy.spin()


    