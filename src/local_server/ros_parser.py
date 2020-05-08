#coding=utf-8
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String,Empty,UInt64
from geometry_msgs.msg import Pose,PoseArray
from map_manager.srv import *
import numpy as np
import rospy
import cv2
import tf
import time

from safety_ctrl.srv import *

class ROS_Parser:
    def __init__(self):
        '''
            建图与定位相关收发器
        '''
        rospy.init_node('http_server')
        #请求地图相关服务：
        self.set_map_Client=rospy.ServiceProxy('load_map_service',LoadMap)
        self.get_map_List_Client=rospy.ServiceProxy('ask_map_list',AskMapList)
        self.start_Draw_Map_Client=rospy.ServiceProxy('start_draw',StartDraw)
        self.dump_Map_Client=rospy.ServiceProxy('stop_dump_map',SaveMap)
        self.get_Scene_Info=rospy.ServiceProxy('get_scene_info',GetSceneInfo)
        #重定位发布
        self.relocate_Pub=rospy.Publisher('/set_pose',Pose,queue_size=1)
        #机器人地图获取以及地图配置信息
        self.map_Sub=rospy.Subscriber('/map',OccupancyGrid,callback=self.Map_Cb)
        self.ros_Map=OccupancyGrid()
        self.homes=[]
        self.areas=[]
        self.lines=[]
        self.have_Map=False
        #机器人位置获取
        self.scene=''
        self.tf_Listener=tf.TransformListener()
        '''
            运动及导航发布
        '''
        #速度控制器发布
        self.vel_Pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        #路径点发布

        self.path_Save_Pub=rospy.Publisher('/path_save',String,queue_size=1)
        self.path_Gen_Pub=rospy.Publisher('/path_generate',String,queue_size=1)
        #清扫区域发布
        self.clean_area_Pub=rospy.Publisher('/clean_area',PoseArray,queue_size=1)
        #取消任务发布
        self.cancel_Pub=None
        '''
            机器人状态
        '''
        #机器人基本状态信息

    '''
        用户操作授权函数
    '''
    def Authorization(self,name,key):
        rospy.wait_for_service('authorization')
        auth=rospy.ServiceProxy('authorization',Authorization)
        try:
            ret=auth(user=name,key=key)
            return ret.auth_ret,''
        except Exception,err:
            return 0,err
    '''
        建图与定位相关函数
    '''
    def Map_Cb(self,map_Data):
        self.ros_Map=map_Data
        self.have_Map=True
    def Map_Save(self,strr='',cfg=''):
        ret=self.dump_Map_Client(OccupancyGrid(),cfg,strr)
        rospy.logerr(ret)
        return ret.ret,ret.retStr
    def Call4Scene_Info(self):
        ret=self.get_Scene_Info()
        return ret.name,ret.config,ret.robot_origin
    def Call4Maplist(self):
        try:
            return 0,self.get_map_List_Client().list
        except:
            return -1,'error'
    def Call4JPEGMAP(self,quanlity=95):
        H=self.ros_Map.info.height
        W=self.ros_Map.info.width
        #载入地图  
        buffer=np.zeros((H,W),dtype=np.uint8)
        for y in range(H):
            for x in range(W):
                if self.ros_Map.data[y*W+x]==-1: buffer[y][x]=100
                elif self.ros_Map.data[y*W+x]==0: buffer[y][x]=255
                else: buffer[y][x]=0
        if len(buffer)==0: return 'MAP INVALID'
        img_encode = cv2.imencode('.jpg',buffer)[1]
        data = np.array(img_encode)
        stringData = data.tostring()
        return stringData
    def Call4MAPConfig(self):
        return self.ros_Map.info.origin.position.x,\
            self.ros_Map.info.origin.position.y,\
            self.ros_Map.info.height,\
            self.ros_Map.info.width,\
            self.homes,\
            self.areas,\
            self.lines
    def Get_TF(self,parent='/map',child='/base_link'):
        try:
            time.sleep(0.02)
            (trans,rot)=self.tf_Listener.lookupTransform(parent,child,rospy.Time(0))
            roll,pitch,yaw=tf.transformations.euler_from_quaternion(rot)
            return trans[0],trans[1],yaw
        except Exception,err:
            rospy.logerr("[http-server][ros-tf]"%err)
            return 0,0,0
    
    '''
        机器人控制及指令函数
    '''
    def Set_Robot_Pose(self,x=0,y=0,z=0,row=0,pitch=0,yaw=0):
        r_pose=Pose()
        r_pose.position.x=x
        r_pose.position.y=y
        r_pose.orientation=tf.transformations.quaternion_from_euler(row,pitch,yaw)
        self.relocate_Pub.publish(r_pose)
    def Manual(self,v,vth):
        try:
            tmp=Twist()
            tmp.angular.z=vth;tmp.linear.x=v;
            self.vel_Pub.publish(tmp)
            return 'ok'
        except Exception,err:
            rospy.logerr('[http-server][ros-parser]%s'%err)
            return '{"ret":"%s"}'%err
    def AreaClean(self,(x1,y1,x2,y2)):
        rospy.loginfo("Starting Area Clean-P1:[%f,%f] P2:[%f,%f]"%(x1,y1,x2,y2))
        arr=PoseArray()
        arr.header.frame_id='map'
        P1=Pose();P2=Pose()
        P1.position.x=x1;P1.position.y=y1
        arr.poses.append(P1)
        P2.position.x=x2;P2.position.y=y2
        arr.poses.append(P2)
        self.clean_area_Pub.publish(arr)
    def Save_Path(self,dat):
        s=String();s.data=dat
        self.path_Save_Pub.publish(s)
    def Gen_Path(self,dat):
        s=String();s.data=dat
        self.path_Gen_Pub.publish(s)
        
    def Cancel_Action(self):
        pass
    '''
        机器人状态函数
    '''
    def Sen_Cb(self,dat):
        pass
    def Set_Scene(self,scene):
        try:
             ret=self.set_map_Client(scene)
             return ret.ret,ret.retStr
        except:
           return -1,'map server invalid'

if __name__=='__main__':
    parser=ROS_Parser()
    print parser.Call4JPEGMAP()

