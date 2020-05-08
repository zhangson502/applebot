#!/usr/bin/env python
# -*- coding=utf-8 -*-
import rospy
import json
import tf
import time
import math
from std_msgs.msg import String,UInt64
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose,PoseStamped,Point
from slamware_ros_sdk.msg import MoveToLocationsRequest as MoveTo
import thread as thread
class Designated_Path_Server:
    def __init__(self):
        self.listener=tf.TransformListener()
        self.r_Switch=False
        self.cache=[0.0,0.0,0.0]
        self.scene=''
        self.routine=[]
        self.path=[]
        self.space=rospy.get_param('static_path_generate_interval',0.1)
        self.paths_Store_File=rospy.get_param('path_cfg_path','')
        self.path_Store_Sub=rospy.Subscriber('path_save',String,callback=self.P_Cb)
        self.path_Pub_Sub=rospy.Subscriber('path_generate',String,callback=self.Resolve_Routine)
        self.path_Pub=rospy.Publisher('designated_path',Path,queue_size=5,latch=True)
        self.slamware_path_Pub=rospy.Publisher('move_to_locations',MoveTo,queue_size=5,latch=True)
    def Sen_Cb(self,dat):
        self.scene=dat.data
    def P_Cb(self,dat):
        p_id=dat.data
        if p_id=='rec':
            thread.start_new_thread(self.Thread_Draw,())
        else:
            self.Dump_Routine(p_id)
    def Get_TF(self):
        for i in range(10):
            try:
                time.sleep(0.05)
                (trans,rot)=self.listener.lookupTransform('/map', '/base_link', rospy.Time(0.0))
                (roll,pitch,yaw)=tf.transformations.euler_from_quaternion(rot)
                return trans[0],trans[1],yaw
            except Exception,err:
                pass
        return 0,0,0
    def Thread_Draw(self):
        if self.r_Switch: return
        self.routine=[]
        self.r_Switch=True
        rospy.loginfo('started generating path...')
        while self.r_Switch==True:
            time.sleep(0.1)
            x,y,yaw=self.Get_TF()
            if (math.fabs(self.cache[0]-x)<self.space) and \
                    (math.fabs(self.cache[1]-y)<self.space): continue
            self.cache=[x,y,yaw]
            pnt={}
            pnt['x']=x
            pnt['y']=y
            pnt['yaw']=yaw
            self.routine.append(pnt)
    def Dump_Routine(self,p_id='Default'):
        self.r_Switch=False
        time.sleep(0.5)
        f=open(self.paths_Store_File+'/'+self.scene+'_'+p_id+'.pathcfg','w+')
        p={}
        p['id']=p_id
        p['path']=self.routine
        f.write(json.dumps(p,indent=4))
        f.close()
        rospy.loginfo('path generated:length:%d'%len(self.routine))
        self.routine=[]
    def Resolve_Routine(self,r_id):
        p_id=r_id.data
        f=open(self.paths_Store_File+'/'+self.scene+'_'+p_id+'.pathcfg','r')
        d=f.read()
        path=json.loads(d)
        p_List=path['path']
        p_Out=Path()
        p_Out.header.frame_id='map'
        go_Out=MoveTo()
        for point in p_List:
            pp=Point()
            pose=Pose()
            pp.x=point['x']
            pp.y=point['y']
            pose.position.x=point['x']
            pose.position.y=point['y']
            pose_Stmp=PoseStamped()
            pose_Stmp.pose=pose
            p_Out.poses.append(pose_Stmp)
            go_Out.locations.append(pp)
            #go_Out.options.opt_flags=go_Out.options.opt_flags.MILESTONE
        self.path_Pub.publish(p_Out)
        self.slamware_path_Pub.publish(go_Out)
        rospy.loginfo('designated path published!')
if __name__=='__main__':
    rospy.init_node('path_generator')
    g=Designated_Path_Server()
    rospy.spin()
