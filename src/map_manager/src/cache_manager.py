#!/usr/bin/env python
#coding=utf-8

'''
    机器人位置存储以及刷新节点
'''
import os
import rospy
import tf
import math
import time
import thread
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped,Pose
filename=""
scene=""
linear_accurancy=0.1
angular_accurancy=0.1
class Env_Cache:
    '''
        场景缓存管理
    '''
    def __init__(self):
        rospy.loginfo('Cache Initialing...!')
        self.scene=''
        self.pose=Pose()
        self.cache=[0.0,0.0,0.0]
        self.listener=tf.TransformListener()
        self.linear_accurancy=rospy.get_param('cache_linear_accurancy',0.1)
        self.angular_accurancy=rospy.get_param('cache_angular_accurancy',0.2)
        self.cache_path=rospy.get_param('cache_database','')
    '''
        更新位置信息
    '''
    def Cache_Dump(self):
        while not rospy.is_shutdown():
            time.sleep(0.05)
            try:
                if not os.path.isdir(self.cache_path): 
                    return
                (trans,rot)=self.listener.lookupTransform('/map', '/base_link', rospy.Time(0.0))
                (roll,pitch,yaw)=tf.transformations.euler_from_quaternion(rot)
                if (math.fabs(self.cache[0]-trans[0])<self.linear_accurancy) and \
                    (math.fabs(self.cache[1]-trans[1])<self.linear_accurancy) and \
                    (math.fabs(self.cache[2]-yaw)<self.angular_accurancy): return
                f2write = open(os.path.join(self.cache_path,'cache.cache'),'w+')
                f2write.write('x: '+str(trans[0])+'\n')
                f2write.write('y: ' + str(trans[1]) + '\n')
                f2write.write('a: ' + str(yaw) + '\n')
                f2write.write('area: ' + self.scene)
                f2write.close()
                self.cache=[trans[0],trans[1],yaw]
                return
            except Exception,err:
                pass
    '''
        初始化位置信息
    '''
    def Cache_Init(self):
        try:
            if not os.path.isdir(self.cache_path):
                raise Exception('lost cache directory!')
                return
            f2read=open(os.path.join(self.cache_path,'cache.cache'),'r')
            while True:
                line=f2read.readline()
                if not line:
                    break
            line=line.split(': ')
            if line[0] == 'area': self.scene = line[1]
            if line[0] == 'x' : self.cache[0]=float(line[1])
            if line[0] == 'y' : self.cache[1]=float(line[1])
            if line[0] == 'a' : self.cache[2]=float(line[1])
            f2read.close()
            rospy.loginfo('Robot initialized in %s by cache'%self.scene)
            thread.start_new_thread(self.Cache_Dump,())
        except Exception,err:
            rospy.logerr("[map_server][cache initializer] %s"%err)
    def Cache_Dump_Loop(self):
        thread.start_new_thread(self.Cache_Dump,())
if __name__ == '__main__':
    rospy.init_node('pos_chache')
    i=Env_Cache()
    i.Cache_Init()
    i.Cache_Dump()