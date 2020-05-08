#!/usr/bin/env python
# -*- coding=utf-8 -*-
import rospy
import time
import thread as thread
from safety_ctrl.srv import *
from geometry_msgs.msg import Twist
class Speed_ctrl:
    def __init__(self):
        self.v=Twist()
        self.priority=0
        self.scans=[]
        self.sonars=[]
        self.mixer=rospy.Service('/set_speed',SetSpeed,self.Set_Speed)
        self.v_Pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        thread.start_new_thread(self.Loop_Spd,())
    def Set_Speed(self,req):
        ret=SetSpeedResponse()
        if req.initial: self.priority=req.priority
        ret.priority= req.priority - self.priority
        if ret.priority>=0:
            self.v=req.speed
            self.priority=req.priority
            ret.retStr='{"ret":"ok"}'
        else:
            ret.retStr='{"ret":"failed"}'
        return ret
    def Loop_Spd(self):
        r=rospy.Rate(30)
        while not rospy.is_shutdown():
            self.v_Pub.publish(self.v)
            r.sleep()
    def Start_Loop(self):
        pass
if __name__=='__main__':
    rospy.init_node('speed_mixer')
    s_ctrl=Speed_ctrl()
    rospy.spin()