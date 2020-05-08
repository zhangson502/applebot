#!/usr/bin/env python
# -*- coding=utf-8 -*-
import rospy
from safety_ctrl.srv import Authorization,AuthorizationResponse
class Auth_Aquire:
    def __init__(self):
        rospy.init_node('authorizing_server')
        self.key_Store_File=rospy.get_param('authorize_key_store_file','')
        self.USERS={'guest':'12345678'}
        self.auth_Server=rospy.Service('authorization',Authorization,self.AUTH)
    def Load_Keys(self):
        f=open(self.key_Store_File,'r')
        while True:
            ret=f.readline()
            if not ret: break
            user,key=ret.split(": ")
            self.USERS[user]=key.strip('\n')
    def AUTH(self,req):
        ids=req.user
        key=req.key
        ret=AuthorizationResponse()
        if not ids in self.USERS.keys():
            ret.auth_ret=ret.REJECT;return ret
        if key!=self.USERS[ids]:
            ret.auth_ret=ret.REJECT;return ret
        if ids=='admin':ret.auth_ret=ret.ADMIN
        elif ids=='guest':ret.auth_ret=ret.USER
        else:ret.auth_ret=ret.USER
        return  ret
if __name__=='__main__':
    au=Auth_Aquire()
    au.Load_Keys()
    rospy.spin()


