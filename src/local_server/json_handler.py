#!/usr/bin/env python
# -*- coding:utf-8 -*-
import json
import rospy
def Resolve_Spd(dat):
    spd=json.loads(dat)
    return float(spd['v']),float(spd['vth'])
def Resolve_Map_Name(dat):
    return json.loads(dat)['name']
def Resolve_id_str(dat):
    return json.loads(dat)['id']
def Resolve_Area(dat):
    d_Dict=json.loads(dat)
    return float(d_Dict['x1']),float(d_Dict['y1']),float(d_Dict['x2']),float(d_Dict['y2'])
def Resolve_2D_Position(dat):
    return float(dat['x']),float(dat['y']),float(dat['yaw'])
def Resolve_Login(dat):
    dat=json.loads(dat)
    return dat['name'],dat['key']
def Create_Map_Config(o_x,o_y,h,w,homes,areas,lines):
    ret={}
    ret['ret']='ok'
    ret['origin']=[o_x,o_y]
    ret['height']=h
    ret['width']=w
    ret['areas']=areas
    ret['lines']=lines
    ret['homes']=homes
    return json.dumps(ret)
def Create_Hardware_State(battery=0,\
        state='free',\
        utils=[0,0,0,0],\
        xx=0.0,\
        yy=0.0,\
        zz=0.0,\
        r=0.0,\
        p=0.0,\
        y=0.0,\
        area='default',\
        speed=0.5,\
        safety=0.0,\
        blocked='false',\
        percentage=0.0):
    nav={}
    hdw={}
    nav['x']=xx
    nav['y']=yy
    nav['z']=zz
    nav['roll']=r
    nav['pitch']=p
    nav['yaw']=y
    nav['area_name']=area
    nav['speed']=speed
    nav['safety']=safety
    nav['blocked']=blocked
    nav['percentage']=percentage
    hdw['battery']=battery
    hdw['state']=state
    hdw['utils']=utils
    ret={'hardware':hdw,'navigation_state':nav,'ret':'ok'}
    return json.dumps(ret)
