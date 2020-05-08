#!/usr/bin/env python
#coding=utf-8
import pygame
from pygame.locals import *
from pygame.transform import rotozoom as Trans
import math
import numpy as np
import rospy
import tf
from get_tf import *
from basic_math_2D import ArrowPoint2D
class Robot:
    def __init__(self):
        self.pos=ArrowPoint2D()
        self.res=pygame.image.load('resource/robot.png')
        self.pic=Trans(self.res,self.pos.Theta_Format()[2],0.1)
        self.listener=tf.TransformListener()
    def RePosition(self):
        self.pos.Reload(Get_TF(listener=self.listener))
        #self.pic.set_colorkey(255)
        self.pic=Trans(self.res,-self.pos.Theta_Format()[2]*180/3.14,0.1)
class Visual_Path:
    def __init__(self):
        self.Points=[]
class Tips:
    def __init__(self,text=None,size=60):
        self.textImg=[]
    def Add_A_Tip(self):
        pass