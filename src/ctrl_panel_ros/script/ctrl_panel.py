#!/usr/bin/env python
#coding=utf-8
import rospy
import tf
from geometry_msgs.msg import Twist
import numpy as np
import pygame
from pygame.locals import *
from pygame.surfarray import make_surface
from sys import exit
import threading
from ros_parser import *
from objects import *
from utils import *

class UI:
    def __init__(self):
        '''
            UI相关
        ''' 
        pygame.init()
        pygame.display.set_caption('Robot Control')
        self.SCREEN_WIDTH=rospy.get_param('window_width',1600)
        self.SCREEN_HEIGHT=rospy.get_param('window_height',900)
        self.window=pygame.display.set_mode([self.SCREEN_WIDTH,self.SCREEN_HEIGHT])     #初始化窗口对象
        self.back=pygame.Surface((self.SCREEN_WIDTH,self.SCREEN_HEIGHT))                #初始化背景（pygame.Surface）
        self.canvas=pygame.Surface((self.SCREEN_WIDTH,self.SCREEN_HEIGHT))       #初始化画布
        '''
            接口
        '''
        self.robot=Robot()
        self.parser=ROS_Parser()  #接口
        self.back.fill((200,200,200))
        self.window.blit(self.back,(0,0))
       
        '''
            数据
        '''
        self.v=0.0
        self.vth=0.0
        self.move_Win=False          #移动窗口缓存
        t=threading.Thread(target=self.Refresh)
        t.start()
        #t.join()                                        #界面刷新线程
    def Refresh(self):
        clock = pygame.time.Clock()
        self.move_Win=False          #移动窗口缓存
        pos_Cache=Point2D()     #移动窗口缓存
        while True:
            clock.tick(30)                                #频率稳定器
            self.window.fill((100,100,100))
            self.canvas.blit(make_surface(self.parser.map),(0,0))
            self.back.blit(self.canvas,(0,0))
            self.window.blit(self.back,(0,100))
            odom_txt="Odometry-X:%f\n  Y：%f\n Angular %f"%(self.parser.robot_Odometry[0],self.parser.robot_Odometry[1],self.parser.robot_Odometry[2])
            odom_surf=self.Render_Text(odom_txt)
            self.window.blit(odom_surf,(0,0))
            pygame.display.update()                         #更新界面
            self.Keyboard()
            self.parser.Manual(v=self.v,vth=self.vth)
    def Keyboard(self):
        for event in pygame.event.get():                #键盘控制
            if event.type==pygame.KEYDOWN:
                if event.key==pygame.K_RIGHT: self.vth=-0.2
                if event.key==pygame.K_LEFT: self.vth=0.2
                if event.key==pygame.K_UP: self.v=0.2
                if event.key==pygame.K_DOWN: self.v=-0.2
                if event.key==pygame.K_PAGEUP:
                    if self.parser.size<2.5: 
                        self.parser.size*=1.2
                        print self.parser.size
                if event.key==pygame.K_PAGEDOWN:
                    if self.parser.size>0.4: 
                        self.parser.size/=1.2
                
            elif event.type==pygame.KEYUP:
                if event.key==pygame.K_RIGHT: self.vth=0
                if event.key==pygame.K_LEFT: self.vth=0      
                if event.key==pygame.K_UP: self.v=0
                if event.key==pygame.K_DOWN: self.v=0 
                self.parser.Manual(v=self.v,vth=self.vth)
            elif event.type==pygame.MOUSEBUTTONDOWN: 
                if event.button==2: 
                    self.move_Win=True
                if event.button==1: 
                    pass
            elif event.type==pygame.MOUSEBUTTONUP: self.move_Win=False
            if event.type==pygame.QUIT:
                    pygame.quit()
                    del self.parser
                    exit()
    def Render_Text(self,txt):
        font=pygame.font.Font("freesansbold.ttf",32)
        text=font.render(txt,True,(255,255,255),None)
        return text
if __name__=='__main__':
    rospy.init_node('control_ui')
    ui=UI()
    rospy.spin()
