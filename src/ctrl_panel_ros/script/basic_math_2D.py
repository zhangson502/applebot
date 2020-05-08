#!/usr/bin/env python
# -*- coding: UTF-8 -*-  
import math
'''
    二维空间点集
'''
class Point2D:
    def __init__(self,x=0.0,y=0.0):
        self.x=x
        self.y=y
    def __add__(self,value):
        return Point2D(self.x+value.x,self.y+value.y)
'''
    二维空间无原点向量
'''
class Vector2D:
    def __init__(self,x=0.0,y=0.0):
        self.x=x
        self.y=y
    def __add__(self,value):
        return Vector2D(self.x+value.x,self.y+value.y)
    def __mul__(self,value):
        return Vector2D(self.x*value.x,self.y*value.y)
'''
    二维空间带有向量的点
'''
class ArrowPoint2D(Point2D):
    def __init__(self,x=0.0,y=0.0,vec_x=0.0,vec_y=0.0):
        self.x=x
        self.y=y
        self.vec_x=vec_x
        self.vec_y=vec_y
    def __init__(self, x1=0.0, y1=0.0, x2=0.0, y2=0.0):
        self.x=x1
        self.y=y1
        self.vec_x = x2-x1
        self.vec_y = y2-y1
    def Reload(self,(x,y,th)):
        self.x=x
        self.y=y
        self.vec_x=math.cos(th)
        self.vec_y=math.sin(th)
    def Theta_Format(self):
        return self.x,self.y,math.atan2(self.vec_y,self.vec_x)
    #def Affine()