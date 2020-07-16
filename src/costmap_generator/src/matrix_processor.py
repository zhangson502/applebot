#!/usr/bin/env python
# -*- coding=utf-8 -*-
import rospy
import numpy as np
def CoordinationTransfer2D(tf_Trans=[0,0,0],realPos=[0,0]):
    '''
        原始数据点转换至栅格(平面)
        方法：首先根据两个坐标之间的位置关系构造出变换矩阵，然后根据矩阵乘法计算生成新的坐标系
    '''
    #   1. 构建变幻矩阵
    #构造原始矩阵（基于坐标系2的坐标矩阵）
    prime_Mat=mat([realPos[0],realPos[1],1.0])
    #构造坐标系2向坐标系1的变换矩阵
    trans_Mat=np.mat([[np.cos(tf_Trans[2]),np.sin(tf_Trans[2])],[-np.sin(tf_Trans[2]),np.cos(tf_Trans[2])],[tf_Trans[0],tf_Trans[1]]])
    #矩阵相乘，实现变换
    coord2Mat=prime_Mat*trans_Mat
    return [coord2Mat[0,0],coord2Mat[0,1]]
def Polar2XY(angle=0.0,dist=0.0):
    '''
        极坐标向直角坐标转换
    '''
    return [dist*np.cos(angle),dist*np.sin(angle)]
def RealXY2MatrixXY2D(realPos=[0,0],resolution=0.05,pix_width=80,pix_height=80):
    '''
        原始数据坐的直角坐标系转化到矩阵的栅格点
    '''
    return int(realPos[0]/resolution)+int(pix_height/2),int(realPos[1]/resolution)+int(pix_width/2)