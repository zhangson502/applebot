#!/usr/bin/env python
# -*- coding=utf-8 -*-
import math
import cv2
def Env2Pix(cor_X_Coord,cor_Y_Coord,resolution,env_X,env_Y):
    '''
        机器人实际环境坐标--->机器人在全局地图上的像素坐标转换
        cor_X/Y_Coord:机器人地图位图左下角对应点坐标
        resolution:机器人分辨率（每个像素方块边长代表的长度）
        env_X,env_Y,环境坐标
    '''
    x=env_X-cor_X_Coord
    y=env_Y-cor_Y_Coord
    return (int)(x/resolution),(int)(y/resolution)

def Pix2Env(cor_X_Coord,cor_Y_Coord,resolution,pix_X,pix_Y):
    '''
        机器人像素坐标--->机器人实际坐标
    '''
    x=pix_X*resolution
    y=pix_Y*resolution
    return x+cor_X_Coord,y+cor_Y_Coord
def Calc_Paths(img):
    pass