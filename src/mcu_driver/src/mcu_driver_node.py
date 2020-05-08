#!/usr/bin/env python
# -*- coding=utf-8 -*-
import serial
import struct
import rospy
import time
import thread
import encoder
from std_msgs.msg import String,Int16,UInt8
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from mcu_driver.srv import *

class CMD_CODES:
    SET_VELOCITY = 0x01
    SET_HEAD_ROTATE_= 0x02
    SET_ARM = 0x03
    SET_UTILS = 0x04
    SET_LED = 0x05
    ASK_UTIL_STATE = 0x11
    ASK_SONAR_VALUE = 0x12
    ASK_IMU = 0x13
    ASK_BATT = 0x14
class POS_CODES:
    PACK_HEAD_POSITION = 0
    PACK_LEN_POSITION = 2
    PACK_CMD_POSITION = 3
    PACK_DATA_POSITION = 4

class MCU_Driver:
    def __init__(self):
        '''
            初始化串口相关数据
        '''
        port=rospy.get_param('~serial_port','/dev/ttyUSB0')
        baudrate=rospy.get_param('~serial_baudrate',460800)
        self.data_io=serial.Serial(port,baudrate,timeout=0.1)

        self.rx_Data=[]
        self.tx_Data=[]
        self.io_Busy=False  #当前串口是否被占用
        '''
            初始化ros相关数据
        '''
        self.robo_Width=rospy.get_param('~robo_width',0.265)
        self.enc_pulse_per_meter=rospy.get_param('~pulse_per_meter',3800)
        rospy.Subscriber('/cmd_vel',Twist,callback=self.SPD)
        rospy.Subscriber('/led',UInt8,callback=self.Set_LED)
        rospy.Service('/get_battery_state',AskBattery,self.Ask_Battery)
        '''
            IO变量
        '''
        self.v_Left=0
        self.v_Right=0
        self.encoder=encoder.Encoder()
        '''
            开启IO循环
        '''
        thread.start_new_thread(self.Thread_IO_Loop,())

    def SPD(self,msg):
        '''
            速度接口
        '''
        self.v_Left=msg.linear.x-msg.angular.z*self.robo_Width/2
        self.v_Right=msg.linear.x+msg.angular.z*self.robo_Width/2
    def Set_Speed(self):
        self.Wait_IO()
        self.Init_Tx_Pack(CMD_CODES.SET_VELOCITY)
        body=[];v_l=0.0;v_r=0.0
        #左轮速度
        if self.v_Left>=0.0: 
            body.append(0x00)
            v_l=self.v_Left
        else: 
            body.append(0x01)
            v_l=-self.v_Left
        v_l=(int)(v_l*1000)
        body.append(v_l/256)
        body.append(v_l%256)
        #右轮速度
        if self.v_Right>=0.0: 
            body.append(0x00)
            v_r=self.v_Right
        else: 
            body.append(0x01)
            v_r=-self.v_Right
        v_r=(int)(v_r*1000)
        body.append(v_r/256)
        body.append(v_r%256)
        self.Pack_Tx_Pack(body)
        self.data_io.write(self.tx_Data)
        ret = self.data_io.read(12)
        
        '''
            解析编码器反馈数据
        '''
        if len(ret)<12:return
        if(ord(ret[0])!=0xff or ord(ret[1])!=0xff or ord(ret[2])!=0x0c or ord(ret[3])!=0x01):
            #验证数据的准确性 
            self.Release_IO()
            return
        en_l=struct.unpack('<i',ret[POS_CODES.PACK_DATA_POSITION:POS_CODES.PACK_DATA_POSITION+4])[0]
        en_r=-struct.unpack('<i',ret[POS_CODES.PACK_DATA_POSITION+4:POS_CODES.PACK_DATA_POSITION+8])[0]
        self.encoder.Refresh_Encoder(en_l,en_r,self.enc_pulse_per_meter,self.robo_Width)
        self.Release_IO()
    def Set_LED(self,val):
        self.Wait_IO()
        self.Init_Tx_Pack(CMD_CODES.SET_LED)
        body=[val.data]
        rospy.loginfo('LED set to %d'%body[0])
        self.Pack_Tx_Pack(body)
        self.data_io.write(self.tx_Data)
        ret=list(self.data_io.read(5))
        self.Release_IO()
    def Ask_Battery(self,req):
        self.Wait_IO()
        self.Init_Tx_Pack(CMD_CODES.ASK_BATT)
        body=[0]
        self.Pack_Tx_Pack(body)
        self.data_io.write(self.tx_Data)
        ret=list(self.data_io.read(7))
        print(ret)
        self.Release_IO()
        return AskBatteryResponse(0,ord(ret[4])/10.0)
        
    def Init_Tx_Pack(self,cmdID):
        '''
            构建标准数据头
        '''
        self.tx_Data=[]
        self.tx_Data.append(0xff);self.tx_Data.append(0xff)
        self.tx_Data.append(0);self.tx_Data.append(cmdID)
    def Pack_Tx_Pack(self,body):
        self.tx_Data.extend(body)
        self.tx_Data[POS_CODES.PACK_LEN_POSITION]=len(self.tx_Data)
    
    def Wait_IO(self):
        while self.io_Busy:
            time.sleep(0.001)
        self.io_Busy=True
    def Release_IO(self):
        self.io_Busy=False

    def Thread_IO_Loop(self):
        r=rospy.Rate(20)
        while not rospy.is_shutdown():
            r.sleep()
            self.Set_Speed()
            
        
if __name__=='__main__':
    rospy.init_node('mcu_driver')
    mcu=MCU_Driver()
    rospy.spin()
    self.Release_IO()
    
