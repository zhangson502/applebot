#!/usr/bin/env python
#!coding=utf-8

import socket
import cv2
import numpy
import rospy
import time
import thread as thread

class Cam_TCP:
    def __init__(self,port=8890,v_ID=1,quality=80):
        
        print ('Waiting for Connection.')
        # 开启摄像头服务
        rospy.loginfo('camera: starting camera')
        try:
            self.cap = cv2.VideoCapture(v_ID)
        except:
            rospy.logerr('camera error: camera initializing failed')
            exit(0)
        self.encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),quality] #设置编码参数
        #开启网络传输服务
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            address = ('', port)
            self.s.bind(address) # 将Socket（套接字）绑定到地址
            self.s.listen(True) # 开始监听TCP传入连接
        except Exception,err:
            rospy.logerr(err)
            exit(0)
        self.frame=''
        self.alive=True
        thread.start_new_thread(self.T_Start_Cam,())
    def T_Start_Cam(self):
        while self.alive:
            ret,frame=self.cap.read()
            result, imgencode = cv2.imencode('.jpg', frame, self.encode_param)
            self.frame = numpy.array(imgencode).tostring()
    def Start_Server(self):
        while self.alive:
            try:
                conn, addr = self.s.accept()
                thread.start_new_thread(self.T_Start_IO,(conn,addr))
            except:
                rospy.logerr('camera error: TCP client accept failed')
                self.alive=False
    def T_Start_IO(self,conn,addr):
        rospy.loginfo('accepted client from '+addr[0])
        while self.alive:
            try:
                #先对图像进行赋值，避免线程之间的变量IO冲突
                frame=self.frame
                #发送
                conn.sendall(str(len(frame)).ljust(16))
                conn.sendall(frame)
            except:
                rospy.loginfo('pipe release from '+addr[0])
                break
        conn.close()

if __name__=='__main__':
    rospy.init_node('cam_tcp_driver')
    cam=Cam_TCP()
    cam.Start_Server()