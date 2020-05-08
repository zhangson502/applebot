#!/usr/bin/env python
# -*- coding=utf-8 -*-
import socket
import threading
import rospy
import time
import http_server_parser as http
import thread

class Robot_Http_Server:
    def __init__(self):
        '''
            初始化服务器
        '''
        self.port=rospy.get_param('http_server_port',8886)
        #初始化Socket
        self.s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.s.bind((rospy.get_param('http_server_ip','192.168.0.201'), self.port))
        self.s.listen(5)
        #初始化HTTP部分
        self.http_parser=http.HttpRequest()
        self.Activate()
    def Activate(self):
        rospy.loginfo('local server started!')
        while not rospy.is_shutdown():
            try:
                while self.http_parser.busy:time.sleep(0.01)
                client_socket,client_address = self.s.accept()
                self.http_parser.Set_Busy()
                thread.start_new_thread(self.Thread_Server,(client_socket,client_address))
            except Exception,err:
                rospy.logerr(err)
    def Thread_Server(self,client_socket,client_address):
            client_socket.settimeout(0.2)
            request_data=''
            while True:
                try:
                    tmp=client_socket.recv(1024)
                    request_data+=tmp
                except:
                    break
            self.http_parser.parse_request(request_data)
            response_data=self.http_parser.get_response()
            try:
                client_socket.sendall(response_data)
                client_socket.close()
                self.http_parser.ok_Packs+=1
                self.http_parser.Set_Free()
            except Exception,err:
                rospy.logerr("[http-server][socket-io]:%s"%err)
                self.http_parser.err_Packs+=1
                self.http_parser.Set_Free()
if __name__=='__main__':
    server=Robot_Http_Server()
    rospy.spin()