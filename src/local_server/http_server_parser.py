#!/usr/bin/env python
# -*- coding:utf-8 -*-
import os
import rospy
import time
import json
from std_msgs.msg import String
from ros_parser import ROS_Parser
from map_parser import Map_Parser
from cam_parser import Cam_Parser
import json_handler
# 返回码
class ErrorCode(object):
    OK = "HTTP/1.1 200 OK\r\n"
    NOT_FOUND = "HTTP/1.1 404 Not Found\r\n"
    ILLEGAL = "HTTP/1.1 401 Not Authorized\r\n"
    NO_MAP = "HTTP/1.1 203 Map Invalid\r\n"
# Content类型
class ContentType(object):
    HTML = 'Content-Type: text/html\r\n'
    JSON = 'Content-Type: text/json\r\n'
    PNG = 'Content-Type: img/png\r\n'
    JPEG = 'Content-Type: image/jpeg\r\n'
class Std_Res:
    OK='{"ret:"ok"}'
    FAIL='{"ret:"fail"}'
    LOGIN_OK={'login':'success','ret':'ok'}
    LOGIN_FAIL={"ret": "fail", "error": "robot locked!"}
class HttpRequest(object):
    RootDir = rospy.get_param('www_root_dir','/home/firefly/RoboStudioV5/src/interfaces/local_server/resource/html')
    NotFoundHtml = RootDir+'/'+'404.html'  # 404页面
    def __init__(self):
        self.method = None
        self.url = None
        self.protocol = None
        self.host = None
        self.request_data = None
        self.response_line = ErrorCode.OK  # 响应码
        self.response_head = ContentType.HTML # 响应头部
        self.response_body = '' # 响应主题
        #大量数据接口
        self.cam_parser=Cam_Parser()
        self.ros_parser=ROS_Parser()
        #缓存变量
        self.ok_Packs=0.0
        self.err_Packs=0.0
        self.busy=False
        self.locked=False
    def Set_Busy(self):
        self.busy=True
    def Set_Free(self):
        self.busy=False
    # 解析请求，得到请求的信息
    def parse_request(self, request):
        self.response_line = ErrorCode.OK  # 响应码
        self.response_head = ContentType.HTML # 响应头部
        self.response_body = '' # 响应主题
        request_line, body = request.split('\r\n', 1)
        header_list = request_line.split(' ')
        self.method = header_list[0].upper()
        self.url = header_list[1]
        self.protocol = header_list[2]
        # 获得请求参数
        if self.method == 'POST':
            self.request_data = {}
            request_body = body.split('\r\n\r\n', 1)[1]
            self.request_data=request_body
            self.handle_post_request(self.url)
        if self.method == 'GET':
            file_name = ''
            self.handle_get_request(self.url)
    '''
        POST请求处理函数
    '''
    def handle_post_request(self,url):
        self.response_line = ErrorCode.OK
        self.response_head = ContentType.HTML
        self.response_body = Std_Res.OK
        try:
            #验证请求信息
            if url=='/login':
                #登录：
                self.locked=True
                name,key=json_handler.Resolve_Login(self.request_data)
                auth=self.ros_parser.Authorization(name,key)
                if auth[0]!=0:
                    self.response_line=ErrorCode.OK
                    self.response_body=json.dumps(Std_Res.OK)
                    self.locked=False
            if self.locked:
                self.response_line = ErrorCode.ILLEGAL
                self.response_body = json.dumps(Std_Res.LOGIN_FAIL)
                return
            if url=='/cmd_vel':
                #原始速度控制
                v,vth=json_handler.Resolve_Spd(self.request_data)
                self.response_body=self.ros_parser.Manual(v,vth)
            if url=='/draw_new_map':
                ret=self.ros_parser.start_Draw_Map_Client()
                if ret.ret!=0:
                    self.response_body=Std_Res.FAIL
            if url=='/map_save':
                #保存地图
                name=json_handler.Resolve_Map_Name(self.request_data)
                ret,retStr=self.ros_parser.Map_Save(strr=name)
                if ret.ret!=0:
                    self.response_body=Std_Res.FAIL
            if url=='/map':
                #地图位图加载控制
                #请求地图
                rospy.loginfo("[http-server][map-io]%s"%self.request_data)
                name=json_handler.Resolve_Map_Name(self.request_data)
                if name=='None' or name=='':
                    if not self.ros_parser.have_Map:
                        self.response_line=ErrorCode.NO_MAP
                        self.response_body=Std_Res.FAIL
                        return
                    self.response_head=ContentType.JPEG
                    self.response_body=self.ros_parser.Call4JPEGMAP()
                else:
                #加载新地图
                    self.ros_parser.have_Map=False
                    ret,retStr=self.ros_parser.Set_Scene(name)
                    if ret==-1:
                        self.response_line=ErrorCode.NO_MAP
                        self.response_body=Std_Res.FAIL
                        return
                    while self.ros_parser.have_Map==False:time.sleep(0.05)
                    self.response_body=self.ros_parser.Call4JPEGMAP()
            if url=='/map_config':
                name=json_handler.Resolve_Map_Name(self.request_data)
                o_x,o_y,h,w,homes,areas,lines=self.ros_parser.Call4MAPConfig()
                self.response_body=json_handler.Create_Map_Config(o_x,o_y,h,w,homes,areas,lines)
            if url=='/relocate':
                #重定位
                name=json_handler.Resolve_Map_Name(self.request_data)
                x,y,yaw=json_handler.Resolve_2D_Position(self.request_data)
                self.ros_parser.have_Map=False
                if self.ros_parser.Set_Scene(name)[1]!=0:
                    self.response_line=ErrorCode.NO_MAP
                    self.response_body=Std_Res.FAIL
                    return
                while self.ros_parser.have_Map==False:time.sleep(0.05)
                self.ros_parser.Set_Robot_Pose(x=x,y=y,yaw=yaw)
                self.response_body=ErrorCode.OK
            
            if url=='/go_path':
                #固定路径行走
                idx=json_handler.Resolve_id_str(self.request_data)
                self.ros_parser.Gen_Path(idx)
            if url=='/save_path':
                #保存一条固定路线
                idx=json_handler.Resolve_id_str(self.request_data)
                self.ros_parser.Save_Path(idx)
            if url=='/clean_area':
                #画区域清扫
                self.ros_parser.AreaClean(json_handler.Resolve_Area(self.request_data))
            if url=='/robot_clean_cancel':
                rospy.logwarn('[http-server][parser]Emergency aborting called!')
                self.ros_parser.Cancel_Action()
        except Exception,err:
            rospy.logerr("[http-server][parser]%s"%err)
            self.response_head=ErrorCode.NOT_FOUND
            self.response_body="ERROR:%s"%err
    '''
        GET请求处理函数
    '''
    def handle_get_request(self, url):
        self.response_line = ErrorCode.OK
        self.response_head = ContentType.HTML
        self.response_body = Std_Res.OK
        try:
            if url=='/map':
                self.response_head=ContentType.JPEG
                self.response_body=self.ros_parser.Call4JPEGMAP()
            if url=='/robot_state':
                x,y,yaw=self.ros_parser.Get_TF()
                self.response_body=json_handler.Create_Hardware_State(xx=x,yy=y,y=yaw,area=self.ros_parser.Call4Scene_Info()[0])
            if url=='/map_list':
                    #请求地图列表
                    code,dat=self.ros_parser.Call4Maplist()
                    if code==0: self.response_body=dat
                    else:self.response_line=ErrorCode.NO_MAP
        except Exception,err:
            rospy.logerr("[http-server][parser]%s"%err)
            self.response_head=ErrorCode.NOT_FOUND
            self.response_body="ERROR:%s"%err
    def get_response(self):
        return self.response_line+self.response_head+'\r\n'+self.response_body