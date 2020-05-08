#!/usr/bin/env python
#coding=utf-8

from Tkinter import *
from ctrl_panel import *
import os
import threading
class UploadPanel:
    def __init__(self):
        self.input=Tk()
        self.input.title('请设置场景名称')
        self.input.geometry("+700+450")
        self.e = Entry(self.input,font=("黑体", 12))
        self.e.grid(row=0,column=1,padx=10,pady=5)
        self.data=""
        Button(self.input,text='提交',font=("黑体", 12),width=10,command=self.Upload)\
            .grid(row=0,column=2,padx=10,pady=5)
    def Upload(self):
        self.data=self.e.get()
        print self.data
        self.input.quit()
class CTRL:
    def __init__(self):
        self.root = Tk()
        self.root.title('机器人控制平台')
        self.root.geometry("+700+450")
        self.panel=None
        self.upload_Panel=None
        Button(self.root,font=("黑体", 12),text='高级地图',width=20,command=self.Panel)\
            .grid(row=0,column=0,sticky=W,padx=10,pady=5)
        Button(self.root,font=("黑体", 12),text='路线发布',width=20,command=self.Path)\
            .grid(row=1,column=0,sticky=W,padx=10,pady=5)
        Button(self.root,font=("黑体", 12),text='地图上传',width=20,command=self.Show_Upload_Map)\
            .grid(row=2,column=0,sticky=W,padx=10,pady=5)
        Button(self.root,font=("黑体", 12),text='路线发布',width=20,command=self.Path)\
            .grid(row=3,column=0,sticky=W,padx=10,pady=5)
        Button(self.root,font=("黑体", 12),text='外设开关',width=20,command=self.Show_Upload_Map)\
            .grid(row=4,column=0,sticky=W,padx=10,pady=5)
        Button(self.root,font=("黑体", 12),text='重定位',width=20,command=self.Show_Upload_Map)\
            .grid(row=5,column=0,sticky=W,padx=10,pady=5)
        Button(self.root,font=("黑体", 12),text='退出',width=20,command=self.root.destroy)\
            .grid(row=6,column=0,sticky=W,padx=10,pady=5)
        mainloop()
        
    def Show_Upload_Map(self):
        upload_Panel=UploadPanel()
    def Panel(self):
        if self.panel==None:
            self.panel=UI()
    def Path(self):
        if self.panel==None:
            self.panel=UI()
        self.panel.flag='path'
if __name__=='__main__':
    rospy.init_node('control_ui')
    cmd=CTRL()
    