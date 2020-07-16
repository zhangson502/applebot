#!/usr/bin/env python
# -*- coding=utf-8 -*-
'''costmap_generator ROS Node'''
# license removed for brevity
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import matrix_processor
from laser_preprocessor import *
from sonar_preprocessor import *
from pcl_processor import *
import threading
class Costmap_Manager:
    def __init__(self):
        self.costmap_Width=rospy.get_param('costmap_width',80)
        self.costmap_Height=rospy.get_param('costmap_height',80)
        self.costmap_Resolution=rospy.get_param('costmap_Resolution',0.05)
        self.publish_Frequency=rospy.get_param('costmap_publish_frequency',5)
        self.base_link='base_link'
        '''
            构建激光类，该类会使用回调函数时刻更新激光雷达的数据
        '''
        self.laser=LaserPtns2Matrix(width=self.costmap_Width,height=self.costmap_Height,resolution=self.costmap_Resolution)
        '''
            模拟构建激光，将D435的数据抽象为激光点数据
        '''
        self.depth=LaserPtns2Matrix(width=self.costmap_Width,height=self.costmap_Height,resolution=self.costmap_Resolution,scan_Topic='depth_simulated_scan')
        #self.depth=
        self.costmap=OccupancyGrid()
        self.costmap_Publisher=rospy.Publisher('sensor_costmap',OccupancyGrid,queue_size=5)
        pub_Thread=threading.Thread(target=self.Update_Loop,args=())
        pub_Thread.daemon=True
        pub_Thread.start()
    def UpDate_CostMap(self):
        self.costmap.header.frame_id=self.base_link
        self.costmap.header.seq+=1
        self.costmap.header.stamp=rospy.Time.now()
        #配置地图参数
        self.costmap.info.height=self.costmap_Height
        self.costmap.info.width=self.costmap_Width
        self.costmap.info.resolution=self.costmap_Resolution
        self.costmap.info.origin.position.x=-(self.costmap_Width*self.costmap_Resolution)/(2.0)
        self.costmap.info.origin.position.y=-(self.costmap_Height*self.costmap_Resolution)/(2.0)
        self.costmap.data=[]
        '''
            融合多张矩阵地图（调用Open的Mask覆叠，将多个矩阵覆叠到一起，形成完整的costmap矩阵），具体步骤如下：
        '''
        # 1. 更新各个传感器所获取的栅格地图数据
        matrix_1=self.laser.Process_Laser_Frame()
        matrix_2=self.depth.Process_Laser_Frame()
        # 2. 各个传感器的数据进行融合
        #   2.1 首先，使用opencv的mask以及与或非关系对地图进行覆叠
        #   2.2 覆叠后，使用opencv的Erode 或Dialation函数实现对障碍点的膨胀处理

        # 3. 逐个像素进行处理,生成最终的costmap
        matrix_Compromised=matrix_1
        for y in range(len(matrix_Compromised)):
            for x in range(len(matrix_Compromised[0])):
                self.costmap.data.append(matrix_Compromised[y][x])
        #发布
        self.costmap_Publisher.publish(self.costmap)
    def Update_Loop(self):
        r=rospy.Rate(self.publish_Frequency)
        while True:
            r.sleep()
            self.UpDate_CostMap()
if __name__ == '__main__':
    rospy.init_node('costmap_generator')
    cm=Costmap_Manager()
    rospy.spin()


