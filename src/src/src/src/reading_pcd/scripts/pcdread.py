#!/usr/bin/python3
import rospy
import cv2
import pcl
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import numpy as np
import os
##### #/usr/bin/env python3 ############
# def point_deal(file):
#     print(file)
#     pt = pcl.load(file)
#     points = pt.to_array()
#     pointcloud1 = PointCloud()
#     pointcloud1.header.stamp = rospy.Time().now()
#     pointcloud1.header.frame_id = "world"
#     for p in points:
#         val_point = Point32()
#         (val_point.x, val_point.y, val_point.z) = (p[0],p[1],p[2])#p[0~3]分别为x,y,z坐标
#         pointcloud1.points.append(val_point)
#     return pointcloud1

def pictyre_deal(file):
    img = cv2.imread(file,0)
    lenght = 1
    height = 1
    pointcloud1 = PointCloud()
    pointcloud1.header.stamp = rospy.Time().now()
    pointcloud1.header.frame_id = "world"
    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            if img[i][j] == 0:
                # for x in np.arange(i, i+1+lenght, lenght):
                #     for y in np.arange(j, j +1+ lenght, lenght):
                        for z in np.arange(0, height+1+lenght, lenght):
                            val_point = Point32()
                            # (val_point.x, val_point.y, val_point.z) = (y,x,z)#p[0~3]分别为x,y,z坐标
                            (val_point.x, val_point.y, val_point.z) = (j,i,z)#p[0~3]分别为x,y,z坐标

                            pointcloud1.points.append(val_point)

    # # 边缘 是否显示
    # for i in range(len(img)):
    #     for x in np.arange(i, i + 1 + lenght, lenght):
    #         for y in np.arange(-1,  lenght, lenght):
    #             for z in np.arange(0, height+1+lenght, lenght):
    #                 val_point = Point32()
    #                 (val_point.x, val_point.y, val_point.z) = (y,x,z)#p[0~3]分别为x,y,z坐标
    #                 pointcloud1.points.append(val_point)
    #     for x in np.arange(i, i + 1 + lenght, lenght):
    #         for y in np.arange(len(img[0]),  len(img[0])+1+lenght, lenght):
    #             for z in np.arange(0, height+1+lenght, lenght):
    #                 val_point = Point32()
    #                 (val_point.x, val_point.y, val_point.z) = (y,x,z)#p[0~3]分别为x,y,z坐标
    #                 pointcloud1.points.append(val_point)
    # for j in range(len(img[0])):
    #     for x in np.arange(-1,  lenght, lenght):
    #         for y in np.arange(j, j +1+ lenght, lenght):
    #             for z in np.arange(0, height+1+lenght, lenght):
    #                 val_point = Point32()
    #                 (val_point.x, val_point.y, val_point.z) = (y,x,z)#p[0~3]分别为x,y,z坐标
    #                 pointcloud1.points.append(val_point)
    #     for x in np.arange(len(img), len(img)+1+lenght, lenght):
    #         for y in np.arange(j, j + 1 + lenght, lenght):
    #             for z in np.arange(0, height + 1 + lenght, lenght):
    #                 val_point = Point32()
    #                 (val_point.x, val_point.y, val_point.z) = (y,x,z)#p[0~3]分别为x,y,z坐标
    #                 pointcloud1.points.append(val_point)
    return pointcloud1





if __name__ == "__main__":
    rospy.init_node("map_pub")
    pcl_pub = rospy.Publisher('/point_map', PointCloud, queue_size=10)
    file_path = os.path.dirname(os.path.abspath(__file__))
    # mapfile   = rospy.get_param('mapfile',default= file_path + '/hard1.pcd')
    mapfile =rospy.get_param('~mapfile',file_path + '/middle0.5@0.5x.png')
    # mapfile = rospy.get_param('mapfile')
    # print(mapfile)
    pointcloud2 = pictyre_deal(mapfile)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        # rospy.loginfo("start map pub")
        pcl_pub.publish(pointcloud2)  #发布消息
        rate.sleep()  #休眠
    # rospy.spin()
