#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class ArrowPublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('arrow_publisher', anonymous=True)

        # 创建一个Publisher对象，发布到'/visualization_marker'主题
        self.pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

        # 创建一个箭头列表
        self.arrows = []

    def add_arrow(self, start_point, end_point, frame_id, r=1.0, g=0.0, b=0.0, a=1.0):
        # 创建一个新的Marker对象
        arrow = Marker()
        arrow.header.frame_id = frame_id
        arrow.ns = "arrows"
        arrow.id = len(self.arrows)  # 为每个箭头设置唯一的ID
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD

        # 设置Marker尺寸
        arrow.scale.x = 0.01  # 箭头长度
        arrow.scale.y = 0.02  # 箭头宽度
        arrow.scale.z = 0.02  # 箭头高度

        # 设置Marker颜色
        arrow.color.r = r
        arrow.color.g = g
        arrow.color.b = b
        arrow.color.a = a  # 不透明度

        # 设置箭头的起点和终点
        arrow.points.append(start_point)
        arrow.points.append(end_point)

        # 将新箭头添加到列表中
        self.arrows.append(arrow)

    def publish_arrows(self):
        # 发布所有箭头
        while not rospy.is_shutdown():
            for arrow in self.arrows:
                arrow.header.stamp = rospy.Time.now()
                self.pub.publish(arrow)
            rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        arrow_publisher = ArrowPublisher()
        # 添加第一个箭头
        arrow_publisher.add_arrow(Point(0, 0, 0.0), Point(0, 0, 0.1), "link1_propeller_r")
        # 添加第二个箭头
        arrow_publisher.add_arrow(Point(0.0, 0, 0.0), Point(0.0, 0, 0.1), "link_b", r=0.0, g=1.0, b=0.0)
        # 添加第三 个箭头
        arrow_publisher.add_arrow(Point(0.0, 0, 0.0), Point(0.0, 0, 0.1), "link1_propeller_l", r=0.0, g=0.0, b=1.0)

        arrow_publisher.publish_arrows()
    except rospy.ROSInterruptException:
        pass