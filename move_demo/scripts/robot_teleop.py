#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelStates, ModelState
from move_demo.srv import MoveArm, MoveArmResponse
import sys, select, termios, tty
import threading
import tf.transformations  # 导入tf转换库

class RobotController(object):
    def __init__(self):
        rospy.init_node('robot_controller')
        self.current_x = 0.0  # 新增变量来存储当前x位置
        self.current_z = 0.0
        self.model_name = 'robot'  # 确保这个名称与Gazebo中的模型名称相匹配
        self.moving_up = False
        self.moving_down = False
        self.moving_forward = False  # 新增标志
        self.moving_backward = False  # 新增标志

        self.move_arm_srv = rospy.ServiceProxy('move_arm', MoveArm)

        # 订阅/gazebo/model_states主题以获取模型的状态
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)

    def model_states_callback(self, data):
        if self.model_name in data.name:
            index = data.name.index(self.model_name)
            self.current_x = data.pose[index].position.x
            self.current_y = data.pose[index].position.y
            self.current_z = data.pose[index].position.z
            orientation = data.pose[index].orientation
            euler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            self.current_yaw = euler[2]  # yaw是欧拉角的第三个元素

    def move_arm_client(self, type):

        try:
            # 调用服务并打印响应
            resp = self.move_arm_srv(type)
            # print("Service call returned: %s" % resp.result)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def move_up(self, z_speed=0.1):
        rospy.wait_for_service('/gazebo/set_model_state')
        set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.move_arm_client(1)
        self.move_arm_client(2)
        # rospy.sleep(2)

        while self.moving_up and not rospy.is_shutdown():
            self.current_z += z_speed * 0.01
            model_state = ModelState()
            model_state.model_name = self.model_name
            model_state.pose.position.x = self.current_x
            model_state.pose.position.z = self.current_z

            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.current_yaw)
            model_state.pose.orientation.x = quaternion[0]
            model_state.pose.orientation.y = quaternion[1]
            model_state.pose.orientation.z = quaternion[2]
            model_state.pose.orientation.w = quaternion[3]

            try:
                set_state_service(model_state)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
            rospy.sleep(0.01)

    def move_down(self, z_speed=0.1):
        rospy.wait_for_service('/gazebo/set_model_state')
        set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.move_arm_client(1)
        self.move_arm_client(3)
        # rospy.sleep(2)

        while self.moving_down and not rospy.is_shutdown():
            self.current_z -= z_speed * 0.01
            model_state = ModelState()
            model_state.model_name = self.model_name
            model_state.pose.position.x = self.current_x
            model_state.pose.position.z = self.current_z

            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.current_yaw)
            model_state.pose.orientation.x = quaternion[0]
            model_state.pose.orientation.y = quaternion[1]
            model_state.pose.orientation.z = quaternion[2]
            model_state.pose.orientation.w = quaternion[3]

            try:
                set_state_service(model_state)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
            rospy.sleep(0.01)

    def move_forward(self, x_speed=0.1):
        rospy.wait_for_service('/gazebo/set_model_state')
        set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.move_arm_client(6)
        self.move_arm_client(2)

        while self.moving_forward and not rospy.is_shutdown():
            self.current_x += x_speed * 0.01
            model_state = ModelState()
            model_state.model_name = self.model_name
            model_state.pose.position.x = self.current_x
            model_state.pose.position.z = self.current_z

            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.current_yaw)
            model_state.pose.orientation.x = quaternion[0]
            model_state.pose.orientation.y = quaternion[1]
            model_state.pose.orientation.z = quaternion[2]
            model_state.pose.orientation.w = quaternion[3]

            try:
                set_state_service(model_state)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
            rospy.sleep(0.01)

    def move_backward(self, x_speed=0.1):
        rospy.wait_for_service('/gazebo/set_model_state')
        set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.move_arm_client(5)
        self.move_arm_client(2)

        while self.moving_backward and not rospy.is_shutdown():
            self.current_x -= x_speed * 0.01
            model_state = ModelState()
            model_state.model_name = self.model_name
            model_state.pose.position.x = self.current_x
            model_state.pose.position.z = self.current_z

            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.current_yaw)
            model_state.pose.orientation.x = quaternion[0]
            model_state.pose.orientation.y = quaternion[1]
            model_state.pose.orientation.z = quaternion[2]
            model_state.pose.orientation.w = quaternion[3]

            try:
                set_state_service(model_state)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
            rospy.sleep(0.01)

    def rotate_yaw(self, yaw_speed=0.1, direction="left"):
        rospy.wait_for_service('/gazebo/set_model_state')
        set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.move_arm_client(4)
        self.move_arm_client(2)

        while self.turn_angle and not rospy.is_shutdown():

            # 根据旋转方向调整偏航角速度的符号
            if direction == "left":
                self.current_yaw += yaw_speed * 0.01
            else:  # direction == "right"
                self.current_yaw -= yaw_speed * 0.01

            # 确保偏航角在-π到π之间
            self.current_yaw = (self.current_yaw + math.pi) % (2 * math.pi) - math.pi
            # 将偏航角转换为四元数
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.current_yaw)
            # 更新模型状态
            model_state = ModelState()
            model_state.model_name = self.model_name
            model_state.pose.position.x = self.current_x
            model_state.pose.position.z = self.current_z
            model_state.pose.orientation.x = quaternion[0]
            model_state.pose.orientation.y = quaternion[1]
            model_state.pose.orientation.z = quaternion[2]
            model_state.pose.orientation.w = quaternion[3]

            try:
                set_state_service(model_state)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)

            rospy.sleep(0.01)

    def start_moving(self, direction):

        if direction == "up":
            if not self.moving_up:
                self.moving_up = True
                self.moving_down = False 
                threading.Thread(target=self.move_up).start()
                
        elif direction == "down":
            if not self.moving_down:
                self.moving_down = True
                self.moving_up = False
                threading.Thread(target=self.move_down).start()

        elif direction == "forward":
            if not self.moving_forward:
                self.moving_forward = True
                self.moving_backward = False
                threading.Thread(target=self.move_forward).start()

        elif direction == "backward":
            if not self.moving_backward:
                self.moving_backward = True
                self.moving_forward = False
                threading.Thread(target=self.move_backward).start()

        elif direction == "left" or direction == "right":
            self.turn_angle = True
            threading.Thread(target=self.rotate_yaw, args=(0.1, direction)).start()

    def stop_moving(self):
        self.moving_up = False
        self.moving_down = False
        self.moving_forward = False
        self.moving_backward = False
        self.turn_angle = False

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    robot_controller = RobotController()

    try:
        print("Press 'q' to move up, 'e' to move down, 'w' to move forward, 's' to move backward, press Space to stop. Press CTRL+C to exit.")
        while True:
            key = robot_controller.getKey()
            if key == 'q':
                robot_controller.start_moving("up")
            elif key == 'e':
                robot_controller.start_moving("down")
            elif key == 'w':
                robot_controller.start_moving("forward")
            elif key == 's':
                robot_controller.start_moving("backward")
            elif key == 'a':
                robot_controller.start_moving("left")
            elif key == 'd':
                robot_controller.start_moving("right")
            elif key == ' ':
                robot_controller.stop_moving()
            elif key == '\x03':
                robot_controller.stop_moving()
                break

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)