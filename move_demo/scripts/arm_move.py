#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from move_demo.srv import MoveArm, MoveArmResponse  # 确保已经正确创建了MoveArm服务文件
from moveit_commander import MoveGroupCommander

class MoveArmServer:
    def __init__(self):
        self.group_name = "fans"  # 假设你的planning group名称为"fans"
        self.move_group = MoveGroupCommander(self.group_name)
        self.service = rospy.Service('move_arm', MoveArm, self.handle_move_arm)
        print("Ready to move arm.")

    def handle_move_arm(self, req):
        if req.type == 1:

            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[1] = 3.14
            joint_goal[2] = 1.57
            joint_goal[4] = 3.14
            joint_goal[5] = 1.57

            self.move_group.go(joint_goal, wait=True)
            self.move_group.stop()
            return MoveArmResponse(True)
        
        if req.type == 2:
            self.move_group.set_max_acceleration_scaling_factor(0.3)
            self.move_group.set_max_velocity_scaling_factor(0.3)
            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[0] += 5000
            joint_goal[3] += 5000
            joint_goal[6] += 5000
            self.move_group.go(joint_goal, wait=False)
            self.move_group.stop()
            return MoveArmResponse(True)

        if req.type == 3:
            self.move_group.set_max_acceleration_scaling_factor(0.3)
            self.move_group.set_max_velocity_scaling_factor(0.3)
            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[0] -= 5000
            joint_goal[3] -= 5000
            joint_goal[6] -= 5000
            self.move_group.go(joint_goal, wait=False)
            self.move_group.stop()
            return MoveArmResponse(True)

        if req.type == 4:

            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[1] = 3.14
            joint_goal[2] = 0
            joint_goal[4] = 3.14
            joint_goal[5] = 0

            self.move_group.go(joint_goal, wait=True)
            self.move_group.stop()
            return MoveArmResponse(True)
        
        if req.type == 5:

            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[1] = 0
            joint_goal[2] = 3.14
            joint_goal[4] = 0
            joint_goal[5] = 0

            self.move_group.go(joint_goal, wait=True)
            self.move_group.stop()
            return MoveArmResponse(True)

        if req.type == 6:

            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[1] = 0
            joint_goal[2] = 0
            joint_goal[4] = 3.14
            joint_goal[5] = 0

            self.move_group.go(joint_goal, wait=True)
            self.move_group.stop()
            return MoveArmResponse(True)
        
        else:
            return MoveArmResponse(False)

def main():
    rospy.init_node('move_arm_server')
    server = MoveArmServer()
    rospy.spin()

if __name__ == "__main__":
    main()