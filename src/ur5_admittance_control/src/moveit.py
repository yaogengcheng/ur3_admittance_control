#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
# from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MoveItIkDemo:

    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')

        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('manipulator')

        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()

        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)

        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.05)

        # 控制机械臂先回到初始化位置
        # arm.set_named_target('home')
        # arm.go()
        # rospy.sleep(2)
        
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        Pose_goal = arm.get_current_pose()
        Pose_goal.header.frame_id = 'base_link'
        Pose_goal.header.stamp = rospy.Time(0)
        Pose_goal.pose.position.x = -0.187001019268   # red line      0.2   0.2
        Pose_goal.pose.position.y = 0.362908560006  # green line  0.15   0.15
        Pose_goal.pose.position.z = 0.322905034903  # blue line   # 0.35   0.6
        Pose_goal.pose.orientation.x = 0
        Pose_goal.pose.orientation.y = 0
        Pose_goal.pose.orientation.z = 0
        Pose_goal.pose.orientation.w = 0
        arm.set_start_state_to_current_state()
        # 设置机械臂终端运动的目标位姿
        # arm.set_pose_target(Pose_goal, "ee_link")

        # 规划运动路径
        # traj = arm.plan()
        # arm.execute(traj)
        # rospy.sleep(5)
    
        print(arm.get_current_pose().pose)

        # current_pose = arm.get_current_pose()
        # for i in range(0,100):
        #     current_pose.pose.position.z -=0.005
        #     current_pose.header.frame_id = 'base_link'
        #     current_pose.header.stamp = rospy.Time(0)
        #     # 设置机器臂当前的状态作为运动初始状态
        #     arm.set_start_state_to_current_state()
            
        #     # 设置机械臂终端运动的目标位姿
        #     arm.set_pose_target(current_pose, end_effector_link)

        #     # 规划运动路径
        #     traj = arm.plan()
        #     arm.execute(traj)
        #     rospy.sleep(3)
        # 控制机械臂终端向x移动5cm 012345:xyzrpy

               
        joint_positions = [1.0472, -1.5708, 1.5708, -1.5708, -1.5708, 0]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(5)
        print(arm.get_current_pose().pose)
        # for i in range(0,10):
        #     arm.shift_pose_target(2, 0.002, end_effector_link)
        #     arm.go()
        #     rospy.sleep(5)
 

        # 控制机械臂终端反向旋转90度
        # arm.shift_pose_target(3, -1.57, end_effector_link)
        # arm.go()
        # rospy.sleep(1)

        # 控制机械臂回到初始化位置
        # arm.set_named_target('home')
        # arm.go()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    move = MoveItIkDemo()
    move.__init__()