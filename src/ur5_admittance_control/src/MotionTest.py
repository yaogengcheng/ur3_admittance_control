#! /usr/bin/env python
import sys
import rospy
import tf
import copy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           


from geometry_msgs.msg import PoseStamped

class MotionTest():
    def __init__(self):

        rospy.init_node('talker', anonymous=True)
        self.tf_listener = tf.TransformListener()
        self.pub = rospy.Publisher('/my_cartesian_compliance_controller/target_frame',PoseStamped, queue_size=3)
    
    def talker(self):

        self.rate = rospy.Rate(100)
        
        self.Pose_goal = PoseStamped()
        self.Pose_goal.header.frame_id = 'base_link'
        self.Pose_goal.pose = self.get_end_effector_pose()
        self.Pose_goal.header.stamp = rospy.Time(0)
        # self.Pose_goal.pose.position.x = 0   # red line      0.2   0.2
        # self.Pose_goal.pose.position.y = 0  # green line  0.15   0.15
        # self.Pose_goal.pose.position.z = 0  # blue line   # 0.35   0.6
        # self.Pose_goal.pose.orientation.x = 0
        # self.Pose_goal.pose.orientation.y = 0
        # self.Pose_goal.pose.orientation.z = 0
        # self.Pose_goal.pose.orientation.w =  0
        
        while not rospy.is_shutdown():
           
            self.pub.publish(self.Pose_goal)
            # print(self.Pose_goal)
            # print(self.Pose_goal)
            self.rate.sleep()
            self.Pose_goal.pose.position.x-=0.0001
            
    def get_end_effector_pose(self):
        try:
            self.tf_listener.waitForTransform(
                "base_link",
                "ee_link",
                rospy.Time(0),
                timeout=rospy.Duration(1),
                polling_sleep_duration=rospy.Duration(0.1)
                )
            pos, rot = self.tf_listener.lookupTransform(
                "base_link",
                "ee_link",
                rospy.Time(0))
        except tf.Exception as e:
            rospy.logwarn("{}".format(e))
            return None

        self.pose = Pose()
        self.pose.position.x = pos[0]
        self.pose.position.y = pos[1]
        self.pose.position.z = pos[2]
        self.pose.orientation.x = rot[0]
        self.pose.orientation.y = rot[1]
        self.pose.orientation.z = rot[2]
        self.pose.orientation.w = rot[3]
        print(self.pose)
        return self.pose
        
    def send_ee_link_pose(self):

        self.rate = rospy.Rate(100)
        self.pose_in_ee_link = PoseStamped()
        self.pose_in_ee_link.header.frame_id = 'ee_link'
        self.pose_in_ee_link.header.stamp = rospy.Time(0)
        self.pose_in_ee_link.pose.position.x = 0   # red line      0.2   0.2
        self.pose_in_ee_link.pose.position.y = 0  # green line  0.15   0.15
        self.pose_in_ee_link.pose.position.z = 0  # blue line   # 0.35   0.6
        self.pose_in_ee_link.pose.orientation.x = 0
        self.pose_in_ee_link.pose.orientation.y = 0
        self.pose_in_ee_link.pose.orientation.z = 0
        self.pose_in_ee_link.pose.orientation.w =  0

        while not rospy.is_shutdown():
           
            try:
                self.tf_listener.waitForTransform(  # Returns at once if transform exists
                        "base_link",
                        "ee_link",
                        rospy.Time(0),
                        timeout=rospy.Duration(1)
                        )
                self.pose_in_base = self.tf_listener.transformPose(
                        "base_link",
                        self.pose_in_ee_link)
            except tf.Exception as e:
                rospy.logwarn("{}".format(e))
                return

            self.pub.publish(self.pose_in_base)
            self.rate.sleep()
            self.pose_in_ee_link.pose.position.x+=0.00001

if __name__ == '__main__':
    try:
        node = MotionTest()
        # node.talker()
        node.send_ee_link_pose()
        # node.get_end_effector_pose()
    except rospy.ROSInterruptException:
        pass