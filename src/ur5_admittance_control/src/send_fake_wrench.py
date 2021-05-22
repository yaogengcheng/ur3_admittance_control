#! /usr/bin/env python
import sys
import rospy
import tf
import copy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
import numpy as np

from geometry_msgs.msg import WrenchStamped
from scipy.spatial.transform import Rotation as R

class FakeWrenchSend():
    def __init__(self):

        rospy.init_node('send_fake_wrench', anonymous=True)
        self.tf_listener = tf.TransformListener()
        self.pub = rospy.Publisher('/my_cartesian_force_controller/ft_sensor_wrench',WrenchStamped, queue_size=3)
        rospy.Subscriber("/ft_sensor/netft_data", WrenchStamped, self.forceCallback)
        self.rate = rospy.Rate(100)
        self.target_wrench = WrenchStamped()
        self.tool_gravity = np.array([-0.0477138,0.0260671,-3.84994])
        self.sensor_zero = np.array([-0.0524807,0.0202237,-3.84746])


    def forceCallback(self,data):
  
        self.get_transform_matrix()
        force_bias = np.dot(self.Rotation_matrix,np.transpose(self.tool_gravity))
    
        self.target_wrench = data
        self.target_wrench.wrench.force.x = self.target_wrench.wrench.force.x-force_bias[0]-self.sensor_zero[0]
        self.target_wrench.wrench.force.y = self.target_wrench.wrench.force.y-force_bias[1]-self.sensor_zero[1]
        self.target_wrench.wrench.force.z = self.target_wrench.wrench.force.z-force_bias[2]-self.sensor_zero[2]
        self.target_wrench.header.frame_id = ''

    def get_transform_matrix(self):
        try:
            self.tf_listener.waitForTransform(
                "sensor_link",
                "base_link",
                rospy.Time(0),
                timeout=rospy.Duration(1),
                polling_sleep_duration=rospy.Duration(0.1)
                )
            pos,rot = self.tf_listener.lookupTransform(
                "sensor_link",
                "base_link",
                rospy.Time(0))
            self.Rotation_matrix = self.quaternion_to_rotation_matrix(rot) 
        except tf.Exception as e:
            rospy.logwarn("{}".format(e))
            return None   
        # print(pos)
        # print(rot)
        # print(self.Rotation_matrix)

    def print_transform_matrix(self):
        try:
            self.tf_listener.waitForTransform(
                "base_link",
                "ee_link",
                rospy.Time(0),
                timeout=rospy.Duration(1),
                polling_sleep_duration=rospy.Duration(0.1)
                )
            pos,rot = self.tf_listener.lookupTransform(
                "base_link",
                "ee_link",
                rospy.Time(0))
            self.Rotation_matrix = self.quaternion_to_rotation_matrix(rot) 
        except tf.Exception as e:
            rospy.logwarn("{}".format(e))
            return None   
        print(pos)
        print(rot)
        # print(self.Rotation_matrix)

    def quaternion_to_rotation_matrix(self,q):  # x, y ,z ,w
        rot_matrix = np.array(
            [[1.0 - 2 * (q[1] * q[1] + q[2] * q[2]), 2 * (q[0] * q[1] - q[3] * q[2]), 2 * (q[3] * q[1] + q[0] * q[2])],
            [2 * (q[0] * q[1] + q[3] * q[2]), 1.0 - 2 * (q[0] * q[0] + q[2] * q[2]), 2 * (q[1] * q[2] - q[3] * q[0])],
            [2 * (q[0] * q[2] - q[3] * q[1]), 2 * (q[1] * q[2] + q[3] * q[0]), 1.0 - 2 * (q[0] * q[0] + q[1] * q[1])]]
            )
        return rot_matrix

    def talker(self):

        self.rate = rospy.Rate(100)
        
        self.target_wrench = WrenchStamped()
        self.target_wrench.header.frame_id = ''
        self.target_wrench.wrench.force.x = 0
        self.target_wrench.wrench.force.y = 0
        self.target_wrench.wrench.force.z = -2.5

        self.target_wrench.wrench.torque.x =0
        self.target_wrench.wrench.torque.y =0
        self.target_wrench.wrench.torque.z =0
        
        while not rospy.is_shutdown():
            self.pub.publish(self.target_wrench)
            # print(self.Pose_goal)
            self.rate.sleep()
          
if __name__ == '__main__':
    try:
        node = FakeWrenchSend()
        # node.print_transform_matrix()
        while not rospy.is_shutdown():
            node.pub.publish(node.target_wrench)
            
            node.rate.sleep()
            # node.talker()
        # node.get_end_effector_pose()
    except rospy.ROSInterruptException:
        pass