#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import WrenchStamped


def callback(data):
    if len(data.states) != 0:

        # rate = rospy.Rate(100)
        # target_wrench = WrenchStamped()
        # target_wrench.wrench = data.states.total_wrench
        # target_wrench.header.frame_id = 'ee_link'

        rospy.loginfo(rospy.get_caller_id() + "force_data: %s", data.states)


    # while not rospy.is_shutdown():
    #     self.pub.publish(self.target_wrench)
    #     # print(self.Pose_goal)
    #     self.rate.sleep()

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/robot_bumper", ContactsState, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
