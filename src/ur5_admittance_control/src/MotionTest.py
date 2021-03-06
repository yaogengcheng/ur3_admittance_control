#! /usr/bin/env python
# encoding: utf-8
import sys
import rospy
from rospy.core import is_shutdown
import tf
import copy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion, WrenchStamped                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
import math
import socket
import threading

from geometry_msgs.msg import PoseStamped
if sys.version > '3':
    import queue as Queue
else:
    import Queue 

class MotionTest():
    def __init__(self):

        rospy.init_node('talker', anonymous=True)
        self.tf_listener = tf.TransformListener()
        self.pub = rospy.Publisher('/my_cartesian_motion_controller/target_frame',PoseStamped, queue_size=3)
        rospy.Subscriber("/my_cartesian_force_controller/ft_sensor_wrench", WrenchStamped, self.forceCallback)
        self.total_wrench = 5
        self.q = Queue.Queue(maxsize=0)
        # t1 = threading.Thread(target=self.connect_server)
        # t1.start()
        self.R = threading.Lock()
        self.rate = rospy.Rate(100)
        self.forceJudgeStart = False
        self.pose_in_ee_link = PoseStamped()
        self.pose_in_ee_link.header.frame_id = 'ee_link'
        self.pose_in_ee_link.header.stamp = rospy.Time(0)
        self.pose_in_ee_link.pose.position.x = 0   # red line      
        self.pose_in_ee_link.pose.position.y = 0  # green line 
        self.pose_in_ee_link.pose.position.z = 0  # blue line   
        self.pose_in_ee_link.pose.orientation.x = 0
        self.pose_in_ee_link.pose.orientation.y = 0
        self.pose_in_ee_link.pose.orientation.z = 0
        self.pose_in_ee_link.pose.orientation.w =  0

        self.qGive()
    
    # data test
    def qGive(self):
        data = "0.052003436415528445 0.3147654951162265 0.3136344705861339 -0.35354257881860196 0.6123754446634613 0.35355443813683707 0.6123750640002141"
        res =list(map(float, data.strip().split()))
        self.q.put(res)
        data = "-0.17717377910067827 0.4050774555404048 0.2006210893186674 -0.17747670667733628 0.8284618113788571 0.4564687929479293 0.2716418353981762"
        res =list(map(float, data.strip().split()))
        self.q.put(res)
        data = "-0.18984737311234284 0.344850473075425 0.2101062611060021 -0.13111734260233388 0.8028691090658835 0.4974692246124217 0.30122052841758973"
        res =list(map(float, data.strip().split()))
        self.q.put(res)
        data = "-0.22274431207804146 0.2704907297973187 0.20540778388273195 0.0902137463168064 0.808465993614266 0.3968361659076245 0.42516499688064757"
        res =list(map(float, data.strip().split()))
        self.q.put(res)

    def connect_server(self):
        
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(('127.0.0.1', 8999))
        while True:
            data = s.recv(1024)
            if len(data)>0:
                res =list(map(float, data.strip().split()))
                # print(res) 
                if len(res)==7:
                    self.q.put(res)
                else:
                    print("==========receive data length error=============")
        # ?????? socket
        s.close()

    def forceCallback(self,data):
        self.total_wrench = math.sqrt(math.pow(data.wrench.force.x,2)+math.pow(data.wrench.force.y,2)+math.pow(data.wrench.force.z,2))
        print("=============total wrench==============:")
        print(self.total_wrench)

    def talker(self):
        if self.q.not_empty:
            self.count = 0
            self.pose = copy.deepcopy(self.q.get())
            self.next_Pose_goal = PoseStamped()
            self.next_Pose_goal.header.frame_id = 'base_link'
            self.next_Pose_goal.header.stamp = rospy.Time(0)
            self.next_Pose_goal.pose.position.x = self.pose[0]   # red line      0.2   0.2
            self.next_Pose_goal.pose.position.y = self.pose[1]  # green line  0.15   0.15
            self.next_Pose_goal.pose.position.z = self.pose[2]  # blue line   # 0.35   0.6
            self.next_Pose_goal.pose.orientation.x = self.pose[3]
            self.next_Pose_goal.pose.orientation.y = self.pose[4]
            self.next_Pose_goal.pose.orientation.z = self.pose[5]
            self.next_Pose_goal.pose.orientation.w = self.pose[6]
            print(self.next_Pose_goal)

            self.rate = rospy.Rate(100)
            self.Pose_goal = PoseStamped()
            self.Pose_goal.header.frame_id = 'base_link'
            self.Pose_goal.pose = self.get_end_effector_pose()
            self.Pose_goal.header.stamp = rospy.Time(0)
            # self.Pose_goal.pose.position.x = 0   # red line      0.2   0.2
            # self.Pose_goal.pose.position.y = 0  # green line  0.15   0.15
            # self.Pose_goal.pose.position.z = 0  # blue line   # 0.35   0.6
            # self.Pose_goal.pose.orientation.x = 0.2824985249800409
            # self.Pose_goal.pose.orientation.y = -0.9123432211863445
            # self.Pose_goal.pose.orientation.z = -0.2824946198034697
            # self.Pose_goal.pose.orientation.w = 0.08956126351006068
            
            self.pathPlan()
            
    def pathPlan(self):
        self.Pose_goal.pose = self.get_end_effector_pose()
        self.target_Pose_goal = copy.deepcopy(self.Pose_goal)
        self.pathLength = math.sqrt(math.pow((self.Pose_goal.pose.position.x-self.next_Pose_goal.pose.position.x), 2)+math.pow((self.Pose_goal.pose.position.y-self.next_Pose_goal.pose.position.y), 2)+math.pow((self.Pose_goal.pose.position.z-self.next_Pose_goal.pose.position.z), 2))
        self.radio = int(self.pathLength/0.0001)
        # while not rospy.is_shutdown():
        # self.radio=100
        for i in range(1,self.radio+1):
            if not rospy.is_shutdown():
                if self.q.qsize<3 and self.count%1 == 0 and (self.total_wrench<4 or self.total_wrench>8):
                    # print("jinru")
                    while (self.total_wrench<4 or self.total_wrench>8) and not rospy.is_shutdown(): 
                        self.send_ee_link_pose()
                    self.count=0
                    self.pathPlan()
                    break
            # if self.total_wrench<6:
                self.count+=1
                self.target_Pose_goal.pose.position.x += (self.next_Pose_goal.pose.position.x - self.Pose_goal.pose.position.x)/self.radio
                self.target_Pose_goal.pose.position.y += (self.next_Pose_goal.pose.position.y - self.Pose_goal.pose.position.y)/self.radio
                self.target_Pose_goal.pose.position.z += (self.next_Pose_goal.pose.position.z - self.Pose_goal.pose.position.z)/self.radio

                self.target_Pose_goal.pose.orientation = self.slerp(self.Pose_goal.pose.orientation, self.next_Pose_goal.pose.orientation, 1.0*i/self.radio)
                # print(self.target_Pose_goal.pose.orientation)
                self.pub.publish(self.target_Pose_goal)
                # print(self.Pose_goal)
                self.rate.sleep()
        

    def slerp(self,start,end,t):
        Pose_goal = copy.deepcopy(start)
        next_Pose_goal = copy.deepcopy(end)

        cosa = Pose_goal.x*next_Pose_goal.x + Pose_goal.y*next_Pose_goal.y + Pose_goal.z*next_Pose_goal.z + Pose_goal.w*next_Pose_goal.w

        if  cosa < 0.:  
            next_Pose_goal.x = -next_Pose_goal.x
            next_Pose_goal.y = -next_Pose_goal.y
            next_Pose_goal.z = -next_Pose_goal.z
            next_Pose_goal.w = -next_Pose_goal.w
            cosa = -cosa
    
        # If the inputs are too close for comfort, linearly interpolate
        if cosa > 0.9995:
            k0 = 1 - t
            k1 = t
        else: 
            sina = math.sqrt( 1 - cosa*cosa)
            a = math.atan2( sina, cosa )
            k0 = math.sin((1 - t)*a)  / sina
            k1 = math.sin(t*a) / sina
        
        Target_pose = Pose()
        Target_pose.orientation.x = Pose_goal.x*k0 + next_Pose_goal.x*k1
        Target_pose.orientation.y = Pose_goal.y*k0 + next_Pose_goal.y*k1
        Target_pose.orientation.z = Pose_goal.z*k0 + next_Pose_goal.z*k1
        Target_pose.orientation.w = Pose_goal.w*k0 + next_Pose_goal.w*k1
        return Target_pose.orientation
    
    # def quaternion2euler(self,quaternion):
    #     x = quaternion.x
    #     y = quaternion.y
    #     z = quaternion.z
    #     w = quaternion.w
    #     r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    #     r = r / math.pi * 180
    #     p = math.asin(2 * (w * y - z * x))
    #     p = p / math.pi * 180
    #     y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    #     y = y / math.pi * 180
    #     return r,p,y

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
        # print(self.pose)
        return self.pose
        
    def send_ee_link_pose(self):

        # while not rospy.is_shutdown():
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
        if self.total_wrench>8:
            self.pose_in_ee_link.pose.position.x-=0.000001
        if self.total_wrench<4:
            self.pose_in_ee_link.pose.position.x+=0.000001

if __name__ == '__main__':
    try:
        node = MotionTest()
        while not rospy.is_shutdown():
            node.talker()
            # node.send_ee_link_pose()
            # node.get_end_effector_pose()
    except rospy.ROSInterruptException:
        pass