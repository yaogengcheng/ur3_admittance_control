#! /usr/bin/env python

from controller_manager_msgs.srv import SwitchController
import rospy

controller_mode = rospy.get_param("/controller_mode")


rospy.wait_for_service('/controller_manager/switch_controller')
try:
    if controller_mode == 0:

        switch_controller = rospy.ServiceProxy(
                            '/controller_manager/switch_controller', SwitchController)
        ret = switch_controller(['my_cartesian_compliance_controller'], 
                                ['arm_controller'], 2)
    if controller_mode == 1:


        switch_controller = rospy.ServiceProxy(
                            '/controller_manager/switch_controller', SwitchController)
        ret = switch_controller(['arm_controller'], 
                                ['my_cartesian_compliance_controller'], 2)
    if controller_mode == 2:


        switch_controller = rospy.ServiceProxy(
                            '/controller_manager/switch_controller', SwitchController)
        ret = switch_controller(['arm_controller'], 
                                ['my_cartesian_motion_controller'], 2)
    
    if controller_mode == 3:


        switch_controller = rospy.ServiceProxy(
                            '/controller_manager/switch_controller', SwitchController)
        ret = switch_controller(['my_cartesian_motion_controller'], 
                                ['arm_controller'], 2)

except rospy.ServiceException, e:
    print "Service call failed: %s"%e