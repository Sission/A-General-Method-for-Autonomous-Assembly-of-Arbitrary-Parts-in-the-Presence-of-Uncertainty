#! /usr/bin/env python3

import rospy
from franka_msgs.srv import SetForceTorqueCollisionBehavior, SetForceTorqueCollisionBehaviorRequest
from franka_msgs.srv import SetEEFrame, SetEEFrameRequest

# from franka_control.msg import ErrorRecoveryActionGoal
# !TODO: Check here
from franka_msgs.msg import ErrorRecoveryActionGoal

from franka_msgs.msg import FrankaState
from std_msgs.msg import Empty
import kinematics


def error_recovery():
    pub = rospy.Publisher('/franka_control/error_recovery/goal', ErrorRecoveryActionGoal, queue_size=1)
    rate = rospy.Rate(1)
    goal = ErrorRecoveryActionGoal()
    goal.goal = {}
    ctrl_c = False
    while not ctrl_c:
        connections = pub.get_num_connections()
        if connections > 0:
            pub.publish(goal)
            ctrl_c = True
            # kinematics.format_printing('Recovered from error')
        else:
            rate.sleep()


def set_defaults():
    rospy.wait_for_service('/franka_control/set_force_torque_collision_behavior')
    ftcb_srv = rospy.ServiceProxy('/franka_control/set_force_torque_collision_behavior', SetForceTorqueCollisionBehavior)
    ftcb_msg = SetForceTorqueCollisionBehaviorRequest()
    # ftcb_msg.lower_torque_thresholds_nominal = [50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0]
    ftcb_msg.upper_torque_thresholds_nominal = [50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0]
    # ftcb_msg.lower_force_thresholds_nominal = [50.0, 50.0, 50.0, 50.0, 50.0, 50.0]  # These will be used by the velocity controller to stop movement
    # ftcb_msg.upper_force_thresholds_nominal = [8.0, 8.0, 8.0, 8.0, 8.0, 8.0]
    #
    # ftcb_msg.lower_torque_thresholds_nominal = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
    # ftcb_msg.upper_torque_thresholds_nominal = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
    # ftcb_msg.lower_force_thresholds_nominal = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]  # These will be used by the velocity controller to stop movement
    ftcb_msg.upper_force_thresholds_nominal = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]

    res = ftcb_srv.call(ftcb_msg).success
    if not res:
        kinematics.format_printing('Failed to set Force/Torque Collision Behaviour Thresholds')
        print
    else:
        kinematics.format_printing('Successfully set Force/Torque Collision Behaviour Thresholds')
