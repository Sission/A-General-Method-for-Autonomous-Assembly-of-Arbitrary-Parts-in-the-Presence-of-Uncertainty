#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
import argparse
from uncertainty_reduction import *
from sensor_msgs.msg import JointState
from trac_ik_python.trac_ik import IK
from datetime import datetime
import time
from force_access import force_collect, error_record

parser = argparse.ArgumentParser()

# parser.add_argument("--robot", required=True)
# parser.add_argument("--mode", required=False)
parser.add_argument("--task", required=True)
parser.add_argument("--force", required=False)

# Read arguments from the command line
args = parser.parse_args()


def franka_joint_state_listener():
    # rospy.init_node('franka_joint_state_listener', anonymous=True)
    msg = rospy.wait_for_message("/joint_states", JointState)
    return np.asarray(msg.position)


def all_close(goal, actual, tolerance):
    """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
    # all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class RobotControl(object):
    def __init__(self, group_name):
        super(RobotControl, self).__init__()
        # initialize a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('panda_robot_controller', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        # Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the
        # surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface()
        # Instantiate a `MoveGroupCommander`_ object.
        self.group_name = group_name
        # group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()

    def go_to_joint_state(self, joints):
        joint_goal = self.move_group.get_current_joint_values()
        for i in range(len(joints)):
            joint_goal[i] = joints[i]
        # if len(joints) == 6:
        # joint_goal[3] += 0.005
        # For the 4-peg
        # joint_goal[3] += 0.01
        # joint_goal[4] += 0.015

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_configuration(self, config):
        rx, ry, rz, rw = ToQuaternion(config[0], config[1], config[2])
        # rx, ry, rz, rw = ToQuaternion(config[0], config[1] - 0.02, config[2] - 0.025)

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = rx
        pose_goal.orientation.y = ry
        pose_goal.orientation.z = rz
        pose_goal.orientation.w = rw
        pose_goal.position.x = config[3]
        pose_goal.position.y = config[4]
        pose_goal.position.z = config[5]
        self.move_group.set_pose_target(pose_goal)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def info(self):
        # print self.move_group.get_current_joint_values()
        print("============ End effector link: %s" % self.eef_link)
        print("============ Current Planning Group is:", self.group_name)
        print('The end_effector pose is', self.move_group.get_current_pose())
        print('rpy is', self.move_group.get_current_rpy())

    def get_joint_pose(self):
        return self.move_group.get_current_joint_values()

    def go_to_franka_joints_ik_solver(self, config, uncertainty=None, dc=None):
        solved_joints = franka_inverse_IK(config)
        try:
            len(solved_joints)
        except (Exception, ):
            error_record(dc, uncertainty[0], uncertainty[1], uncertainty[2])
            solved_joints = [0.4997515795569448, 0.5572374224495469, -0.5355297536441107,
                                                -1.90318553441031, 0.3916464849428998, 2.356481505155563,
                                                1.3200247102127614]
        self.go_to_joint_state(solved_joints)

    def go_to_irb_joints_ik_solver(self, config, uncertainty=None, dc=None):
        solved_joints = irb_inverse_IK(config)
        try:
            len(solved_joints)
        except (Exception, ):
            error_record(dc, uncertainty[0], uncertainty[1], uncertainty[2])
            solved_joints = irb_inverse_IK(np.array([0, np.pi / 2, -np.pi / 4, 0.54, 0, 0.206]))
        self.go_to_joint_state(solved_joints)




def franka_inverse_IK(config):
    ik_solver = IK('panda_link0', 'panda_link8')
    # format_printing("IK solver for joints:")
    # print(ik_solver.joint_names)
    seed_state = list(franka_joint_state_listener())

    # seed_state = [0.0] * 7
    rx, ry, rz, rw = ToQuaternion(config[0], config[1], config[2])
    solved_joints = ik_solver.get_ik(seed_state,
                                     config[3], config[4], config[5],  # X, Y, Z
                                     rx, ry, rz, rw)
    # print(solved_joints)
    return solved_joints


def irb_inverse_IK(config):
    ik_solver = IK('base_link', 'link_6')
    # format_printing("IK solver for joints:")
    # print(ik_solver.joint_names)
    # seed_state = list(franka_joint_state_listener())

    seed_state = [2.290331212861929e-05, 1.0513262748718262, -0.6549764275550842, 1.894644992717076e-05,
                  1.174484372138977, 0.7853364944458008]
    rx, ry, rz, rw = ToQuaternion(config[0], config[1], config[2])
    solved_joints = ik_solver.get_ik(seed_state,
                                     config[3], config[4], config[5],  # X, Y, Z
                                     rx, ry, rz, rw)
    # print(solved_joints)
    return solved_joints
    # print("IK solver uses link chain:")
    # print(ik_solver.link_names)
    #
    # print("IK solver base frame:")
    # print(ik_solver.base_link)
    #
    # print("IK solver tip link:")
    # print(ik_solver.tip_link)


def main():
    pass


if __name__ == '__main__':
    rospy.init_node("pose_control", log_level=rospy.FATAL)
    rate = rospy.Rate(1000)
    main()
