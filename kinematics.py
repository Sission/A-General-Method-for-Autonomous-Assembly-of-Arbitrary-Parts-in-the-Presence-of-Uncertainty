#!/usr/bin/env python3


import math
import numpy as np
import os


# TODO: working on z direction

# roll - X
# pitch - Y
# yaw -Z

def rot_z(yaw):
    rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    return rz


def rot_y(pitch):
    ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    return ry


def rot_x(roll):
    rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    return rx


def rpy(roll, pitch, yaw):
    r = np.einsum('ij,jk,km', rot_z(yaw), rot_y(pitch), rot_x(roll))
    return r


def diff_yaw(yaw):
    dy = np.array([[-np.sin(yaw), -np.cos(yaw), 0],
                   [np.cos(yaw), -np.sin(yaw), 0],
                   [0, 0, 0]])
    return dy


def diff_pitch(pitch):
    dp = np.array([[-np.sin(pitch), 0, np.cos(pitch)],
                   [0, 0, 0],
                   [-np.cos(pitch), 0, -np.sin(pitch)]])
    return dp


def diff_roll(roll):
    dr = np.array([[0, 0, 0],
                   [0, -np.sin(roll), -np.cos(roll)],
                   [0, np.cos(roll), -np.sin(roll)]])
    return dr


def homo(config):
    roll = config[0]
    pitch = config[1]
    yaw = config[2]
    x = config[3]
    y = config[4]
    z = config[5]
    rotation = rpy(roll, pitch, yaw)
    zero = np.array([[0, 0, 0]])
    d = np.array([[x], [y], [z], [1]])
    a = np.vstack([rotation, zero])
    h = np.hstack([a, d])
    return h


def inverse_homo(H):
    R = np.transpose(H[0:3, 0:3])
    d = - np.matmul(R, H[0:3, 3]).reshape((3, 1))
    zero = np.array([[0, 0, 0, 1]])
    h = np.hstack((R, d))
    T = np.vstack([h, zero])
    return T


def ToQuaternion(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


# noinspection DuplicatedCode
def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    if x < 0:
        x += 2 * np.pi
    if y < 0:
        y += 2 * np.pi
    if z < 0:
        z += 2 * np.pi
    return np.array([x, y, z])


def fkine(S, M, q):
    T = np.eye(4)
    n = S.shape[1]
    for i in range(n):
        T = np.matmul(T, twist2ht(S[:, i], q[i]))
    T = np.matmul(T, M)
    return T


def refine_rpy(config):
    with np.nditer(config[0:3], op_flags=['readwrite']) as it:
        for x in it:
            if x > np.pi:
                x[...] -= 2 * np.pi
            elif x < -np.pi:
                x[...] += 2 * np.pi
    return np.asarray(config)


def matrix_to_config(T):
    rpy_v = rotationMatrixToEulerAngles(T)
    config = np.hstack((rpy_v, T[0:3, 3].T))
    config = refine_rpy(config)
    return config


def twist2ht(S, theta):
    omgR = omgmat(S[0:3])
    R = axisangle2rot(S[0:3], theta)
    v = np.eye(3) * theta + (1 - np.cos(theta)) * omgR + (theta - np.sin(theta)) * np.matmul(omgR, omgR)
    d = np.dot(v, S[3:6]).reshape((3, 1))
    scale = np.array([0, 0, 0, 1])
    temp = np.hstack((R, d))
    T = np.vstack((temp, scale))
    return T


def omgmat(w):
    M = np.array([[0, -w[2], w[1]],
                  [w[2], 0, -w[0]],
                  [-w[1], w[0], 0]])
    return M


def axisangle2rot(w, theta):
    omega_ss = omgmat(w)
    R = np.eye(3) + np.sin(theta) * omega_ss + (1 - np.cos(theta)) * np.matmul(omega_ss, omega_ss)
    return R


def forward_kine(S, M, q, add_trans):
    ee_trans_T = fkine(S, M, q)
    add_trans_T = np.matmul(ee_trans_T, add_trans)
    ee_rpy = rotationMatrixToEulerAngles(ee_trans_T)
    add_rpy = rotationMatrixToEulerAngles(add_trans_T)
    ee_config = np.hstack((ee_rpy, ee_trans_T[0:3, 3].T))
    add_config = np.hstack((add_rpy, add_trans_T[0:3, 3].T))
    return ee_trans_T, add_trans_T, ee_rpy, add_rpy, ee_config, add_config


def dist_ext1(x, y):
    nx, p = x.shape
    x_ext = np.empty((nx, 3 * p))
    x_ext[:, :p] = 1
    x_ext[:, p:2 * p] = x
    x_ext[:, 2 * p:] = np.square(x)

    ny = y.shape[0]
    y_ext = np.empty((3 * p, ny))
    y_ext[:p] = np.square(y).T
    y_ext[p:2 * p] = -2 * y.T
    y_ext[2 * p:] = 1

    return x_ext.dot(y_ext)


def format_printing(display, signal=0):
    # pass
    if not signal:
        print('{:20s} {:40s}  {:20s}'.format('--------------------', display, '--------------------'))


def configuration_for_franka(T):
    dc = DefaultConfigurations()
    T_ee = np.linalg.multi_dot(
        [T, dc.franka_irb_pure_rotation_T, inverse_homo(dc.peg_T), inverse_homo(dc.peg_supporter_T)])
    T_support = np.linalg.multi_dot([T, dc.franka_irb_pure_rotation_T, inverse_homo(dc.peg_T)])
    support_config = matrix_to_config(T_support)
    ee_config = matrix_to_config(T_ee)
    return T_support, T_ee, support_config, ee_config


def moving_to_config(T, guidance_T):
    target_T = np.matmul(T, guidance_T)
    target_config = matrix_to_config(target_T)
    return target_config


def rt_sphere_cloud(T, objects):
    sphere_xyz = np.matmul(T[0:3, 0:3], objects) + T[0:3, 3].reshape((3, 1))
    return sphere_xyz


def absolute_error(T):
    alpha = np.arccos(T[0, 0])
    beta = np.arccos(T[1, 1])
    theta = np.arccos(T[2, 2])
    return alpha, beta, theta


def sphere_cloud():
    peg_st = SphereData('peg')
    hole_st = SphereData('hole')
    return peg_st, hole_st


class SphereData:
    def __init__(self, structure):
        # display = 'Generating ' + structure + ' sphere tree'
        # format_printing(display, signal=0)
        # os.chdir("..")
        dir_path = os.path.dirname(os.path.realpath(__file__))
        # print dir_path
        data_path = os.path.dirname(dir_path) + '/sphere_generation/sphere_cloud'
        # data_path = os.getcwd() + '/sphere_generation/sphere_cloud'
        coord = []
        with open(data_path + '/' + structure + '_spheres.txt', 'r') as f:
            for line in f:
                temp = [float(num) for num in line.split(',')]
                coord.append(temp)

        self.coord = np.asarray(coord)

        with open(data_path + '/' + structure + '_radius.txt', 'r') as f:
            for line in f:
                r = line
        self.radius = float(r)
        # os.chdir('sphere_cloud')
        self.min_x, self.max_x, self.min_z, self.max_z = self.min_max()

    def min_max(self):
        min_x = np.amin(self.coord[0, :])
        max_x = np.amax(self.coord[0, :])
        min_z = np.amin(self.coord[2, :])
        max_z = np.amax(self.coord[2, :])
        return min_x, max_x, min_z, max_z


# Gazebo Link7 - not include 0.107
# Moveit - include 0.107 (ee-pose)
class DefaultConfigurations:
    def __init__(self):
        # IRB initial pose
        self.irb_q_init = np.array([0, 0, 0, 0, 0, 0], dtype=np.float64)

        # IRB pose for replace the hole structure
        self.irb_q_replace = np.array([np.pi / 5, np.pi / 2, -np.pi / 2, 0, -np.pi / 2, 0])

        # IRB testing configuration
        self.irb_q_test = np.array([np.pi / 9, np.pi / 2, -np.pi / 2, 0, -np.pi / 2 + 0.1, 0.2])
        self.irb_q_test = np.array([0, 0, 0, 0, 0, 0])
        # calibration for force sensor
        self.irb_q_calibration = np.array([0, np.pi / 2, -np.pi / 2, 0, -np.pi / 2, 0])

        # Franka Emika Initial(Home)
        self.franka_q_init = np.array([0, -0.785, 0, -2.356, 0, 1.57, np.pi / 2])

        # Franka Panda ready to insert pose
        # self.franka_q_ready = np.array([0, 0, 0, -1.2, 0, 1.57, np.pi / 2])
        self.franka_q_ready = np.array([0, 0, 0, -np.pi / 2, 0, np.pi / 2, 0])
        self.franka_q_initial_force = np.array([0, 0.4175267223349789, 0,
                                                -1.9064060555173636, 0, 2.3249112667976757, np.pi / 2])

        self.franka_q_calibration = np.array([0.3060930632653474, -0.19765961084627645, -0.2706508966011701,
                                              -1.8107227377473263, -0.05259102333254284, 1.6191248450936613,
                                              1.6139649975360377])

        # IRB configuration
        self.irb_ready_config = np.array([0, np.pi / 2, -np.pi / 4, 0.54, 0, 0.3])
        self.irb_true_config = np.array([0, np.pi / 2, -np.pi / 4, 0.54, 0, 0.206])
        self.irb_test_config = np.array([0, np.pi / 2, -np.pi / 4, 0.54, 0, 0.19])

        # Franka Panda Screw Axis
        self.franka_S = np.array([[0, 0, 0, 0, 0, 0, 0],
                                  [0, 1, 0, -1, 0, -1, 0],
                                  [1, 0, 1, 0, 1, 0, -1],
                                  [0, -0.333, 0, 0.649, 0, 1.033, 0],
                                  [0, 0, 0, 0, 0, 0, 0.088],
                                  [0, 0, 0, -0.0825, 0, 0, 0]])
        # Franka Panda Home Matrix
        self.franka_M = np.array([[1, 0, 0, 0.088],
                                  [0, -1, 0, 0],
                                  [0, 0, -1, 0.926],
                                  [0, 0, 0, 1]])

        # IRB Screw Axis
        self.irb_S = np.array([[0, 0, 0, 1, 0, 1],
                               [0, 1, 1, 0, 1, 0],
                               [1, 0, 0, 0, 0, 0],
                               [0, -0.29, -0.56, 0, -0.63, 0],
                               [0, 0, 0, 0.63, 0, 0.63],
                               [0, 0, 0, 0, 0.302, 0]
                               ])
        self.irb_M = np.array([[0, 0, 1, 0.374],
                               [0, 1, 0, 0],
                               [-1, 0, 0, 0.63],
                               [0, 0, 0, 1]
                               ])

        self.G = np.diag([1, 1, 1, 1, 1, 1])

        # The dimension of Franka Panda and IRB supporter

        # The height of the force sensor and the supporter of the peg
        self.peg_supporter_h = 0.0385

        # The height of black part of IRB and the supporter of the hole. 0.0985-0.055
        self.hole_supporter_h = 0.0435

        self.peg_supporter_T = homo([0, 0, np.pi, 0, 0, self.peg_supporter_h])
        self.hole_supporter_T = homo([0, 0, np.pi / 2, 0, 0, self.hole_supporter_h])
        # 0.0715
        # The height of Peg and Hole structure
        self.hole_h = 0.055
        self.peg_h = 0.05

        # The workspace transformation matrix between Franka Panda and IRB
        # adjust this for the real robot relative pose. However, the pose shown in simulated environment keeps the same
        # For the 4-peg
        self.shared_ws = homo([0, 0, np.pi, 1.2, 0, -0.4195])
        self.shared_ws_demo = homo([0, 0, np.pi, 1.166, -0.002, -0.0002])

        self.franka_irb_pure_rotation_T = homo([np.pi, 0, np.pi, 0, 0, 0])

        # Define the haptic move, insertion depth
        self.hole_T = homo([0, 0, 0, 0, 0, self.hole_h])
        self.peg_T = homo([0, 0, 0, 0, 0, self.peg_h])

        self.peg_structure = homo([0, 0, 0, 0, 0, self.peg_h + self.peg_supporter_h])

        self.irb_peg_connector_h = 0.123
        self.hole_height_wrt_irb = 0.206 - self.irb_peg_connector_h
        self.irb_peg_connector_T = homo([0, 0, 0, 0, 0, self.irb_peg_connector_h])

        # TODO: modify
        self.franka_peg_connector_h = 0.118
        self.hole_height_wrt_franka = 0.199 - self.franka_peg_connector_h
        self.franka_peg_connector_T = homo([0, 0, 0, 0, 0, self.franka_peg_connector_h])
        # TODO: modify

        self.haptic_step = 0.01
        # self.insert_step = 0.001
        self.insert_step = 0.001
        self.haptic_movement_T = homo([0, 0, 0, 0, 0, self.haptic_step])
        self.insert_movement_T = homo([0, 0, 0, 0, 0, self.insert_step])
        self.insert_demo_movement_T = homo([0, 0, 0, 0, 0, 0.03])

        self.force_movement_T = homo([0, 0, 0, 0, 0, 0.003])
        self.attempt_T = homo([0, 0, -0.002, 0, 0, 0.00])
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        self.save_path = os.path.dirname(self.dir_path) + '/rt_data'
        print()
        # self.uncertainty_range = range(-5, 6)
        self.temp_uncertainty_range1 = range(9, 16)
        self.temp_uncertainty_range2 = range(-15, 16)
        self.temp_uncertainty_range3 = range(-15, 16)

        self.hole_to_irb_T = homo([0, np.pi / 2, -np.pi / 4, 0, 0, 0])

        self.hole_to_franka_T = homo([np.pi, 0, -np.pi / 2, 0, 0, 0])


class Demo:
    def __init__(self, q4_error=0, q5_error=0, q6_error=0):
        self.q4_error = q4_error
        self.q5_error = q5_error
        self.q6_error = q6_error

        self.dc = DefaultConfigurations()
        self.franka_ready_q = np.array(
            [-0.0021836689899237542, 0.33445115417622684, -0.0003633938274354461, -0.8477895065525121,
             0.0008681121661105953, 1.1837766126673062, 1.5666394351257218])
        # IRB pose with uncertainty
        self.irb_q = np.array([0, np.pi / 2, -np.pi / 2, 0 - q4_error, -np.pi / 2 - q5_error, 0 - q6_error])
        self.irb_q_calibration = np.array([0, np.pi / 2, -np.pi / 2, 0, -np.pi / 2, 0])

        self.irb_ee_T = np.matmul(self.dc.shared_ws_demo, fkine(self.dc.irb_S, self.dc.irb_M, self.irb_q))
        self.irb_nominal_ee_T = np.matmul(self.dc.shared_ws_demo,
                                          fkine(self.dc.irb_S, self.dc.irb_M, self.irb_q_calibration))

        self.irb_supporter_surface_T = np.matmul(self.irb_ee_T, self.dc.hole_supporter_T)
        self.irb_supporter_nominal_surface_T = np.matmul(self.irb_nominal_ee_T, self.dc.hole_supporter_T)

        self.hole_surface_T = np.matmul(self.irb_supporter_surface_T, self.dc.hole_T)

        # Provide xyz information to the Franka Panda
        self.hole_surface_nominal_T = np.hstack((np.linalg.multi_dot(
            [self.irb_supporter_nominal_surface_T, self.dc.hole_T])[:, 0:3], self.hole_surface_T[:, 3].reshape(4, 1)))

        # _, _, _, self.franka_ee_ready_config_demo = configuration_for_franka(
        #     self.irb_supporter_nominal_surface_T_demo)
        self.franka_support_nominal_T, self.franka_ee_nominal_T, self.franka_support_nominal_config, \
        self.franka_ee_nominal_config = configuration_for_franka(self.hole_surface_nominal_T)

        self.franka_support_correct_T, self.franka_ee_correct_T, _, self.franka_ee_correct_config = \
            configuration_for_franka(self.hole_surface_T)

        # self.hole_surface_nominal_T_demo = np.hstack((np.linalg.multi_dot(
        #     [self.irb_supporter_nominal_surface_T_demo, self.dc.hole_T])[:, 0:3],
        #                                               self.hole_surface_T_demo[:, 3].reshape(4, 1)))
        #
        # self.franka_support_nominal_T_demo, self.franka_ee_nominal_T_demo, self.franka_support_nominal_config_demo, \
        # self.franka_ee_nominal_config_demo = configuration_for_franka(self.hole_surface_nominal_T_demo)
        #


class Kinematics:
    def __init__(self, task='exec', q4_error=0, q5_error=0, q6_error=0):
        # Unit: meter
        # Franka Panda Home frame is the World frame
        self.q4_error = q4_error
        self.q5_error = q5_error
        self.q6_error = q6_error

        self.dc = DefaultConfigurations()
        self.peg_st, self.hole_st = sphere_cloud()

        # IRB pose with uncertainty
        self.irb_q_true = np.array([0, np.pi / 2, -np.pi / 2, 0 - q4_error, -np.pi / 2 - q5_error, 0 - q6_error])

        if task == 'calibration':
            self.irb_q = self.dc.irb_q_calibration
        elif task == 'test':
            self.irb_q = self.dc.irb_q_test
        elif task == 'exec' or task == 'collect':
            self.irb_q = self.irb_q_true

        # VERY IMPORTANT!!!!!
        # The configuration given to the Franka Panda is the top of the force sensor/the end-effector+ a sensor supporter

        # Irb end-effector transformation matrix express in the World Frame (not include the black part)
        self.irb_ee_T = np.matmul(self.dc.shared_ws, fkine(self.dc.irb_S, self.dc.irb_M, self.irb_q))
        self.irb_nominal_ee_T = np.matmul(self.dc.shared_ws,
                                          fkine(self.dc.irb_S, self.dc.irb_M, self.dc.irb_q_calibration))

        # The real-time surface of IRB supporter pose
        self.irb_supporter_surface_T = np.matmul(self.irb_ee_T, self.dc.hole_supporter_T)
        self.irb_supporter_nominal_surface_T = np.matmul(self.irb_nominal_ee_T, self.dc.hole_supporter_T)
        _, _, _, self.franka_ee_ready_config = configuration_for_franka(
            np.matmul(self.irb_supporter_nominal_surface_T, homo([0, 0, 0, 0, 0, 0.18])))

        # The real-time hole points cloud
        self.hole_xyz = rt_sphere_cloud(self.irb_supporter_surface_T, self.hole_st.coord)
        # np.savetxt(self.dc.save_path + '/hole_rt.txt', self.hole_xyz, fmt='%s')
        # format_printing('generated hole sphere cloud')

        # The hole surface pose in the World frame
        self.hole_surface_T = np.linalg.multi_dot([self.irb_supporter_surface_T, self.dc.hole_T])

        # Provide xyz information to the Franka Panda
        self.hole_surface_nominal_T = np.hstack((np.linalg.multi_dot(
            [self.irb_supporter_nominal_surface_T, self.dc.hole_T])[:, 0:3], self.hole_surface_T[:, 3].reshape(4, 1)))
        self.hole_surface_collect_T = np.hstack((self.hole_surface_T[:, 0:3], np.linalg.multi_dot(
            [self.irb_supporter_nominal_surface_T, self.dc.hole_T])[:, 3].reshape(4, 1)))
        # print(self.hole_surface_T)
        # print(self.hole_surface_nominal_T)
        # print(self.hole_surface_collect_T)

        # End-effector pose of Franka panda to arrive the top of the hold structure
        self.franka_support_correct_T, self.franka_ee_correct_T, _, self.franka_ee_correct_config = \
            configuration_for_franka(self.hole_surface_T)

        self.relative_principle_T = np.eye(4)
        self.relative_target_T = np.matmul(self.relative_principle_T, self.dc.insert_movement_T)

        # format_printing('franka_support_correct_T')
        # print(self.franka_support_correct_T)
        # print(np.linalg.inv(self.franka_support_correct_T))
        # print(self.relative_principle_T)
        self.franka_support_nominal_T, self.franka_ee_nominal_T, self.franka_support_nominal_config, \
        self.franka_ee_nominal_config = configuration_for_franka(self.hole_surface_nominal_T)

        self.franka_support_collect_T, self.franka_ee_collect_T, self.franka_support_collect_config, \
        self.franka_ee_collect_config = configuration_for_franka(self.hole_surface_collect_T)
        # The 6-dimensional uncertainties
        # print(self.franka_ee_nominal_config)
        # print(self.franka_ee_collect_config)
        self.franka_ee_force_collect_config = moving_to_config(self.franka_ee_nominal_T, self.dc.force_movement_T)

        self.uncertainty = refine_rpy(self.franka_ee_correct_config - self.franka_ee_nominal_config)
        # format_printing('computed uncertainty: ' + str(np.round(self.uncertainty[0:3], 4)))
        # format_printing('computed uncertainty: ' + str(np.round(self.uncertainty[3:6] * 1000, 1)))

        # Correct Configuration for insert
        self.franka_ee_target_correct_config = moving_to_config(self.franka_ee_correct_T, self.dc.insert_movement_T)

    # Return the configuration for peg tip to arrive the corresponding pose in the IRB local frame


def fixed_hole_top_to_franka_ee(hole_top_T, dc):
    ee_T_based_on_world = np.matmul(hole_top_T, dc.franka_peg_connector_T)
    ee_T_based_on_franka = np.matmul(ee_T_based_on_world, dc.hole_to_franka_T)
    ee_T_config = matrix_to_config(ee_T_based_on_franka)
    return ee_T_based_on_franka, ee_T_config


def insert_config(T, dc):
    ee_insert_T = np.matmul(T, dc.insert_movement_T)
    return ee_insert_T, matrix_to_config(ee_insert_T)


def insert_demo_config(T, dc):
    ee_insert_T = np.matmul(T, dc.insert_demo_movement_T)
    return ee_insert_T, matrix_to_config(ee_insert_T)


class FixedFrankaKinematics:
    def __init__(self, uncertainty=None):
        if uncertainty is None:
            uncertainty = np.array([0, 0, 0])
        self.uncertainty = uncertainty
        self.roll, self.pitch, self.yaw = uncertainty

        self.dc = DefaultConfigurations()
        self.hole_top_T = homo([0, 0, 0, 0.628, 0, self.dc.hole_height_wrt_franka])
        self.hole_top_uncertainty_T = homo([self.roll, self.pitch, self.yaw, 0.628, 0, self.dc.hole_height_wrt_franka])
        self.ee_T_based_on_franka, self.ee_config = fixed_hole_top_to_franka_ee(self.hole_top_uncertainty_T, self.dc)
        _, ww = fixed_hole_top_to_franka_ee(self.hole_top_T, self.dc)
        print(self.ee_config)
        print(ww)


def fixed_hole_top_to_irb_ee(hole_top_T, dc):
    ee_T_based_on_world = np.matmul(hole_top_T, dc.irb_peg_connector_T)
    ee_T_based_on_irb = np.matmul(ee_T_based_on_world, dc.hole_to_irb_T)
    ee_T_config = matrix_to_config(ee_T_based_on_irb)
    return ee_T_based_on_irb, ee_T_config


class FixedIRBKinematics:
    def __init__(self, uncertainty=None):
        if uncertainty is None:
            uncertainty = np.array([0, 0, 0])
        self.uncertainty = uncertainty
        self.roll, self.pitch, self.yaw = uncertainty

        dc = DefaultConfigurations()
        self.hole_top_T = homo([0, 0, 0, 0.54, 0, dc.hole_height_wrt_irb])
        self.hole_top_uncertainty_T = homo([self.roll, self.pitch, self.yaw, 0.54, 0, dc.hole_height_wrt_irb])
        self.ee_T_based_on_irb, self.ee_config = fixed_hole_top_to_irb_ee(self.hole_top_uncertainty_T, dc)


class RT:
    def __init__(self, franka_ee_contact_T):
        defaults_config = DefaultConfigurations()
        self.franka_ee_contact_T = franka_ee_contact_T
        # self.franka_ee_contact_T = fkine(defaults_config.franka_S, defaults_config.franka_M, joints)
        self.franka_support_contact_T = np.matmul(self.franka_ee_contact_T, defaults_config.peg_supporter_T)
        self.franka_ee_contact_config = matrix_to_config(self.franka_ee_contact_T)
        self.franka_support_contact_config = matrix_to_config(self.franka_support_contact_T)
        # The real-time peg points cloud
        print(self.franka_support_contact_T)
        print(absolute_error(self.franka_support_contact_T))
        # self.peg_xyz = rt_sphere_cloud(self.franka_support_contact_T, defaults_config.peg_st.coord)
        # np.savetxt(defaults_config.save_path + '/peg_rt.txt', self.peg_xyz, fmt='%s')
        # format_printing('generated peg sphere cloud')

        # np.savetxt('peg_rt.txt', self.peg_xyz, fmt='%s')

        self.franka_ee_haptic_config = moving_to_config(self.franka_ee_contact_T, defaults_config.haptic_movement_T)
        self.franka_support_haptic_config = moving_to_config(self.franka_support_contact_T,
                                                             defaults_config.haptic_movement_T)
