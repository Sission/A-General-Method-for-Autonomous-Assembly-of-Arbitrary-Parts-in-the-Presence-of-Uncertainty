from cvxopt import matrix
from cvxopt import solvers
from kinematics import *

dir_path = os.path.dirname(os.path.realpath(__file__))
save_path = os.path.dirname(dir_path) + '/sphere_cloud'


def collision_detection_ccp(scene, rt_data):
    # type: (Kinematics, RT) -> tuple

    peg_cloud = np.transpose(rt_data.peg_xyz)
    hole_cloud = np.transpose(scene.hole_xyz)
    # np.savetxt('peg_rt_ccp.txt', rt_data.peg_xyz, fmt='%s')
    # np.savetxt('hole_rt_ccp.txt', scene.hole_xyz, fmt='%s')
    dist = dist_ext1(peg_cloud, hole_cloud)
    ccp_list = np.where(dist <= np.square(scene.dc.peg_st.radius + scene.dc.hole_st.radius + scene.dc.haptic_step))
    ccp_peg_index = ccp_list[0]
    ccp_hole_index = ccp_list[1]
    list_size = ccp_hole_index.size
    display = "CCP predict number of pairs: %d" % list_size
    format_printing(display)
    return ccp_peg_index, ccp_hole_index, list_size


# Construct constraint matrix
def matrix_diff_rpy(config):
    # type: (np.ndarray) -> (np.ndarray,np.ndarray)
    [roll, pitch, yaw, _, _, _] = config[:]
    df_yaw = 2 * np.einsum('ij,jk,kl', diff_yaw(yaw), rot_y(pitch), rot_x(roll))
    df_pitch = 2 * np.einsum('ij,jk,kl', rot_z(yaw), diff_pitch(pitch), rot_x(roll))
    df_roll = 2 * np.einsum('ij,jk,kl', rot_z(yaw), rot_y(pitch), diff_roll(roll))
    R = rpy(roll, pitch, yaw)
    diff_rpy = np.hstack([df_yaw[:], df_pitch[:], df_roll[:]])
    return diff_rpy, R


def matrix_construction(diff_rpy, config, R, sp, sh):
    # type: (np.ndarray,np.ndarray,np.ndarray,np.ndarray,np.ndarray) -> (np.ndarray,)
    A = (config[3:6] - sh[:]).reshape(1, 3)
    df_yaw = np.einsum('ij,jk,kl', A, diff_rpy[:, 0:3], sp.reshape(3, 1))
    df_pitch = np.einsum('ij,jk,kl', A, diff_rpy[:, 3:6], sp.reshape(3, 1))
    df_roll = np.einsum('ij,jk,kl', A, diff_rpy[:, 6:9], sp.reshape(3, 1))
    diff_xyz = 2 * (np.transpose(np.matmul(R, sp.reshape(3, 1))) + A)
    diff_matrix = np.hstack([df_roll[:], df_pitch[:], df_yaw[:], diff_xyz[:]])
    return diff_matrix


def optimization_matrix(scene, rt_data, ccp_peg_index, ccp_hole_index, ccp_list_size):
    # type: (Kinematics, RT, np.ndarray, np.ndarray,int) -> tuple

    peg_xyz_rt = np.matmul(rt_data.franka_support_contact_T[0:3, 0:3], scene.dc.peg_st.coord[:, ccp_peg_index]) + rt_data.franka_support_contact_T[0:3, 3].reshape((3, 1))
    hole_xyz_rt = scene.hole_xyz[:, ccp_hole_index]

    C_pre_config_distance = np.sum((peg_xyz_rt - hole_xyz_rt) ** 2, axis=0) - (scene.dc.peg_st.radius + scene.dc.hole_st.radius) ** 2
    diff_rpy, R = matrix_diff_rpy(rt_data.franka_support_contact_config)
    C_pre_config_differential = np.zeros((ccp_list_size, 6))
    for i in range(ccp_list_size):
        C_pre_config_differential[i, :] = matrix_construction(diff_rpy, rt_data.franka_support_contact_config,
                                                              R, scene.dc.peg_st.coord[:, ccp_peg_index[i]], scene.hole_xyz[:, ccp_hole_index[i]])
    G = C_pre_config_differential
    h = C_pre_config_distance.reshape((ccp_list_size, 1)) + np.matmul(C_pre_config_differential, (rt_data.franka_support_haptic_config - rt_data.franka_support_contact_config).reshape((6, 1)))
    return C_pre_config_differential, C_pre_config_distance, G, h


def qp_optimization(G, h, rt_data):
    # type: (np.ndarray, np.ndarray, RT) -> np.ndarray

    # P is the stiffness matrix
    P = matrix(np.diag(np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])), tc='d')
    q = matrix(np.zeros((6, 1)), tc='d')
    G = matrix(G, tc='d')
    h = matrix(h, tc='d')

    format_printing('Starting QP solver')
    solvers.options['show_progress'] = False
    solvers.options['maxiters'] = 20000
    solvers.options['abstol'] = 1e-10
    solvers.options['reltol'] = 1e-10
    solvers.options['feastol'] = 1e-10
    solvers.options['refinement'] = 1

    sol = solvers.qp(P, q, G, h)
    format_printing('Solution Found')
    # delta_q = q_haptic - q_graphic
    delta_q = np.asarray(sol['x']).reshape((6,))
    predicted_config = rt_data.franka_support_haptic_config - delta_q
    return predicted_config


def haptic_algorithm(scene, rt_data):
    # type: (Kinematics, RT) -> tuple

    #  The reason is that these points are the closest pairs. The remaining points are impossible in collision
    #  in the next time step within the given ccp_threshold
    ccp_peg_index, ccp_hole_index, list_size = collision_detection_ccp(scene, rt_data)
    _, _, G_matrix, h_matrix = optimization_matrix(scene, rt_data, ccp_peg_index, ccp_hole_index, list_size)
    franka_support_predicted_config = qp_optimization(G_matrix, h_matrix, rt_data)

    # Output for inspection
    new_peg_xyz = rt_sphere_cloud(homo(franka_support_predicted_config), scene.dc.peg_st.coord)
    old_peg_xyz = rt_sphere_cloud(homo(scene.franka_support_nominal_config), scene.dc.peg_st.coord)
    haptic_peg_xyz = rt_sphere_cloud(homo(rt_data.franka_support_haptic_config), scene.dc.peg_st.coord)

    # np.savetxt('new_peg_xyz_rt.txt', new_peg_xyz, fmt='%s')
    # np.savetxt('old_peg_xyz_rt.txt', old_peg_xyz, fmt='%s')
    # np.savetxt('haptic_peg_xyz_rt.txt', haptic_peg_xyz, fmt='%s')

    # np.savetxt(save_path + '/old_peg_xyz_rt.txt', new_peg_xyz, fmt='%s')

    computed_force = franka_support_predicted_config - rt_data.franka_support_contact_config
    return franka_support_predicted_config, computed_force


def uncertainty_prediction(scene, rt_data):
    pass
    # # type: (Kinematics, RT) -> tuple
    #
    franka_support_predicted_config, computed_force = haptic_algorithm(scene, rt_data)
    # idx = 0

    # # TODO: feed force to the FC model
    # Find the computed_force that match the measured force
    # force_idx = Model(measured force)
    # corresponding computed force = computed_force(force_idx)
    # TODO: find the configuration that fit the measured force fm

    # franka_predicted_ee_config = find_confi(# corresponding computed force)
    # franka_predicted_ee_target_config = moving_to_config(homo(franka_predicted_ee_config), scene.insert_movement_T)
    #
    # return franka_predicted_ee_config, franka_predicted_ee_target_config
