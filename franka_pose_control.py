#!/usr/bin/env python3
import numpy as np
import time
from load_model import *
from pose_control import *
import set_panda_defaults
from std_msgs.msg import String, Float32
from force_access import force_collect, error_record, read_force, force_monitor
import clear_bias
from kinematics import insert_config

Tz = 0


def tz_force_callback(data):
    global Tz
    Tz = abs(data.data)


def run_shutdown(panda_move, dc):
    set_panda_defaults.error_recovery()
    panda_move.go_to_joint_state(dc.franka_q_ready)


#    panda_move.go_to_joint_state(dc.franka_q_init)


def run_restore(panda_move, dc):
    set_panda_defaults.error_recovery()
    panda_move.go_to_joint_state(dc.franka_q_initial_force)
    clear_bias.main('http://airlab:airlab@192.168.125.185/rundata.htm')


def run_test(panda_move, dc, uncertainty):
    set_panda_defaults.error_recovery()
    scene = FixedFrankaKinematics([0.14, -0.02, -0.05])
    panda_move.go_to_franka_joints_ik_solver(scene.ee_config, uncertainty=scene.uncertainty, dc=scene.dc)


def insert_forward(panda_move, scene, threshold):
    configs = []
    current_T, current_config = scene.ee_T_based_on_franka, scene.ee_config
    panda_move.go_to_franka_joints_ik_solver(current_config, uncertainty=scene.uncertainty, dc=scene.dc)
    configs.append(current_config)
    print(Tz)
    while Tz < threshold and current_config[5] > 0.17:
        current_T, current_config = insert_config(current_T, scene.dc)
        configs.append(current_config)
        panda_move.go_to_franka_joints_ik_solver(current_config, uncertainty=scene.uncertainty, dc=scene.dc)
    return current_config


def insert_demo(panda_move, scene):
    configs = []
    current_T, current_config = scene.ee_T_based_on_franka, scene.ee_config
    panda_move.go_to_franka_joints_ik_solver(current_config, uncertainty=scene.uncertainty, dc=scene.dc)
    configs.append(current_config)
    print(Tz)
    # while Tz < threshold and current_config[5] > 0.17:
    current_T, current_config = insert_demo_config(current_T, scene.dc)
    configs.append(current_config)
    panda_move.go_to_franka_joints_ik_solver(current_config, uncertainty=scene.uncertainty, dc=scene.dc)
    return current_config


def insert_backward(panda_move, current_config):
    scene = FixedFrankaKinematics()
    restore_config = np.array([np.pi, 0, -np.pi / 2, 0.628, 0, current_config[-1] + 0.002])
    panda_move.go_to_franka_joints_ik_solver(restore_config)
    panda_move.go_to_franka_joints_ik_solver(scene.ee_config, uncertainty=scene.uncertainty, dc=scene.dc)

def run_collect(panda_move, dc):
    for i in dc.temp_uncertainty_range1:
        roll_error = i / 100
        for j in dc.temp_uncertainty_range2:
            pitch_error = j / 100
            for k in dc.temp_uncertainty_range3:
                yaw_error = k / 100

                uncertainty = np.array([roll_error, pitch_error, yaw_error])
                scene = FixedFrankaKinematics(uncertainty)
                print(datetime.now(), roll_error, pitch_error, yaw_error)
                configs = insert_forward(panda_move, scene)
                # panda_move.go_to_joints_ik_solver(scene.ee_config, uncertainty, dc)
                time.sleep(1)
                force_collect(dc, 'franka_full_force_data.csv', roll_error, pitch_error, yaw_error)
                time.sleep(0.5)
                insert_backward(panda_move, configs)
                run_restore(panda_move, dc)
    run_shutdown(panda_move, dc)


def run_demo(panda_move, dc, uncertainty):
    scene = FixedFrankaKinematics(uncertainty)
    print('uncertainty is', uncertainty)

    # Ready Franka Emika Pose
    run_restore(panda_move, dc)

    # Execute the trajectory to the nominal pose (got blocked)
    panda_move.go_to_franka_joints_ik_solver(scene.ee_config)
    insert_forward(panda_move, scene, 7)
    set_panda_defaults.error_recovery()
    time.sleep(0.5)

    measured_force = read_force()
    normalized_fm = preprocessing.normalize(np.asarray(measured_force[0:3]).reshape((1, -1)))
    normalized_tm = preprocessing.normalize(np.asarray(measured_force[3:6]).reshape((1, -1)))
    normalized_force = np.hstack((normalized_fm, normalized_tm))

    print('The normalized Fm is', normalized_force)
    zone_info = direction_info(normalized_fm)

    print('Zone Info is', zone_info)
    prediction = uncertainty_pred(zone_info, normalized_force)
    print('Prediction is', prediction)

    # Demo
    prediction = uncertainty
    # print('Prediction is', prediction)

    offset = np.abs(np.subtract(np.asarray(uncertainty), np.asarray(prediction))).tolist()

    scene_true = FixedFrankaKinematics(offset)
    panda_move.go_to_franka_joints_ik_solver(scene_true.ee_config)

    # Demo
    current_config = insert_demo(panda_move, scene_true)

    time.sleep(1)

    # current_config = insert_forward(panda_move, scene_true, 15)
    insert_backward(panda_move, current_config)

    run_restore(panda_move, dc)


def run_exec(panda_move, dc, uncertainty):
    scene = FixedFrankaKinematics(uncertainty)
    print('uncertainty is', uncertainty)

    # Ready Franka Emika Pose
    run_restore(panda_move, dc)

    # Execute the trajectory to the nominal pose (got blocked)
    panda_move.go_to_franka_joints_ik_solver(scene.ee_config)
    insert_forward(panda_move, scene, 7)
    set_panda_defaults.error_recovery()
    time.sleep(0.5)

    measured_force = read_force()
    normalized_fm = preprocessing.normalize(np.asarray(measured_force[0:3]).reshape((1, -1)))
    normalized_tm = preprocessing.normalize(np.asarray(measured_force[3:6]).reshape((1, -1)))
    normalized_force = np.hstack((normalized_fm, normalized_tm))

    print('The normalized Fm is', normalized_force)
    zone_info = direction_info(normalized_fm)

    print('Zone Info is', zone_info)
    prediction = uncertainty_pred(zone_info, normalized_force)
    print('Prediction is', prediction)

    # print('Prediction is', prediction)

    offset = np.abs(np.subtract(np.asarray(uncertainty), np.asarray(prediction))).tolist()

    scene_true = FixedFrankaKinematics(offset)
    panda_move.go_to_franka_joints_ik_solver(scene_true.ee_config)

    current_config = insert_forward(panda_move, scene_true, 15)
    insert_backward(panda_move, current_config)

    run_restore(panda_move, dc)


def my_hook():
    print("shutdown time!")


def create_uncertainty():
    uncertainty = np.random.randint(-50, 50, size=3) / 1000
    return uncertainty


def main(panda_move, dc, task, collect_force=False):
    try:
        if Tz:
            print('Force data loaded!')
        uncertainty = create_uncertainty()
        # uncertainty = [0.02, -0.01, -0.03]
        uncertainty = [0.05, -0.02, -0.03]
        # q4_error, q5_error, q6_error = create_uncertainty()
        pub = rospy.Publisher('force_signal', String, queue_size=10)
        set_panda_defaults.error_recovery()
        run_restore(panda_move, dc)

        if collect_force:
            time.sleep(1)
            force_signal = "start"
            pub.publish(force_signal)

        if task in {'calibration', 'restore'}:
            run_restore(panda_move, dc)

        elif task == 'test':
            run_test(panda_move, dc, uncertainty)
        elif task == 'collect':
            run_collect(panda_move, dc)
        elif task == 'exec':
            run_exec(panda_move, dc, uncertainty)
        elif task == 'shutdown':
            run_shutdown(panda_move, dc)
        elif task == 'demo':
            run_exec(panda_move, dc, uncertainty)

        if collect_force:
            force_signal = "end"
            pub.publish(force_signal)
        rospy.on_shutdown(my_hook)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    rospy.init_node("franka_haptic_control", log_level=rospy.FATAL)
    rospy.Subscriber('Tz_force', Float32, tz_force_callback)

    rate = rospy.Rate(1000)
    assert args.task in {'restore', 'shutdown', 'exec', 'calibration', 'test', 'collect'}
    format_printing("Start")
    panda_arm = RobotControl("panda_arm")
    set_panda_defaults.error_recovery()
    set_panda_defaults.set_defaults()
    defaults_config = DefaultConfigurations()
    # force_monitor()
    main(panda_arm, defaults_config, args.task, args.force)
    # rospy.spin()
