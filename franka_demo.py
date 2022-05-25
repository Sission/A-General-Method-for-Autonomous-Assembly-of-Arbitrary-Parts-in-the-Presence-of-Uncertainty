#!/usr/bin/env python3
import time

from pose_control import *
import set_panda_defaults
from std_msgs.msg import String, Float32
from force_access import force_collect, error_record
import clear_bias
from kinematics import insert_config

Tz = 0


def tz_force_callback(data):
    global Tz
    Tz = abs(data.data)


def run_shutdown(panda_move, dc):
    set_panda_defaults.error_recovery()
    panda_move.go_to_joint_state(dc.franka_q_ready)


def run_restore(panda_move, dc):
    set_panda_defaults.error_recovery()
    panda_move.go_to_joint_state(dc.franka_q_initial_force)
    clear_bias.main('http://airlab:airlab@192.168.125.185/rundata.htm')


def insert_forward(panda_move, dc, config, config_T):
    configs = []
    current_T, current_config = config_T, config
    panda_move.go_to_franka_joints_ik_solver(current_config)
    configs.append(current_config)

    for i in range(4):
        current_T, current_config = insert_config(current_T, dc)
        configs.append(current_config)
        panda_move.go_to_franka_joints_ik_solver(current_config)
    return configs[::-1]


def insert_backward(panda_move, configs):
    for i, value in enumerate(configs):
        panda_move.go_to_franka_joints_ik_solver(configs[i])


def run_test(panda_move, dc):
    set_panda_defaults.error_recovery()
    scene = Demo(0.02, -0.1, 0.02)
    panda_move.go_to_franka_joints_ik_solver(scene.franka_ee_nominal_config)
    rt_data = RT(scene.franka_ee_nominal_T)
    # set_panda_defaults.error_recovery()
    #
    # # Move to the haptic configuration pose (got blocked)
    panda_move.go_to_franka_joints_ik_solver(rt_data.franka_ee_haptic_config)
    set_panda_defaults.error_recovery()
    time.sleep(5)
    panda_move.go_to_franka_joints_ik_solver(scene.franka_ee_correct_config)
    configs = insert_forward(panda_move, dc, scene.franka_ee_correct_config, scene.franka_ee_correct_T)
    insert_backward(panda_move, configs)
    panda_move.go_to_joint_state(scene.franka_ready_q)


def my_hook():
    print("shutdown time!")


def main(panda_move, dc, task, collect_force=False):
    try:
        if Tz:
            print('Tz data received!')
        # uncertainty = create_uncertainty()
        # q4_error, q5_error, q6_error = create_uncertainty()
        pub = rospy.Publisher('force_signal', String, queue_size=10)
        set_panda_defaults.error_recovery()
        # run_restore(panda_move, dc)

        if collect_force:
            time.sleep(1)
            force_signal = "start"
            pub.publish(force_signal)

        if task in {'calibration', 'restore'}:
            run_restore(panda_move, dc)

        elif task == 'test':
            run_test(panda_move, dc)
        elif task == 'shutdown':
            run_shutdown(panda_move, dc)

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
    main(panda_arm, defaults_config, args.task, args.force)
    # rospy.spin()
