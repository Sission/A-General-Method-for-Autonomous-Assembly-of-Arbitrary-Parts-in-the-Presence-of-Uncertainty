#!/usr/bin/env python3

from pose_control import *
from force_access import force_collect, error_record
import clear_bias


def run_shutdown(irb_move, dc):
    irb_move.go_to_joint_state(dc.irb_q_init)


def run_restore(irb_move, dc):
    irb_move.go_to_irb_joints_ik_solver(dc.irb_ready_config)
    time.sleep(1)


def run_replace(irb_move, dc):
    irb_move.go_to_joint_state(dc.irb_q_replace)


def run_test(irb_move, dc, uncertainty):
    # scene = Kinematics('collect')
    scene = Demo(0,0,0)
    irb_move.go_to_joint_state(scene.irb_q)
    # irb_move.go_to_irb_joints_ik_solver(dc.irb_test_config)

    # force_collect(dc, 'test.csv', -0.08, -0.1, -0.05)
    # print(uncertainty)
    # print(scene.ee_T_config)
    # irb_move.go_to_irb_joints_ik_solver(scene.ee_T_config)
    # run_restore(irb_move, dc)


def run_collect(irb_move, dc):
    for i in dc.uncertainty_range:
        roll_error = i / 100
        for j in dc.uncertainty_range:
            pitch_error = j / 100
            for k in dc.uncertainty_range:
                yaw_error = k / 100

                uncertainty = np.array([roll_error, pitch_error, yaw_error])
                scene = FixedIRBKinematics(uncertainty)
                print(datetime.now(), uncertainty)

                irb_move.go_to_irb_joints_ik_solver(scene.ee_config, uncertainty, dc)
                time.sleep(1.5)
                force_collect(dc, 'irb_force_data.csv', roll_error, pitch_error, yaw_error)
                time.sleep(1)
                run_restore(irb_move, dc)


def run_exec(irb_move, dc, uncertainty):
    scene = FixedIRBKinematics(uncertainty)
    # scene = Kinematics(task='exec', q4_error=q4_error, q5_error=q5_error, q6_error=q6_error)


def main(irb_move, dc, task):
    try:
        uncertainty = create_uncertainty()
        # irb_move.go_to_irb_joints_ik_solver(dc.irb_true_config)
        # run_restore(irb_move, dc)
        if task == 'shutdown':
            run_shutdown(irb_move, dc)
        elif task == 'test':
            run_test(irb_move, dc, uncertainty)

        elif task in {'calibration', 'restore'}:
            run_restore(irb_move, dc)
        elif task == 'exec':
            run_exec(irb_move, dc, uncertainty)
        elif task == 'replace':
            run_replace(irb_move, dc)
        elif task == 'collect':
            run_collect(irb_move, dc)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    rospy.init_node("irb_haptic_control", log_level=rospy.FATAL)
    rate = rospy.Rate(1000)
    assert args.task in {'restore', 'shutdown', 'exec', 'calibration', 'test', 'collect', 'replace'}
    format_printing("Start")
    irb_arm = RobotControl("manipulator")
    defaults_config = DefaultConfigurations()
    main(irb_arm, defaults_config, args.task)
