#!/usr/bin/env python3

"""
NOTE (YL) This VelocityController is not working,
mainly serves as a backup from the original code.

This is moved here from the original widowx_controller.py file, to
make the code more readable and cleaner.

CONVERTED TO ROS2 - but retains original issues and incomplete parts
"""

from __future__ import print_function
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import time
import pickle as pkl
from pyquaternion import Quaternion

try:
    # older version of interbotix sdk
    from interbotix_xs_sdk.msg import JointGroupCommand
except:
    # newer version of interbotix sdk
    from interbotix_xs_msgs.msg import JointGroupCommand

from widowx_controller.widowx_controller import WidowX_Controller

# TODO: This package needs to be converted to ROS2 or replaced
# from visual_mpc.envs.util.teleop.server import SpaceMouseRemoteReader

from widowx_envs.utils.exceptions import Environment_Exception

from modern_robotics.core import (
    JacobianSpace,
    Adjoint,
    MatrixLog6,
    se3ToVec,
    TransInv,
    FKinSpace,
)

##############################################################################


def compute_joint_velocities_from_cartesian(Slist, M, T, thetalist_current):
    """Computes inverse kinematics in the space frame for an open chain robot

    :param Slist: The joint screw axes in the space frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param M: The home configuration of the end-effector
    :param T: The desired end-effector configuration Tsd
    :param thetalist_current: An initial guess of joint angles that are close to
                       satisfying Tsd
    """
    thetalist = np.array(thetalist_current).copy()
    Tsb = FKinSpace(M, Slist, thetalist)
    Vs = np.dot(Adjoint(Tsb), se3ToVec(MatrixLog6(np.dot(TransInv(Tsb), T))))
    theta_vel = np.dot(np.linalg.pinv(JacobianSpace(Slist, thetalist)), Vs)
    return theta_vel


##############################################################################


class WidowXVelocityController(WidowX_Controller):
    def __init__(self, *args, **kwargs):
        super(WidowXVelocityController, self).__init__(*args, **kwargs)
        self.bot.dxl.robot_set_operating_modes("group", "arm", "velocity")
        self.bot.arm.set_trajectory_time(moving_time=0.2, accel_time=0.05)

        # TODO: SpaceMouseRemoteReader needs to be converted to ROS2 or replaced
        # TODO(YL): WARNING: This package is not available, fix this
        # self.space_mouse = SpaceMouseRemoteReader()
        self.space_mouse = None  # Placeholder until converted

        # Create timer for robot command updates (ROS2 style)
        self._robot_cmd_timer = self.create_timer(0.02, self.update_robot_cmds)

        self.last_update_cmd = time.time()
        self.enable_cmd_thread = False
        self.do_reset = False
        self.task_stage = 0
        self.num_task_stages = int(1e9)

        # Add missing attribute from original code
        self._last_healthy_tstamp = self.get_clock().now().seconds_nanoseconds()[0]

    def update_robot_cmds(self):
        """ROS2 timer callback - no event parameter needed"""
        if self.space_mouse is None:
            return

        reading = self.space_mouse.get_reading()
        if reading is not None and self.enable_cmd_thread:
            self.last_update_cmd = time.time()
            if reading["left"] and reading["right"] or reading["left_and_right"]:
                self.task_stage += 1
                self.task_stage = np.clip(self.task_stage, 0, self.num_task_stages)
                if self.task_stage == self.num_task_stages:
                    self.get_logger().info("resetting!")
                    self.do_reset = True
                time.sleep(1.0)  # ROS2: use time.sleep instead of rospy.sleep
            self.apply_spacemouse_action(reading)

    def apply_spacemouse_action(self, readings):
        if readings is None:
            self.get_logger().warn("readings are None!")
            return

        if self.custom_gripper_controller:
            if readings["left"]:
                self._gripper.open()
            if readings["right"]:
                self._gripper.close()
        else:
            if readings["left"]:
                self.bot.gripper.open()
            if readings["right"]:
                self.bot.gripper.close()

        if self.enable_rotation:
            pose = self.get_cartesian_pose(matrix=True)

            current_quat = Quaternion(matrix=pose[:3, :3])
            translation_scale = 0.1
            commanded_translation_velocity = readings["xyz"] * translation_scale
            new_pos = pose[:3, 3] + commanded_translation_velocity

            rotation_scale = 0.4
            commanded_rotation_velocity = readings["rot"] * rotation_scale
            if self.enable_rotation == "4dof":
                commanded_rotation_velocity = commanded_rotation_velocity[2]
                new_rot = (
                    Quaternion(axis=[0, 0, 1], angle=commanded_rotation_velocity)
                    * current_quat
                )
            elif self.enable_rotation == "6dof":
                new_rot = (
                    Quaternion(axis=[1, 0, 0], angle=commanded_rotation_velocity[0])
                    * Quaternion(axis=[0, 1, 0], angle=commanded_rotation_velocity[1])
                    * Quaternion(axis=[0, 0, 1], angle=commanded_rotation_velocity[2])
                    * current_quat
                )
            else:
                raise NotImplementedError

            new_transform = np.eye(4)
            new_transform[:3, :3] = new_rot.rotation_matrix
            new_transform[:3, 3] = new_pos
        else:
            new_transform = self.get_cartesian_pose(matrix=True)
            new_transform[:3, 3] += readings["xyz"] * 0.1

        joint_velocities = compute_joint_velocities_from_cartesian(
            self.bot.arm.robot_des.Slist,
            self.bot.arm.robot_des.M,
            new_transform,
            self.get_joint_angles(),
        )
        self.cap_joint_limits(joint_velocities)
        try:
            # TODO(YL): Fixed - JointCommands -> JointGroupCommand
            # Note: The original code had an undefined JointCommands class
            joint_commands = JointGroupCommand()
            joint_commands.name = self.bot.arm.group_name
            joint_commands.cmd = joint_velocities.tolist()
            self.bot.dxl.pub_group.publish(joint_commands)
        except Exception as e:
            self.get_logger().error(f"could not set joint velocity! Error: {e}")

    def stop_motors(self):
        """Stop all motors by sending zero velocity commands"""
        try:
            # Send zero velocities
            joint_commands = JointGroupCommand()
            joint_commands.name = self.bot.arm.group_name
            joint_commands.cmd = [0.0] * self._qn
            self.bot.dxl.pub_group.publish(joint_commands)
        except Exception as e:
            self.get_logger().error(f"could not stop motors! Error: {e}")

    def move_to_state(self, target_xyz, target_zangle, duration=2):
        pose = np.eye(4)
        rot = Quaternion(axis=[0, 0, 1], angle=target_zangle) * Quaternion(
            matrix=self.default_rot
        )
        pose[:3, :3] = rot.rotation_matrix
        pose[:3, 3] = target_xyz

        joint_pos, success = self.bot.arm.set_ee_pose_matrix(
            pose, custom_guess=self.get_joint_angles(), moving_time=2, execute=False
        )
        if success:
            self.move_to_pos_with_velocity_ctrl(joint_pos)
            return True
        else:
            self.get_logger().error("no kinematics solution found!")
            raise Environment_Exception

    def cap_joint_limits(self, ctrl):
        """Cap joint velocities to respect joint limits"""
        current_angles = self.get_joint_angles()
        if current_angles is None:
            return

        for i in range(self._qn):
            if current_angles[i] < self._lower_joint_limits[i]:
                self.get_logger().warn(f"lower joint angle limit violated for j{i + 1}")
                if ctrl[i] < 0:
                    self.get_logger().warn("setting velocity to zero")
                    ctrl[i] = 0
            if current_angles[i] > self._upper_joint_limits[i]:
                self.get_logger().warn(f"upper joint angle limit violated for j{i + 1}")
                if ctrl[i] > 0:
                    self.get_logger().warn("setting velocity to zero")
                    ctrl[i] = 0

    def move_to_neutral(self, duration=4):
        self.move_to_pos_with_velocity_ctrl(
            self.neutral_joint_angles, duration=duration
        )

    def move_to_pos_with_velocity_ctrl(self, des, duration=3):
        """Move to desired position using velocity control"""
        nsteps = 30
        per_step = float(duration) / nsteps

        tstart = time.time()
        current = self.get_joint_angles()
        if current is None:
            self.get_logger().error("Cannot get current joint angles")
            return

        error = des - current
        while (time.time() - tstart) < duration and np.linalg.norm(error) > 0.15:
            current = self.get_joint_angles()
            if current is None:
                break

            error = des - current
            ctrl = error * 0.8
            max_speed = 0.5
            ctrl = np.clip(ctrl, -max_speed, max_speed)
            self.cap_joint_limits(ctrl)

            try:
                joint_commands = JointGroupCommand()
                joint_commands.name = self.bot.arm.group_name
                joint_commands.cmd = ctrl.tolist()
                self.bot.dxl.pub_group.publish(joint_commands)
                self._last_healthy_tstamp = (
                    self.get_clock().now().seconds_nanoseconds()[0]
                )
            except Exception as e:
                self.get_logger().error(f"Error publishing joint commands: {e}")

            time.sleep(per_step)

        # Stop motors at the end
        try:
            joint_commands = JointGroupCommand()
            joint_commands.name = self.bot.arm.group_name
            joint_commands.cmd = [0.0] * self._qn
            self.bot.dxl.pub_group.publish(joint_commands)
        except Exception as e:
            self.get_logger().error(f"Error stopping motors: {e}")

    def apply_endeffector_velocity(self, action):
        """Apply end effector velocity command (missing from original but used in main)"""
        current_pose = self.get_cartesian_pose(matrix=True)

        # Apply translation
        new_pos = current_pose[:3, 3] + action[:3]

        # Apply rotation (if action has rotation component)
        if len(action) > 3:
            current_quat = Quaternion(matrix=current_pose[:3, :3])
            if len(action) == 4:  # 4DOF case
                new_rot = Quaternion(axis=[0, 0, 1], angle=action[3]) * current_quat
            elif len(action) >= 6:  # 6DOF case
                new_rot = (
                    Quaternion(axis=[1, 0, 0], angle=action[3])
                    * Quaternion(axis=[0, 1, 0], angle=action[4])
                    * Quaternion(axis=[0, 0, 1], angle=action[5])
                    * current_quat
                )
        else:
            new_rot = Quaternion(matrix=current_pose[:3, :3])

        new_transform = np.eye(4)
        new_transform[:3, :3] = new_rot.rotation_matrix
        new_transform[:3, 3] = new_pos

        joint_velocities = compute_joint_velocities_from_cartesian(
            self.bot.arm.robot_des.Slist,
            self.bot.arm.robot_des.M,
            new_transform,
            self.get_joint_angles(),
        )
        self.cap_joint_limits(joint_velocities)

        try:
            joint_commands = JointGroupCommand()
            joint_commands.name = self.bot.arm.group_name
            joint_commands.cmd = joint_velocities.tolist()
            self.bot.dxl.pub_group.publish(joint_commands)
        except Exception as e:
            self.get_logger().error(
                f"could not apply end effector velocity! Error: {e}"
            )


##############################################################################


def main():
    """Main function for testing the velocity controller"""
    rclpy.init()

    try:
        # Test with saved trajectory data (as in original)
        dir_path = "/mount/harddrive/spt/trainingdata/realworld/can_pushing_line/2020-09-04_09-28-29/raw/traj_group0/traj2"

        try:
            dict_policy = pkl.load(open(dir_path + "/policy_out.pkl", "rb"))
            actions = np.stack([d["actions"] for d in dict_policy], axis=0)
            dict_obs = pkl.load(open(dir_path + "/obs_dict.pkl", "rb"))
            states = dict_obs["raw_state"]
        except FileNotFoundError:
            print(f"Test data not found at {dir_path}, creating dummy data")
            actions = np.random.rand(20, 4) * 0.1  # 20 timesteps, 4DOF actions
            states = np.random.rand(20, 4)  # 20 timesteps, xyz + angle

        controller = WidowXVelocityController("wx250s", True, {})

        executor = MultiThreadedExecutor()

        import threading

        def run_test():
            time.sleep(2)

            controller.move_to_neutral()
            controller.move_to_state(states[0, :3], target_zangle=0.0)

            prev_eef = controller.get_cartesian_pose()[:3]
            for t in range(min(20, len(actions))):
                controller.apply_endeffector_velocity(actions[t] / 0.2)

                new_eef = controller.get_cartesian_pose()[:3]
                print("current eef pos", new_eef[:3])
                print("desired eef pos", states[t, :3])
                print("delta", states[t, :3] - new_eef[:3])
                time.sleep(0.2)

        test_thread = threading.Thread(target=run_test)
        test_thread.start()

        # Spin the controller
        rclpy.spin(controller, executor)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        if "controller" in locals():
            controller.get_logger().error(f"Error in velocity controller: {e}")
    finally:
        # Clean up
        if "controller" in locals():
            controller.stop_motors()
            controller.destroy_node()

        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()
