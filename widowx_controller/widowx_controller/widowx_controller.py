#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from pyquaternion import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from rclpy import ok, init
from rclpy.task import Future

from threading import Lock
import logging
import os
import time
import signal
import sys

import tf2_ros
from tf_transformations import quaternion_from_matrix

from interbotix_xs_modules.xs_robot.arm import (
    InterbotixArmXSInterface,
    InterbotixArmXSInterface,
    InterbotixRobotXSCore,
    InterbotixGripperXSInterface,
)
from interbotix_common_modules.common_robot import InterbotixRobotNode

try:
    # older version of interbotix sdk
    from interbotix_xs_sdk.msg import JointGroupCommand
except:
    # newer version of interbotix sdk
    from interbotix_xs_msgs.msg import JointGroupCommand
    from interbotix_xs_msgs.srv import Reboot

from widowx_envs.utils.exceptions import Environment_Exception
from modern_robotics.core import (
    JacobianSpace,
    Adjoint,
    MatrixLog6,
    se3ToVec,
    TransInv,
    FKinSpace,
)
import widowx_envs.utils.transformation_utils as tr

from widowx_controller.custom_gripper_controller import GripperController
from widowx_controller.controller_base import RobotControllerBase

import numpy as np
from numba import jit

##############################################################################

# NOTE: experimental values
ABS_MAX_JOINT_EFFORTS = np.array([800, 1000, 600.0, 600.0, 600.0, 700.0]) * 1.7
NEUTRAL_JOINT_STATE = np.array(
    [-0.13192235, -0.76238847, 0.44485444, -0.01994175, 1.7564081, -0.15953401]
)
DEFAULT_ROTATION = np.array([[0, 0, 1.0], [0, 1.0, 0], [-1.0, 0, 0]])

##############################################################################


@jit()
def ModifiedIKinSpace(Slist, M, T, thetalist0, eomg, ev, maxiterations=40):
    """
    ModifiedIKinSpace - Inverse Kinematics in the Space Frame
    this exposed the max_iterations parameter to the user

    # original source:
    https://github.com/NxRLab/ModernRobotics/blob/master/packages/Python/modern_robotics/core.py
    """
    thetalist = np.array(thetalist0).copy()
    i = 0
    Tsb = FKinSpace(M, Slist, thetalist)
    Vs = np.dot(Adjoint(Tsb), se3ToVec(MatrixLog6(np.dot(TransInv(Tsb), T))))
    err = (
        np.linalg.norm([Vs[0], Vs[1], Vs[2]]) > eomg
        or np.linalg.norm([Vs[3], Vs[4], Vs[5]]) > ev
    )
    while err and i < maxiterations:
        thetalist = thetalist + np.dot(
            np.linalg.pinv(JacobianSpace(Slist, thetalist)), Vs
        )
        i = i + 1
        Tsb = FKinSpace(M, Slist, thetalist)
        Vs = np.dot(Adjoint(Tsb), se3ToVec(MatrixLog6(np.dot(TransInv(Tsb), T))))
        err = (
            np.linalg.norm([Vs[0], Vs[1], Vs[2]]) > eomg
            or np.linalg.norm([Vs[3], Vs[4], Vs[5]]) > ev
        )
    if err:
        print("IKinSpace: did not converge")
        print("Vs", Vs)
    return (thetalist, not err)


##############################################################################


def publish_transform(node, transform, name, parent_name="wx250s/base_link"):
    """Publish transform using ROS2 tf2 broadcaster"""
    translation = transform[:3, 3]

    br = tf2_ros.TransformBroadcaster(node)
    t = TransformStamped()

    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = parent_name
    t.child_frame_id = name
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]

    quat = quaternion_from_matrix(transform)
    t.transform.rotation.w = quat[0]
    t.transform.rotation.x = quat[1]
    t.transform.rotation.y = quat[2]
    t.transform.rotation.z = quat[3]

    br.sendTransform(t)


class ModifiedInterbotixManipulatorXS(object):
    def __init__(
        self,
        robot_model,
        group_name="arm",
        gripper_name="gripper",
        robot_name=None,
        moving_time=2.0,
        accel_time=0.3,
        gripper_pressure=0.5,
        gripper_pressure_lower_limit=150,
        gripper_pressure_upper_limit=350,
        node=None,
    ):
        self.dxl = InterbotixRobotXSCore(robot_model, robot_name, node=node)
        self.arm = ModifiedInterbotixArmXSInterface(
            self.dxl, robot_model, group_name, moving_time, accel_time
        )
        if gripper_name is not None:
            self.gripper = InterbotixGripperXSInterface(
                self.dxl,
                gripper_name,
                gripper_pressure,
                gripper_pressure_lower_limit,
                gripper_pressure_upper_limit,
            )


class ModifiedInterbotixArmXSInterface(InterbotixArmXSInterface):
    def __init__(self, *args, **kwargs):
        super(ModifiedInterbotixArmXSInterface, self).__init__(*args, **kwargs)
        self.waist_index = self.group_info.joint_names.index("waist")

    def set_ee_pose_matrix_fast(self, T_sd, custom_guess=None, execute=True):
        """
        this version of set_ee_pose_matrix does not set the velocity profile registers in the servos and therefore runs faster
        """
        if custom_guess is None:
            initial_guesses = self.initial_guesses
        else:
            initial_guesses = [custom_guess]

        for guess in initial_guesses:
            theta_list, success = ModifiedIKinSpace(
                self.robot_des.Slist, self.robot_des.M, T_sd, guess, 0.001, 0.001
            )
            solution_found = True

            # Check to make sure a solution was found and that no joint limits were violated
            if success:
                theta_list = [int(elem * 1000) / 1000.0 for elem in theta_list]
                for x in range(self.group_info.num_joints):
                    if not (
                        self.group_info.joint_lower_limits[x]
                        <= theta_list[x]
                        <= self.group_info.joint_upper_limits[x]
                    ):
                        solution_found = False
                        break
            else:
                solution_found = False

            if solution_found:
                if execute:
                    self.publish_positions_fast(theta_list)
                    self.T_sb = T_sd
                return theta_list, True
            else:
                self.core.get_logger().info("Guess failed to converge...")

        self.core.get_logger().info("No valid pose could be found")
        return theta_list, False

    def publish_positions_fast(self, positions):
        self.joint_commands = list(positions)
        joint_commands = JointGroupCommand(
            group_name=self.group_name, cmd=self.joint_commands
        )
        self.core.pub_group.publish(joint_commands)
        self.T_sb = FKinSpace(
            self.robot_des.M, self.robot_des.Slist, self.joint_commands
        )


##############################################################################


class WidowX_Controller(RobotControllerBase, Node):
    def __init__(
        self,
        robot_name,
        print_debug,
        gripper_params,
        enable_rotation="6dof",
        gripper_attached="custom",
        normal_base_angle=0,
        node=None,
    ):
        """
        gripper_attached: either "custom" or "default"
        """
        if not ok():
            init()
        if node is None:
            self.interbotix_node = InterbotixRobotNode(robot_name)
        else:
            self.interbotix_node = node

        Node.__init__(self, 'widowx_controller')
        self.get_logger().info("waiting for widowx_controller to be set up...")
        self.bot = ModifiedInterbotixManipulatorXS(robot_model=robot_name, node=self.interbotix_node)

        if gripper_params is None:
            gripper_params = {}

        self._robot_name = robot_name

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        logger = logging.getLogger("robot_logger")
        formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
        )
        log_level = logging.WARN
        if print_debug:
            log_level = logging.DEBUG
        ch = logging.StreamHandler()
        ch.setLevel(log_level)
        ch.setFormatter(formatter)
        logger.addHandler(ch)
        self._init_gripper(gripper_attached, gripper_params)

        self._joint_lock = Lock()
        self._angles, self._velocities, self._effort = {}, {}, {}

        # Create callback group for joint state subscriber
        self._joint_cb_group = MutuallyExclusiveCallbackGroup()
        self._joint_subscription = self.create_subscription(
            JointState,
            f"/{robot_name}/joint_states",
            self._joint_callback,
            10,
            callback_group=self._joint_cb_group,
        )

        time.sleep(1)
        self._n_errors = 0

        self._upper_joint_limits = np.array(self.bot.arm.group_info.joint_upper_limits)
        self._lower_joint_limits = np.array(self.bot.arm.group_info.joint_lower_limits)
        self._qn = self.bot.arm.group_info.num_joints

        self.joint_names = self.bot.arm.group_info.joint_names
        self.default_rot = np.dot(
            tr.eulerAnglesToRotationMatrix([0, 0, normal_base_angle]), DEFAULT_ROTATION
        )

        self.neutral_joint_angles = NEUTRAL_JOINT_STATE
        self.enable_rotation = enable_rotation

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.get_logger().info("Received shutdown signal, cleaning up...")
        self.clean_shutdown()

    def reboot_motor(self, joint_name: str):
        """Experimental function to reboot the motor
        Supported joint names:
            - waist, shoulder, elbow, forearm_roll,
            - wrist_angle, wrist_rotate, gripper, left_finger, right_finger
        """
        # Create service client
        client = self.create_client(Reboot, "/wx250s/reboot_motors")

        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Reboot service not available")
            return None

        try:
            request = Reboot.Request()
            request.cmd_type = "single"
            request.name = joint_name
            request.enable = True
            request.smart_reboot = True

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            if future.result() is not None:
                return future.result()
            else:
                self.get_logger().error("Service call failed")
                return None

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return None

    def clean_shutdown(self):
        pid = os.getpid()
        self.get_logger().info(f"Exiting with pid: {pid}")
        try:
            self.destroy_node()
            rclpy.shutdown()
        except:
            pass
        finally:
            os.kill(pid, 9)

    def move_to_state(self, target_xyz, target_zangle, duration=1.5):
        new_pose = np.eye(4)
        new_pose[:3, -1] = target_xyz
        new_quat = Quaternion(
            axis=np.array([0.0, 0.0, 1.0]), angle=target_zangle
        ) * Quaternion(matrix=self.default_rot)
        new_pose[:3, :3] = new_quat.rotation_matrix
        self.move_to_eep(new_pose, duration)

    def set_moving_time(self, moving_time):
        self.bot.arm.set_trajectory_time(
            moving_time=moving_time * 1.25, accel_time=moving_time * 0.5
        )

    def move_to_eep(
        self, target_pose, duration=1.5, blocking=True, check_effort=True, step=True
    ):
        try:
            if step and not blocking:
                # this is a call from the `step` function so we use a custom faster way to set the ee pose
                solution, success = self.bot.arm.set_ee_pose_matrix_fast(
                    target_pose, custom_guess=self.get_joint_angles(), execute=True
                )
            else:
                if not step:  # we want to move fast when it is step() call
                    self.set_moving_time(moving_time=duration)
                solution, success = self.bot.arm.set_ee_pose_matrix(
                    target_pose,
                    custom_guess=self.get_joint_angles(),
                    moving_time=duration,
                    accel_time=duration * 0.45,
                    blocking=blocking,
                )

            self.des_joint_angles = solution

            if not success:
                self.get_logger().warn("no IK solution found, do nothing")

            if check_effort:
                current_effort = self.get_joint_effort()
                if (
                    current_effort is not None
                    and np.max(np.abs(current_effort) - ABS_MAX_JOINT_EFFORTS) > 10
                ):
                    self.get_logger().warn(
                        f"violation: {np.abs(current_effort) - ABS_MAX_JOINT_EFFORTS}"
                    )
                    self.get_logger().warn(
                        f"motor number: {np.argmax(np.abs(current_effort) - ABS_MAX_JOINT_EFFORTS)}"
                    )
                    self.get_logger().warn(f"max effort reached: {current_effort}")
                    self.get_logger().warn(
                        f"max effort allowed: {ABS_MAX_JOINT_EFFORTS}"
                    )

        except Exception as e:
            self.get_logger().error(f"stuck during move: {e}")
            self.move_to_neutral()

    def set_joint_angles(self, target_positions, duration=4):
        target_positions_to_reach = [target_positions]
        if len(target_positions_to_reach) > 1000:
            self.get_logger().error("set_joint_angles failed")
            raise Environment_Exception
        try:
            while target_positions_to_reach:
                target_position = target_positions_to_reach[-1]
                success = self.bot.arm.set_joint_positions(
                    target_position, moving_time=duration
                )
                if success is False:
                    intermediate_pos = np.mean(
                        [self.get_joint_angles(), target_position], axis=0
                    )
                    target_positions_to_reach.append(intermediate_pos)
                else:
                    target_positions_to_reach.pop()
        except Exception as e:
            self.get_logger().error(f"stuck during motion: {e}")

    def check_motor_status_and_reboot(self):
        self.get_logger().info("checking motor status")
        status_codes = self.bot.dxl.robot_get_motor_registers(
            "group", "all", "Hardware_Error_Status"
        )

        if len(status_codes.values) < 7:
            self.get_logger().warn("Some motor went wrong!")
            self.bot.dxl.robot_reboot_motors(
                "group", "all", enable=True, smart_reboot=True
            )
            self.get_logger().info("robot rebooted")
            self.move_to_neutral()
            raise Environment_Exception

    def move_to_neutral(self, duration=4):
        self.get_logger().info("moving to neutral..")
        try:
            self.bot.arm.set_joint_positions(
                self.neutral_joint_angles, moving_time=duration
            )
            current_angles = self.get_joint_angles()
            if (
                current_angles is not None
                and np.linalg.norm(self.neutral_joint_angles - current_angles) > 0.1
            ):
                self.get_logger().warn("moving to neutral failed!")
                self.check_motor_status_and_reboot()
        except Exception as e:
            self.get_logger().error(f"stuck during reset: {e}")

    def _joint_callback(self, msg):
        with self._joint_lock:
            for name, position, velocity, effort in zip(
                msg.name, msg.position, msg.velocity, msg.effort
            ):
                self._angles[name] = position
                self._velocities[name] = velocity
                self._effort[name] = effort

    def get_joint_angles(self):
        """
        Returns current joint angles
        """
        with self._joint_lock:
            try:
                return np.array([self._angles[k] for k in self.joint_names])
            except KeyError:
                return None

    def get_joint_effort(self):
        """
        Returns current joint effort
        """
        with self._joint_lock:
            try:
                return np.array([self._effort[k] for k in self.joint_names])
            except KeyError:
                return None

    def get_joint_angles_velocity(self):
        """
        Returns velocities for joints
        """
        with self._joint_lock:
            try:
                return np.array([self._velocities[k] for k in self.joint_names])
            except KeyError:
                return None

    def get_cartesian_pose(self, matrix=False):
        """Returns cartesian end-effector pose"""
        joint_positions = list(
            self.bot.dxl.joint_states.position[
                self.bot.arm.waist_index : (self._qn + self.bot.arm.waist_index)
            ]
        )
        pose = FKinSpace(
            self.bot.arm.robot_des.M, self.bot.arm.robot_des.Slist, joint_positions
        )
        if matrix:
            return pose
        else:
            return np.concatenate(
                [pose[:3, -1], np.array(Quaternion(matrix=pose[:3, :3]).elements)]
            )

    def _init_gripper(self, gripper_attached, gripper_params):
        if gripper_attached == "custom":
            self._gripper = GripperController(
                robot_name=self.bot.dxl.robot_name,
                des_pos_max=gripper_params.des_pos_max,
                des_pos_min=gripper_params.des_pos_min,
                node=self,
            )
            self.custom_gripper_controller = True
        elif gripper_attached == "custom_narrow":
            self._gripper = GripperController(
                robot_name=self.bot.dxl.robot_name,
                upper_limit=0.022,
                des_pos_max=gripper_params.des_pos_max,
                des_pos_min=gripper_params.des_pos_min,
                node=self,
            )
            self.custom_gripper_controller = True
        elif gripper_attached == "default":
            self.custom_gripper_controller = False
        else:
            raise ValueError(
                "gripper_attached value has to be either 'custom', 'custom_narrow' or 'default'"
            )

    def get_gripper_desired_position(self):
        if self.custom_gripper_controller:
            return self._gripper.get_gripper_target_position()
        else:
            return self.des_gripper_state

    def set_continuous_gripper_position(self, target):
        assert self.custom_gripper_controller
        self._gripper.set_continuous_position(target)

    def get_gripper_position(self):
        assert self.custom_gripper_controller
        return self.get_continuous_gripper_position()

    def get_continuous_gripper_position(self):
        assert self.custom_gripper_controller
        return self._gripper.get_continuous_position()

    def wait_until_gripper_position_reached(self):
        if self.custom_gripper_controller:
            goal_reached = (
                self.get_gripper_desired_position() - 0.075
                < self.get_continuous_gripper_position()
                < self.get_gripper_desired_position() + 0.075
            )
            if goal_reached:
                return
            still_moving = True
            previous_position = self.get_continuous_gripper_position()
            while still_moving:
                time.sleep(0.05)
                current_position = self.get_continuous_gripper_position()
                still_moving = not (
                    previous_position - 0.01
                    < current_position
                    < previous_position + 0.01
                )
                previous_position = current_position

            time.sleep(0.1)

    def open_gripper(self, wait=False):
        if self.custom_gripper_controller:
            self._gripper.open()
        else:
            self.des_gripper_state = np.array([1])
            self.bot.gripper.open()

    def close_gripper(self, wait=False):
        if self.custom_gripper_controller:
            self._gripper.close()
        else:
            self.des_gripper_state = np.array([0])
            self.bot.gripper.close()


def main():
    rclpy.init()

    robot_name = "wx250s"
    print_debug = True
    gripper_params = {}
    interbotix_node = InterbotixRobotNode(robot_name)

    try:
        controller = WidowX_Controller(
            robot_name=robot_name,
            print_debug=print_debug,
            gripper_params=gripper_params,
            enable_rotation="6dof",
            gripper_attached="custom",
            normal_base_angle=0,
            node=interbotix_node,
        )

        executor = MultiThreadedExecutor()
        executor.add_node(interbotix_node)
        executor.add_node(controller)
        executor.spin()

    except KeyboardInterrupt:
        pass
    finally:
        if "controller" in locals():
            controller.clean_shutdown()


if __name__ == "__main__":
    main()

