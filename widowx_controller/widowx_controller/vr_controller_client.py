#!/usr/bin/env python3

import logging
import numpy as np
import rclpy
from rclpy.node import Node

from widowx_controller.srv import DisableController
from widowx_controller.srv import EnableController
from widowx_controller.srv import GetCartesianPose
from widowx_controller.srv import GetGripperDesiredState
from widowx_controller.srv import GetState
from widowx_controller.srv import GetVRButtons
from widowx_controller.srv import GotoNeutral
from widowx_controller.srv import MoveToEEP
from widowx_controller.srv import MoveToState
from widowx_controller.srv import OpenGripper
from widowx_controller.srv import SetGripperPosition
from widowx_envs.utils.exceptions import Environment_Exception

from widowx_controller.controller_base import RobotControllerBase

# TODO: abstract class for all controllers:
# with internal ROS init, thus higher level impl doesnt need to interact
# with ROS directly, middleware agnostic


class WidowX_VRControllerClient(RobotControllerBase, Node):
    def __init__(self, print_debug=False, node_name="vr_controller_client"):
        Node.__init__(self, node_name)

        logger = logging.getLogger("robot_logger")
        formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
        )
        logger.setLevel(logging.DEBUG)

        log_level = logging.INFO
        if print_debug:
            log_level = logging.DEBUG
        ch = logging.StreamHandler()
        ch.setLevel(log_level)
        ch.setFormatter(formatter)
        logger.addHandler(ch)

        self._goto_neutral_client = self.create_client(GotoNeutral, "go_to_neutral")
        self._move_to_eep_client = self.create_client(MoveToEEP, "move_to_eep")
        self._move_to_state_client = self.create_client(MoveToState, "move_to_state")
        self._get_cartesian_pose_client = self.create_client(
            GetCartesianPose, "get_cartesian_pose"
        )
        self._get_state_client = self.create_client(GetState, "get_state")
        self._get_vr_buttons_client = self.create_client(GetVRButtons, "get_vr_buttons")
        self._enable_controller_client = self.create_client(
            EnableController, "enable_controller"
        )
        self._disable_controller_client = self.create_client(
            DisableController, "disable_controller"
        )
        self._get_gripper_desired_state_client = self.create_client(
            GetGripperDesiredState, "get_gripper_desired_state"
        )
        self._open_gripper_client = self.create_client(OpenGripper, "open_gripper")
        self._set_gripper_position_client = self.create_client(
            SetGripperPosition, "set_gripper_position"
        )

    def _wait_for_service(self, client, service_name, timeout_sec=5.0):
        """Wait for service to become available"""
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                f"Service {service_name} not available after {timeout_sec} seconds"
            )
            return False
        return True

    def _call_service(self, client, request, service_name, timeout_sec=10.0):
        """Generic service call with timeout and error handling"""
        if not self._wait_for_service(client, service_name):
            return None

        try:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

            if future.result() is not None:
                return future.result()
            else:
                self.get_logger().error(f"Service call to {service_name} failed")
                return None
        except Exception as e:
            self.get_logger().error(f"Service call to {service_name} failed: {e}")
            return None

    def move_to_neutral(self, duration=4):
        request = GotoNeutral.Request()
        request.duration = duration
        response = self._call_service(
            self._goto_neutral_client, request, "go_to_neutral"
        )
        return response is not None

    def move_to_eep(self, target_pose, duration=1.5):
        request = MoveToEEP.Request()
        request.des_eep = target_pose.flatten().tolist()
        request.duration = duration
        response = self._call_service(self._move_to_eep_client, request, "move_to_eep")
        return response is not None

    def move_to_state(self, startpos, zangle, duration=1.5):
        request = MoveToState.Request()
        request.target_xyz = (
            startpos.tolist() if hasattr(startpos, "tolist") else list(startpos)
        )
        request.target_zangle = float(zangle)
        request.duration = duration

        response = self._call_service(
            self._move_to_state_client, request, "move_to_state"
        )
        if response is not None:
            if not response.success:
                raise Environment_Exception("Move to state failed")
            return True
        return False

    def get_cartesian_pose(self, matrix=False):
        assert matrix, "Only matrix=True is supported"

        request = GetCartesianPose.Request()
        response = self._call_service(
            self._get_cartesian_pose_client, request, "get_cartesian_pose"
        )

        if response is not None:
            return np.array(response.pose).reshape(4, 4)
        return None

    def get_state(self):
        request = GetState.Request()
        response = self._call_service(self._get_state_client, request, "get_state")

        if response is not None:
            joint_angles = (
                np.array(response.joint_angles) if response.joint_angles else None
            )
            joint_velocities = (
                np.array(response.joint_velocities)
                if response.joint_velocities
                else None
            )
            cartesian_pose = (
                np.array(response.cartesian_pose) if response.cartesian_pose else None
            )
            return joint_angles, joint_velocities, cartesian_pose
        return None, None, None

    def get_vr_buttons(self):
        request = GetVRButtons.Request()
        response = self._call_service(
            self._get_vr_buttons_client, request, "get_vr_buttons"
        )

        if response is not None:
            return {
                "handle": bool(response.rg),  # RG button (right grip) acts as handle
                "A": bool(response.a),
                "B": bool(response.b),
                "RJ": bool(response.rj),
            }
        return None

    def enable_controller(self):
        request = EnableController.Request()
        response = self._call_service(
            self._enable_controller_client, request, "enable_controller"
        )
        return response is not None

    def disable_controller(self):
        request = DisableController.Request()
        response = self._call_service(
            self._disable_controller_client, request, "disable_controller"
        )
        return response is not None

    def get_gripper_desired_state(self):
        request = GetGripperDesiredState.Request()
        response = self._call_service(
            self._get_gripper_desired_state_client, request, "get_gripper_desired_state"
        )

        if response is not None:
            return (
                response.state
            )
        return None

    def open_gripper(self, wait=False):
        self.get_logger().info("opening gripper")
        request = OpenGripper.Request()
        response = self._call_service(
            self._open_gripper_client, request, "open_gripper"
        )
        return response is not None

    def set_gripper_position(self, position):
        request = SetGripperPosition.Request()
        request.position = float(position)
        response = self._call_service(
            self._set_gripper_position_client, request, "set_gripper_position"
        )
        return response is not None

    def close_gripper(self, wait=False):
        """Close gripper - typically sets position to 0"""
        return self.set_gripper_position(0.0)

    def destroy_client(self):
        """Clean up resources"""
        try:
            self.destroy_node()
        except:
            pass


def main():
    """Example usage of the VR controller client"""
    rclpy.init()

    try:
        client = WidowX_VRControllerClient(print_debug=True)

        client.get_logger().info("VR Controller Client started")

        client.enable_controller()
        client.move_to_neutral(duration=3.0)

        joint_angles, joint_velocities, cartesian_pose = client.get_state()
        if joint_angles is not None:
            client.get_logger().info(f"Current joint angles: {joint_angles}")

        buttons = client.get_vr_buttons()
        if buttons is not None:
            client.get_logger().info(f"VR button states: {buttons}")

        pose = client.get_cartesian_pose(matrix=True)
        if pose is not None:
            client.get_logger().info(f"Current pose shape: {pose.shape}")

        client.get_logger().info(
            "Example complete - keeping node alive for service calls"
        )

        rclpy.spin(client)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        if "client" in locals():
            client.get_logger().error(f"Error in VR controller client: {e}")
    finally:
        if "client" in locals():
            client.destroy_client()

        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()
