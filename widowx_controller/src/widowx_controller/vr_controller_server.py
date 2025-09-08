#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time
from oculus_reader import OculusReader
from widowx_controller.widowx_controller import WidowX_Controller
from widowx_envs.control_loops import Environment_Exception
import widowx_envs.utils.transformation_utils as tr
from pyquaternion import Quaternion
import numpy as np

from std_msgs.msg import Float64

from widowx_controller.srv import GotoNeutral, GotoNeutral_Request, GotoNeutral_Response
from widowx_controller.srv import MoveToEEP, MoveToEEP_Request, MoveToEEP_Response
from widowx_controller.srv import MoveToState, MoveToState_Request, MoveToState_Response
from widowx_controller.srv import (
    GetCartesianPose,
    GetCartesianPose_Request,
    GetCartesianPose_Response,
)
from widowx_controller.srv import GetState, GetState_Request, GetState_Response
from widowx_controller.srv import (
    GetVRButtons,
    GetVRButtons_Request,
    GetVRButtons_Response,
)
from widowx_controller.srv import (
    EnableController,
    EnableController_Request,
    EnableController_Response,
)
from widowx_controller.srv import (
    DisableController,
    DisableController_Request,
    DisableController_Response,
)

from widowx_controller.widowx_controller import publish_transform

##############################################################################


class VR_WidowX_ControllerServer(WidowX_Controller):
    def __init__(self, grasp_mode="continuous", *args, **kwargs):
        super(VR_WidowX_ControllerServer, self).__init__(*args, **kwargs)

        self.reader = OculusReader()

        self._moving_time = 0.05
        self.prev_handle_press = False
        self.reference_vr_transform = None
        self.reference_robot_transform = None

        self.do_reset = False
        self.task_stage = 0
        self.num_task_stages = int(1e9)
        self.last_update_time = time.time()

        self._control_loop_active = True

        self._service_cb_group = ReentrantCallbackGroup()

        # Create services
        self._goto_neutral_service = self.create_service(
            GotoNeutral,
            "go_to_neutral",
            self.goto_neutral_service,
            callback_group=self._service_cb_group,
        )
        self._move_to_eep_service = self.create_service(
            MoveToEEP,
            "move_to_eep",
            self.move_to_eep_service,
            callback_group=self._service_cb_group,
        )
        self._move_to_state_service = self.create_service(
            MoveToState,
            "move_to_state",
            self.move_to_state_service,
            callback_group=self._service_cb_group,
        )
        self._get_cartesian_pose_service = self.create_service(
            GetCartesianPose,
            "get_cartesian_pose",
            self.get_cartesian_pose_service,
            callback_group=self._service_cb_group,
        )
        self._get_state_service = self.create_service(
            GetState,
            "get_state",
            self.get_state_service,
            callback_group=self._service_cb_group,
        )
        self._get_vr_buttons_service = self.create_service(
            GetVRButtons,
            "get_vr_buttons",
            self.get_vr_buttons_service,
            callback_group=self._service_cb_group,
        )
        self._enable_controller_service = self.create_service(
            EnableController,
            "enable_controller",
            self.enable_controller_service,
            callback_group=self._service_cb_group,
        )
        self._disable_controller_service = self.create_service(
            DisableController,
            "disable_controller",
            self.disable_controller_service,
            callback_group=self._service_cb_group,
        )

        self.pub_gripper_command = self.create_publisher(Float64, "/gripper_despos", 3)
        self.grasp_mode = grasp_mode

        self.last_pressed_times = {}

        self.initial_vr_offset = None

    def _init_gripper(self, gripper_attached, gripper_params):
        """Override gripper initialization - VR controller handles gripper directly"""
        pass

    def goto_neutral_service(self, request, response):
        self.get_logger().info(f"moving to neutral: seconds: {request.duration}")
        self._control_loop_active = False
        self.move_to_neutral(request.duration)
        self._control_loop_active = True
        return response

    def move_to_eep_service(self, request, response):
        self._control_loop_active = False
        des_transform = np.array(request.des_eep).reshape(4, 4)
        self.move_to_eep(des_transform, request.duration)
        self._control_loop_active = True
        return response

    def move_to_state_service(self, request, response):
        self._control_loop_active = False
        try:
            self.move_to_state(
                request.target_xyz, request.target_zangle, request.duration
            )
            response.success = True
        except Environment_Exception:
            response.success = False
        self._control_loop_active = True
        return response

    def get_cartesian_pose_service(self, request, response):
        pose = self.get_cartesian_pose(matrix=True)
        response.pose = pose.flatten().tolist()
        return response

    def get_state_service(self, request, response):
        joint_angles, joint_velocities, cartesian_pose = self.get_state()
        response.joint_angles = (
            joint_angles.tolist() if joint_angles is not None else []
        )
        response.joint_velocities = (
            joint_velocities.tolist() if joint_velocities is not None else []
        )
        response.cartesian_pose = (
            cartesian_pose.tolist() if cartesian_pose is not None else []
        )
        return response

    def get_vr_buttons_service(self, request, response):
        def check_press(key):
            if key in self.last_pressed_times:
                if time.time() - self.last_pressed_times[key] < 0.5:
                    return True
            return False

        response.rg = int(check_press("RG"))
        response.a = int(check_press("A"))
        response.b = int(check_press("B"))
        response.rj = int(check_press("RJ"))
        return response

    def enable_controller_service(self, request, response):
        self._control_loop_active = True
        return response

    def disable_controller_service(self, request, response):
        self._control_loop_active = False
        return response

    def get_pose_and_button(self):
        poses, buttons = self.reader.get_transformations_and_buttons()

        for key, value in buttons.items():
            if not isinstance(value, tuple):
                if value:
                    self.last_pressed_times[key] = time.time()

        if "r" not in poses:
            return None, None, None, None
        return poses["r"], buttons["RTr"], buttons["rightTrig"][0], buttons["RG"]

    def set_gripper_position(self, position):
        msg = Float64()
        msg.data = position
        self.pub_gripper_command.publish(msg)

    def update_robot_cmds(self):
        """Main control loop - called externally or by timer"""
        self.last_update_time = time.time()
        t1 = time.time()

        current_vr_transform, trigger, trigger_continuous, handle_button = (
            self.get_pose_and_button()
        )
        if current_vr_transform is None:
            return
        elif not self._control_loop_active:
            self.prev_handle_press = False
            return
        else:
            if not self.prev_handle_press and handle_button:
                self.get_logger().info("resetting reference pose")
                self.reference_vr_transform = self.oculus_to_robot(current_vr_transform)
                self.initial_vr_offset = tr.RpToTrans(
                    np.eye(3), self.reference_vr_transform[:3, 3]
                )
                self.reference_vr_transform = tr.TransInv(self.initial_vr_offset).dot(
                    self.reference_vr_transform
                )

                self.reference_robot_transform = self.get_cartesian_pose(matrix=True)
                self.bot.arm.set_trajectory_time(
                    moving_time=self._moving_time, accel_time=self._moving_time * 0.5
                )

            if not handle_button:
                self.reference_vr_transform = None
                self.reference_robot_transform = self.get_cartesian_pose(matrix=True)
                self.prev_handle_press = False
                return

        self.prev_handle_press = True

        self.get_logger().info(f"gripper set point: {1 - trigger_continuous}")
        self.set_gripper_position(1 - trigger_continuous)

        current_vr_transform = self.oculus_to_robot(current_vr_transform)
        current_vr_transform = tr.TransInv(self.initial_vr_offset).dot(
            current_vr_transform
        )

        publish_transform(self, current_vr_transform, "currentvr_robotsystem")
        publish_transform(self, self.reference_vr_transform, "reference_vr_transform")

        delta_vr_transform = current_vr_transform.dot(
            tr.TransInv(self.reference_vr_transform)
        )
        publish_transform(
            self, self.reference_robot_transform, "reference_robot_transform"
        )

        M_rob, v_rob = tr.TransToRp(self.reference_robot_transform)
        M_delta, v_delta = tr.TransToRp(delta_vr_transform)
        new_robot_transform = tr.RpToTrans(M_delta.dot(M_rob), v_rob + v_delta)

        publish_transform(self, new_robot_transform, "des_robot_transform")

        delta_translation_norm = np.linalg.norm(
            self.get_cartesian_pose(matrix=True)[:3, 3] - new_robot_transform[:3, 3]
        )
        if delta_translation_norm > 0.2:
            self.get_logger().warn(
                f"delta transform norm too large: {delta_translation_norm}"
            )

            return

        try:
            tset = time.time()
            solution, success = self.bot.arm.set_ee_pose_matrix_fast(
                new_robot_transform, custom_guess=self.get_joint_angles()
            )
            self.get_logger().debug(f"time for setting pos: {time.time() - tset}")
        except Exception as e:
            self.get_logger().error(f"stuck during move: {e}")
            self.move_to_neutral()

        loop_time = time.time() - t1
        self.get_logger().debug(f"loop time: {loop_time}")
        if loop_time > 0.02:
            self.get_logger().warn("Control loop is slow!")

    def get_state(self):
        """Get current robot state"""
        joint_angles = self.get_joint_angles()
        joint_velocities = self.get_joint_angles_velocity()
        cartesian_pose = self.get_cartesian_pose()
        return joint_angles, joint_velocities, cartesian_pose

    def oculus_to_robot(self, current_vr_transform):
        """Transform Oculus coordinate system to robot coordinate system"""
        current_vr_transform = (
            tr.RpToTrans(
                Quaternion(axis=[0, 0, 1], angle=-np.pi / 2).rotation_matrix,
                np.zeros(3),
            )
            .dot(
                tr.RpToTrans(
                    Quaternion(axis=[1, 0, 0], angle=np.pi / 2).rotation_matrix,
                    np.zeros(3),
                )
            )
            .dot(current_vr_transform)
        )
        return current_vr_transform


def main():
    """Main function to run the VR controller server"""
    rclpy.init()

    try:
        controller = VR_WidowX_ControllerServer(
            robot_name="wx250s",
            print_debug=True,
            gripper_attached="custom",
            enable_rotation="6dof",
            gripper_params={},
        )

        controller.get_logger().info("VR WidowX Controller Server started")

        controller.set_moving_time(1)
        controller.move_to_neutral(duration=3)
        controller.set_moving_time(controller._moving_time)

        executor = MultiThreadedExecutor()

        control_timer = controller.create_timer(
            0.02, controller.update_robot_cmds
        )

        controller.get_logger().info("Starting main control loop")

        rclpy.spin(controller, executor)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        if "controller" in locals():
            controller.get_logger().error(f"Error in VR controller: {e}")
    finally:
        if "controller" in locals():
            try:
                controller.destroy_node()
            except:
                pass

        try:
            rclpy.shutdown()
        except:
            pass


def run_manual():
    """Alternative run function that mimics the original manual control loop"""
    rclpy.init()

    try:
        controller = VR_WidowX_ControllerServer(
            robot_name="wx250s",
            print_debug=True,
            gripper_attached="custom",
            enable_rotation="6dof",
            gripper_params={},
        )

        controller.set_moving_time(1)
        controller.move_to_neutral(duration=3)
        controller.set_moving_time(controller._moving_time)

        executor = MultiThreadedExecutor()

        import threading

        spin_thread = threading.Thread(target=lambda: rclpy.spin(controller, executor))
        spin_thread.daemon = True
        spin_thread.start()

        controller.get_logger().info("Starting manual control loop")
        while rclpy.ok():
            controller.update_robot_cmds()
            time.sleep(0.02)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        if "controller" in locals():
            controller.get_logger().error(f"Error in VR controller: {e}")
    finally:
        if "controller" in locals():
            try:
                controller.destroy_node()
            except:
                pass

        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()

