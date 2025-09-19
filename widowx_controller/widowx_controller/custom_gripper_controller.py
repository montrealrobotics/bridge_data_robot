#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import numpy as np
from threading import Lock
from sensor_msgs.msg import JointState
import time

from std_msgs.msg import Float64
from widowx_controller_interfaces.srv import OpenGripper
from widowx_controller_interfaces.srv import (
    GetGripperDesiredState
)

try:
    # older version of interbotix sdk
    from interbotix_xs_sdk.msg import JointSingleCommand
except:
    # newer version of interbotix sdk
    from interbotix_xs_msgs.msg import JointSingleCommand

from widowx_controller.controller_base import GripperControllerBase


class GripperController(GripperControllerBase):
    def __init__(
        self,
        robot_name,
        node=None,
        upper_limit=0.035,
        lower_limit=0.010,
        des_pos_max=1,
        des_pos_min=0,
    ):
        self.node = node
        if self.node is None:
            self.node = Node("gripper_controller")
            self._owns_node = True
        else:
            self._owns_node = False

        assert (
            des_pos_max >= des_pos_min
        ), "gripper des_pos_max has to be >= des_pos_min"
        self.des_pos_max = des_pos_max
        self.des_pos_min = des_pos_min
        self._upper_limit = upper_limit
        self._lower_limit = lower_limit
        assert self._upper_limit > self._lower_limit

        self._timer = self.node.create_timer(0.02, self.update_gripper_pwm)

        self._joint_lock = Lock()
        self._des_pos_lock = Lock()

        self._angles = {}
        self._velocities = {}

        self._pub_gripper_command = self.node.create_publisher(
            JointSingleCommand, f"/{robot_name}/commands/joint_single", 3
        )

        self._joint_cb_group = MutuallyExclusiveCallbackGroup()
        self._joint_subscription = self.node.create_subscription(
            JointState,
            f"/{robot_name}/joint_states",
            self._joint_callback,
            10,
            callback_group=self._joint_cb_group,
        )

        self._moving = False
        self._time_movement_started = None
        self._grace_period_until_can_be_marked_as_stopped = 0.1
        self._des_pos = None
        self.des_pos = self._upper_limit

    @property
    def des_pos(self):
        return self._des_pos

    @des_pos.setter
    def des_pos(self, value):
        if value != self._des_pos:
            with self._des_pos_lock:
                self._moving = True
                self._time_movement_started = time.time()
                self._des_pos = value

    def get_gripper_pos(self):
        with self._joint_lock:
            if "left_finger" not in self._angles:
                if self.node is not None:
                    self.node.get_logger().warn(
                        "left_finger should be available in gripper"
                    )
                return 0.0
            return self._angles["left_finger"]

    def _joint_callback(self, msg):
        with self._joint_lock:
            for name, position, velocity in zip(msg.name, msg.position, msg.velocity):
                self._angles[name] = position
                self._velocities[name] = velocity

    def open(self):
        self.des_pos = self._upper_limit

    def close(self):
        self.des_pos = self._lower_limit

    def set_continuous_position(self, target):
        target_clipped = np.clip(target, self.des_pos_min, self.des_pos_max)
        if target != target_clipped:
            if self.node is not None:
                self.node.get_logger().warn(
                    f"Warning target gripper pos outside of range: {target}"
                )
        self.des_pos = self.denormalize(target_clipped)

    def get_continuous_position(self):
        gripper_pos = self.get_gripper_pos()
        return self.normalize(gripper_pos)

    def normalize(self, x):
        return (self.des_pos_max - self.des_pos_min) * (x - self._lower_limit) / (
            self._upper_limit - self._lower_limit
        ) + self.des_pos_min

    def denormalize(self, x):
        return (x - self.des_pos_min) * (self._upper_limit - self._lower_limit) / (
            self.des_pos_max - self.des_pos_min
        ) + self._lower_limit

    def is_moving(self):
        return self._moving

    def get_gripper_target_position(self):
        des_pos_normed = self.normalize(self.des_pos)
        assert des_pos_normed <= self.des_pos_max and des_pos_normed >= self.des_pos_min
        return des_pos_normed

    def update_gripper_pwm(self):
        with self._des_pos_lock:
            moving = self._moving
            des_pos = self.des_pos

        if moving:
            gripper_pos = self.get_gripper_pos()
            ctrl = (des_pos - gripper_pos) * 300
            pwm = self.get_gripper_pwm(ctrl)

            gripper_command = JointSingleCommand()
            gripper_command.name = "gripper"
            gripper_command.cmd = pwm
            self._pub_gripper_command.publish(gripper_command)

    def get_gripper_pwm(self, pressure):
        """
        :param pressure: range -1, 1
        :return: pwm
        """
        pressure = np.clip(pressure, -1, 1)
        offset = 0
        if pressure < 0:
            gripper_pwm = -(offset + int(-pressure * 350))
        else:
            gripper_pwm = offset + int(pressure * 350)

        time_since_movements_started = time.time() - self._time_movement_started
        # Optional: Add logic to stop movement based on velocity and time
        # if abs(self._velocities.get('gripper', 0)) == 0.0 and time_since_movements_started > self._grace_period_until_can_be_marked_as_stopped:
        #     gripper_pwm = 0
        #     self._moving = False
        #     self._time_movement_started = None
        return gripper_pwm

    def destroy(self):
        """Clean up resources"""
        if self._timer is not None:
            self._timer.cancel()
        if self._owns_node and self.node is not None:
            self.node.destroy_node()


class GripperControllerServer(GripperController, Node):
    def __init__(
        self,
        robot_name,
        upper_limit=0.037,
        lower_limit=0.010,
        des_pos_max=1,
        des_pos_min=0,
    ):
        Node.__init__(self, "gripper_controller_server")

        GripperController.__init__(
            self,
            robot_name,
            node=self,
            upper_limit=upper_limit,
            lower_limit=lower_limit,
            des_pos_max=des_pos_max,
            des_pos_min=des_pos_min,
        )

        self._open_gripper_service = self.create_service(
            OpenGripper, "open_gripper", self.open_gripper_service
        )

        self._get_gripper_desired_state_service = self.create_service(
            GetGripperDesiredState,
            "get_gripper_desired_state",
            self.get_gripper_desired_state_service,
        )

        self._gripper_despos_cb_group = ReentrantCallbackGroup()
        self._gripper_despos_subscription = self.create_subscription(
            Float64,
            "/gripper_despos",
            self._gripper_despos_callback,
            10,
            callback_group=self._gripper_despos_cb_group,
        )

        # Create timer for main control loop
        self._control_timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

    def _gripper_despos_callback(self, msg):
        self.set_continuous_position(msg.data)

    def open_gripper_service(self, request, response):
        self.open()
        return response

    def get_gripper_desired_state_service(self, request, response):
        response.state = self.get_gripper_target_position()
        return response

    def control_loop(self):
        """Main control loop - calls update_gripper_pwm"""
        self.update_gripper_pwm()


def main():
    """Main function for running the gripper controller server"""
    rclpy.init()

    try:
        controller = GripperControllerServer(robot_name="wx250s")

        executor = MultiThreadedExecutor()

        controller.get_logger().info("Gripper controller server started")

        rclpy.spin(controller, executor)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        if "controller" in locals():
            controller.get_logger().error(f"Error in gripper controller: {e}")
    finally:
        # Clean up
        if "controller" in locals():
            controller.destroy()

        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()
