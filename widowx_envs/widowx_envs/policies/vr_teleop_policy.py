#! /usr/bin/python3
import threading
import time

import numpy as np

from widowx_envs.policies.policy import Policy
from widowx_envs.utils.utils import AttrDict, banner
import widowx_envs.utils.transformation_utils as tr


def vec_to_reorder_mat(vec):
    X = np.zeros((len(vec), len(vec)))
    for i in range(X.shape[0]):
        ind = int(abs(vec[i])) - 1
        X[i, ind] = np.sign(vec[i])
    return X


class VRTeleopPolicy(Policy):
    def __init__(self, ag_params, policyparams):
        self._hp = self._default_hparams()
        self._override_defaults(policyparams)

        self.env = ag_params.env_handle
        self.reader = self.env.oculus_reader
        self.action_space = self.env._hp.action_mode
        self.internal_counter = 0

        self.controller_id = 'r' if self._hp.right_controller else 'l'
        self.spatial_coeff = self._hp.spatial_coeff
        self.global_to_env_mat = vec_to_reorder_mat(self._hp.rmat_reorder)
        self.vr_to_global_mat = np.eye(4)
        self.reset_orientation = True
        self._lock = threading.Lock()
        self.reset_state()
        threading.Thread(target=self._update_internal_state, daemon=True).start()

    def _default_hparams(self):
        dict = AttrDict(
            load_file="",
            type=None,
            policy_T=None,
            right_controller=True,
            spatial_coeff=1.0,
            rmat_reorder=[-2, -1, -3, 4],
            gripper_scale=1.5,
        )
        default_dict = super(Policy, self)._default_hparams()
        default_dict.update(dict)
        return default_dict

    def reset_state(self):
        with self._lock:
            self._state = {
                "poses": {},
                "buttons": {"A": False, "B": False, "X": False, "Y": False},
                "movement_enabled": False,
                "controller_on": True,
            }
            self.update_sensor = True
            self.reset_origin = True
            self.robot_origin = None
            self.vr_origin = None
            self.vr_state = None

    def reset(self):
        self.internal_counter = 0
        self.reset_state()

    def _update_internal_state(self, num_wait_sec=5, hz=50):
        side = self.controller_id.upper()
        last_read_time = time.time()
        joystick_was_pressed = False
        joystick_blocked = False  # a press that started as "discard" stays ignored until released
        while True:
            time.sleep(1 / hz)
            poses, buttons = self.reader.get_transformations_and_buttons()
            with self._lock:
                self._state["controller_on"] = time.time() - last_read_time < num_wait_sec
                if self.controller_id not in poses or side + "G" not in buttons:
                    continue

                # the joystick press also means "discard" while the env asks to save the trajectory
                raw_joystick = buttons[side + "J"]
                awaiting_confirmation = getattr(self.env, "awaiting_confirmation", False)
                joystick_blocked = raw_joystick and (joystick_blocked or awaiting_confirmation)
                joystick_pressed = raw_joystick and not joystick_blocked

                toggled = self._state["movement_enabled"] != buttons[side + "G"]
                self.update_sensor = self.update_sensor or buttons[side + "G"]
                self.reset_orientation = self.reset_orientation or joystick_pressed
                self.reset_origin = self.reset_origin or toggled

                self._state["poses"] = poses
                self._state["buttons"] = buttons
                self._state["movement_enabled"] = buttons[side + "G"]
                self._state["controller_on"] = True
                last_read_time = time.time()

                # update the definition of "forward"
                stop_updating = joystick_pressed or self._state["movement_enabled"]
                if self.reset_orientation:
                    rot_mat = np.asarray(poses[self.controller_id])
                    if stop_updating:
                        self.reset_orientation = False
                    try:
                        rot_mat = np.linalg.inv(rot_mat)
                    except np.linalg.LinAlgError:
                        rot_mat = np.eye(4)
                        self.reset_orientation = True
                    self.vr_to_global_mat = rot_mat
                    if not self.reset_orientation and not (joystick_pressed and joystick_was_pressed):
                        cause = 'joystick click' if joystick_pressed else 'first grip'
                        banner('Forward direction set to the controller heading ({})'.format(cause), 'ok')
                joystick_was_pressed = joystick_pressed

    def _process_reading(self):
        pose = self.global_to_env_mat @ self.vr_to_global_mat @ np.asarray(self._state["poses"][self.controller_id])
        trig = self._state["buttons"]["rightTrig" if self.controller_id == 'r' else "leftTrig"][0]
        self.vr_state = {"pos": self.spatial_coeff * pose[:3, 3], "rot": pose[:3, :3], "gripper": trig}

    def act(self, t=None, i_tr=None, images=None, task_id=None):
        with self._lock:
            if self._state["poses"] == {} or self.controller_id not in self._state["poses"]:
                return self.get_zero_action()
            if self.update_sensor:
                self._process_reading()
                self.update_sensor = False
            reset_origin, self.reset_origin = self.reset_origin, False
            vr_state = self.vr_state
            movement_enabled = self._state["movement_enabled"]

        prev_target, _ = self.env.get_target_state()
        if reset_origin or self.robot_origin is None:
            self.robot_origin = prev_target
            self.vr_origin = vr_state

        rot_offset = vr_state["rot"].dot(self.vr_origin["rot"].T)
        target_pos = self.robot_origin[:3, 3] + (vr_state["pos"] - self.vr_origin["pos"])
        target_rot = rot_offset.dot(self.robot_origin[:3, :3])
        new_robot_transform = tr.RpToTrans(target_rot, target_pos)

        if self.action_space == '3trans1rot':
            new_robot_transform = self.zero_out_pitchroll(new_robot_transform)
        if self.action_space == '3trans':
            new_robot_transform = self.zero_out_yawpitchroll(new_robot_transform)

        des_gripper_position = 1.0 - float(np.clip(vr_state["gripper"] * self._hp.gripper_scale, 0.0, 1.0))
        delta_robot_transform = new_robot_transform.dot(tr.TransInv(prev_target))
        actions = tr.transform2action_local(delta_robot_transform, des_gripper_position, self.env._controller.pose[:3])

        if self.env._hp.action_mode == '3trans1rot':
            actions = np.concatenate([actions[:3], np.array([actions[5]]), np.array([des_gripper_position])])
        if self.env._hp.action_mode == '3trans':
            actions = np.concatenate([actions[:3], np.array([des_gripper_position])])

        output = {'actions': actions, 'new_robot_transform': new_robot_transform,
                  'delta_robot_transform': delta_robot_transform, 'policy_type': 'VRTeleopPolicy'}

        if movement_enabled:
            self.internal_counter += 1
        if self._hp.policy_T and self.internal_counter >= self._hp.policy_T:
            output['done'] = True
        return output

    def forward(self, obs_dict=None, include_info=False):
        out = self.act()
        if include_info:
            info = {"target_cartesian_position": out.get("new_robot_transform"), **self.get_info()}
            return out["actions"], info
        return out["actions"]

    def get_action(self, obs_np, task_id_vec=None):
        out = self.act(images=obs_np, task_id=task_id_vec)
        return out['actions'], {'policy_type': out['policy_type']}

    def get_info(self):
        with self._lock:
            buttons = self._state["buttons"]
            return {
                "success": buttons.get("A" if self.controller_id == 'r' else "X", False),
                "failure": buttons.get("B" if self.controller_id == 'r' else "Y", False),
                "movement_enabled": self._state["movement_enabled"],
                "controller_on": self._state["controller_on"],
            }

    def get_zero_action(self):
        if self.env._hp.action_mode == '3trans3rot':
            actions = np.concatenate([np.zeros(6), np.array([1])])
        elif self.env._hp.action_mode == '3trans1rot':
            actions = np.concatenate([np.zeros(4), np.array([1])])
        elif self.env._hp.action_mode == '3trans':
            actions = np.concatenate([np.zeros(3), np.array([1])])
        else:
            raise NotImplementedError
        return {'actions': actions, 'new_robot_transform': np.eye(4), 'delta_robot_transform': np.eye(4),
                'policy_type': 'VRTeleopPolicy'}

    def zero_out_pitchroll(self, new_robot_transform):
        rot, xyz = tr.TransToRp(new_robot_transform)
        euler = tr.rotationMatrixToEulerAngles(rot.dot(self.env._controller.default_rot.transpose()), check_error_thresh=1e-5)
        euler[:2] = np.zeros(2)  # zeroing out pitch roll
        new_rot = tr.eulerAnglesToRotationMatrix(euler).dot(self.env._controller.default_rot)
        return tr.RpToTrans(new_rot, xyz)

    def zero_out_yawpitchroll(self, new_robot_transform):
        rot, xyz = tr.TransToRp(new_robot_transform)
        return tr.RpToTrans(self.env._controller.default_rot, xyz)
