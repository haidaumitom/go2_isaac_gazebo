#!/usr/bin/env python3
"""Run a Quadloco-style Go2 policy from ROS 2 state with a hardcoded velocity command.

The node is dry-run by default. It always publishes the actor observation and
action on debug topics, but only creates /lowcmd output when --publish-lowcmd
and the explicit confirmation token are supplied. Even then, a SetBool service
must be called to arm motor output.

The policy command is hardcoded below as [vx, vy, yaw].
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np
import yaml


OBSERVATION_TERMS = ["dof_pos", "dof_vel", "actions", "ang_vel", "gravity_vec", "commands"]
OBSERVATION_DIM = 45
ACTION_DIM = 12

# -----------------------------------------------------------------------------
# Hardcoded velocity command sent to the RL policy: [vx, vy, yaw]
#
# Examples:
#   [0.10, 0.00, 0.00] -> move forward slowly
#   [0.00, 0.10, 0.00] -> move sideways
#   [0.00, 0.00, 0.20] -> yaw
#   [0.00, 0.00, 0.00] -> zero velocity command
# -----------------------------------------------------------------------------
HARDCODED_COMMAND = np.asarray([0.10, 0.00, 0.00], dtype=np.float32)
HARDCODED_COMMAND_LIMITS = np.asarray([1.00, 0.75, 0.85], dtype=np.float32)
ACTION_PRINT_RATE_HZ = 5.0

POS_STOP_F = 2.146e9
VEL_STOP_F = 16000.0
LOWCMD_CONFIRMATION = "I_UNDERSTAND_LOWCMD_CONTROLS_MOTORS"


def load_policy_config(policy_dir: Path) -> tuple[dict[str, Any], Path]:
    config_path = policy_dir / "config.yaml"
    with config_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or len(document) != 1:
        raise ValueError(f"{config_path} must contain exactly one policy mapping")

    config = next(iter(document.values()))
    if not isinstance(config, dict):
        raise ValueError(f"Policy entry in {config_path} is not a mapping")
    if config.get("observations") != OBSERVATION_TERMS:
        raise ValueError(
            "Expected Quadloco observation order "
            f"{OBSERVATION_TERMS}, got {config.get('observations')}"
        )
    if int(config.get("num_observations", -1)) != OBSERVATION_DIM:
        raise ValueError("Quadloco policy must declare num_observations: 45")

    required_vectors = {
        "default_dof_pos": ACTION_DIM,
        "joint_mapping": ACTION_DIM,
        "action_scale": ACTION_DIM,
        "rl_kp": ACTION_DIM,
        "rl_kd": ACTION_DIM,
    }
    for name, expected_size in required_vectors.items():
        values = config.get(name)
        if not isinstance(values, list) or len(values) != expected_size:
            raise ValueError(f"{name} must contain {expected_size} values")

    mapping = [int(value) for value in config["joint_mapping"]]
    if sorted(mapping) != list(range(ACTION_DIM)):
        raise ValueError("joint_mapping must be a permutation of hardware joints 0..11")

    model_path = policy_dir / str(config["model_name"])
    if not model_path.is_file():
        raise FileNotFoundError(f"Policy model does not exist: {model_path}")
    return config, model_path


def quat_rotate_inverse_wxyz(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    """Rotate a world-frame vector into the body frame using a wxyz quaternion."""
    quaternion = np.asarray(quaternion, dtype=np.float32)
    vector = np.asarray(vector, dtype=np.float32)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm < 1.0e-6:
        raise ValueError("Invalid IMU quaternion")
    quaternion = quaternion / norm
    w = quaternion[0]
    xyz = quaternion[1:]
    return (
        vector * (2.0 * w * w - 1.0)
        - 2.0 * w * np.cross(xyz, vector)
        + 2.0 * xyz * np.dot(xyz, vector)
    ).astype(np.float32)


def build_quadloco_observation(
    q_policy: np.ndarray,
    dq_policy: np.ndarray,
    previous_actions: np.ndarray,
    gyro_body: np.ndarray,
    quaternion_wxyz: np.ndarray,
    commands: np.ndarray,
    config: dict[str, Any],
) -> np.ndarray:
    default_q = np.asarray(config["default_dof_pos"], dtype=np.float32)
    observation = np.concatenate(
        (
            (q_policy - default_q) * float(config.get("dof_pos_scale", 1.0)),
            dq_policy * float(config.get("dof_vel_scale", 1.0)),
            previous_actions,
            gyro_body * float(config.get("ang_vel_scale", 1.0)),
            quat_rotate_inverse_wxyz(
                quaternion_wxyz, np.asarray([0.0, 0.0, -1.0], dtype=np.float32)
            ),
            commands * np.asarray(config.get("commands_scale", [1.0, 1.0, 1.0]), dtype=np.float32),
        )
    ).astype(np.float32)
    if observation.shape != (OBSERVATION_DIM,):
        raise RuntimeError(f"Built observation has shape {observation.shape}, expected (45,)")
    if not np.all(np.isfinite(observation)):
        raise ValueError("Observation contains NaN or infinity")
    clip = float(config.get("clip_obs", 100.0))
    return np.clip(observation, -clip, clip)


class HistoryBuffer:
    """Match rl_sar history indexing: index 0 is newest, N is N steps old."""

    def __init__(self, selected_indices: list[int], warm_start: bool):
        self.selected_indices = selected_indices
        self.capacity = max(selected_indices, default=-1) + 1
        self.warm_start = warm_start
        self.frames: deque[np.ndarray] = deque(maxlen=self.capacity)

    def insert_and_select(self, observation: np.ndarray) -> np.ndarray:
        if self.capacity <= 0:
            return np.empty((0, observation.size), dtype=np.float32)
        if not self.frames:
            fill = observation if self.warm_start else np.zeros_like(observation)
            self.frames.extend(fill.copy() for _ in range(self.capacity))
        self.frames.appendleft(observation.copy())
        return np.stack([self.frames[index] for index in self.selected_indices]).astype(np.float32)


class PolicyModel:
    def __init__(self, model_path: Path, mode: str):
        self.mode = mode
        suffix = model_path.suffix.lower()
        if suffix == ".onnx":
            try:
                import onnxruntime as ort
            except ImportError as error:
                raise RuntimeError(
                    "ONNX model selected but Python onnxruntime is not installed"
                ) from error
            self.backend = "onnx"

            # Explicitly use one intra-op thread. This also avoids ONNX Runtime
            # trying to set CPU affinity on systems where that is not permitted.
            session_options = ort.SessionOptions()
            session_options.intra_op_num_threads = 1
            self.model = ort.InferenceSession(
                str(model_path),
                sess_options=session_options,
                providers=["CPUExecutionProvider"],
            )
            self.input_names = [item.name for item in self.model.get_inputs()]
        elif suffix in {".pt", ".pth"}:
            try:
                import torch
            except ImportError as error:
                raise RuntimeError("TorchScript model selected but Python torch is not installed") from error
            self.backend = "torch"
            self.torch = torch
            self.model = torch.jit.load(str(model_path), map_location="cpu")
            self.model.eval()
            self.input_names = []
        else:
            raise ValueError(f"Unsupported policy extension: {suffix}")

    def forward(self, observation: np.ndarray, history: np.ndarray) -> np.ndarray:
        if self.mode == "single":
            inputs = [observation.reshape(1, -1)]
        elif self.mode == "flat_history":
            inputs = [history.reshape(1, -1)]
        elif self.mode == "obs_history":
            inputs = [observation.reshape(1, -1), history.reshape(1, *history.shape)]
        else:
            raise ValueError(f"Unsupported model_forward_mode: {self.mode}")

        if self.backend == "onnx":
            if len(inputs) != len(self.input_names):
                raise RuntimeError(
                    f"Model exposes {len(self.input_names)} inputs, but mode {self.mode} supplies {len(inputs)}"
                )
            feed = {name: value.astype(np.float32) for name, value in zip(self.input_names, inputs)}
            result = self.model.run(None, feed)[0]
        else:
            tensors = [self.torch.from_numpy(value.astype(np.float32)) for value in inputs]
            with self.torch.inference_mode():
                result = self.model(*tensors)
            if isinstance(result, (tuple, list)):
                result = result[0]
            result = result.detach().cpu().numpy()

        actions = np.asarray(result, dtype=np.float32).reshape(-1)
        if actions.size != ACTION_DIM or not np.all(np.isfinite(actions)):
            raise RuntimeError(f"Policy returned invalid action shape/value: {actions.shape}")
        return actions


def crc32_core(data: bytes) -> int:
    if len(data) % 4:
        raise ValueError("CRC input length must be divisible by four")
    crc = 0xFFFFFFFF
    polynomial = 0x04C11DB7
    for (word,) in struct.iter_unpack("<I", data):
        xbit = 1 << 31
        for _ in range(32):
            crc = ((crc << 1) & 0xFFFFFFFF) ^ (polynomial if crc & 0x80000000 else 0)
            if word & xbit:
                crc ^= polynomial
            xbit >>= 1
    return crc & 0xFFFFFFFF


def lowcmd_crc_bytes(message: Any) -> bytes:
    """Pack the Unitree ABI bytes preceding LowCmd.crc (808 bytes on Go2)."""
    payload = bytearray()
    payload.extend(
        struct.pack(
            "<4B",
            int(message.head[0]),
            int(message.head[1]),
            int(message.level_flag),
            int(message.frame_reserve),
        )
    )
    payload.extend(struct.pack("<2I2IH", *message.sn, *message.version, int(message.bandwidth)))
    payload.extend(b"\x00\x00")
    for motor in message.motor_cmd:
        payload.extend(
            struct.pack(
                "<B3x5f3I",
                int(motor.mode),
                float(motor.q),
                float(motor.dq),
                float(motor.tau),
                float(motor.kp),
                float(motor.kd),
                *[int(value) for value in motor.reserve],
            )
        )
    payload.extend(struct.pack("<B3B", int(message.bms_cmd.off), *message.bms_cmd.reserve))
    payload.extend(bytes(message.wireless_remote))
    payload.extend(bytes(message.led))
    payload.extend(bytes(message.fan))
    payload.extend(struct.pack("<B", int(message.gpio)))
    payload.extend(b"\x00")
    payload.extend(struct.pack("<I", int(message.reserve)))
    if len(payload) != 808:
        raise RuntimeError(f"Packed LowCmd prefix is {len(payload)} bytes, expected 808")
    return bytes(payload)


def set_lowcmd_crc(message: Any) -> None:
    message.crc = crc32_core(lowcmd_crc_bytes(message))


def run_self_test() -> None:
    identity = np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    gravity = quat_rotate_inverse_wxyz(identity, np.asarray([0.0, 0.0, -1.0], dtype=np.float32))
    np.testing.assert_allclose(gravity, [0.0, 0.0, -1.0], atol=1.0e-6)
    config = {
        "default_dof_pos": [0.0] * 12,
        "dof_pos_scale": 1.0,
        "dof_vel_scale": 0.05,
        "ang_vel_scale": 0.2,
        "commands_scale": [1.0, 1.0, 1.0],
        "clip_obs": 100.0,
    }
    observation = build_quadloco_observation(
        np.arange(12, dtype=np.float32),
        np.ones(12, dtype=np.float32),
        np.zeros(12, dtype=np.float32),
        np.ones(3, dtype=np.float32),
        identity,
        np.asarray([1.0, 0.0, 0.0], dtype=np.float32),
        config,
    )
    assert observation.shape == (45,)
    np.testing.assert_allclose(observation[:12], np.arange(12, dtype=np.float32))
    np.testing.assert_allclose(observation[12:24], 0.05)
    np.testing.assert_allclose(observation[36:39], 0.2)
    np.testing.assert_allclose(observation[39:42], [0.0, 0.0, -1.0])
    np.testing.assert_allclose(observation[42:45], [1.0, 0.0, 0.0])
    print("Self-test passed: observation order, scales, gravity, and dimensions are correct.")


def build_parser() -> argparse.ArgumentParser:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy-dir", type=Path, default=repo_root / "policy/go2/policy_history")
    parser.add_argument("--lowstate-topic", default="/lowstate")
    parser.add_argument("--lowcmd-topic", default="/lowcmd")
    parser.add_argument("--policy-rate", type=float, default=50.0)
    parser.add_argument("--lowcmd-rate", type=float, default=200.0)
    parser.add_argument("--state-timeout", type=float, default=0.10)
    parser.add_argument("--max-command-rate", type=float, default=2.0)
    parser.add_argument("--max-target-step", type=float, default=0.10)
    parser.add_argument("--transition-seconds", type=float, default=1.5)
    parser.add_argument("--max-arm-pose-error", type=float, default=0.60)
    parser.add_argument("--publish-lowcmd", action="store_true")
    parser.add_argument("--confirm-lowcmd", default="")
    parser.add_argument("--self-test", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.self_test:
        run_self_test()
        return 0
    if args.publish_lowcmd and args.confirm_lowcmd != LOWCMD_CONFIRMATION:
        print(
            "Refusing live LowCmd output. Supply --confirm-lowcmd " + LOWCMD_CONFIRMATION,
            file=sys.stderr,
        )
        return 2

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Float32MultiArray
        from std_srvs.srv import SetBool
        from unitree_go.msg import LowCmd, LowState
    except ImportError as error:
        print(
            "ROS 2 imports failed. Source ROS and the workspace containing unitree_go first: "
            f"{error}",
            file=sys.stderr,
        )
        return 2

    config, model_path = load_policy_config(args.policy_dir.resolve())
    history_indices = [int(value) for value in config.get("observations_history", [])]
    mode = str(config.get("model_forward_mode", "single"))
    if mode == "single" and history_indices:
        mode = "flat_history"
    if mode != "single" and not history_indices:
        raise ValueError(f"model_forward_mode={mode} requires observations_history")
    model = PolicyModel(model_path, mode)

    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    class QuadlocoPolicyNode(Node):
        def __init__(self) -> None:
            super().__init__("quadloco_ros2_policy")
            self.config = config
            self.mapping = np.asarray(config["joint_mapping"], dtype=np.int64)
            self.default_q = np.asarray(config["default_dof_pos"], dtype=np.float32)
            self.action_scale = np.asarray(config["action_scale"], dtype=np.float32)
            self.kp = np.asarray(config["rl_kp"], dtype=np.float32)
            self.kd = np.asarray(config["rl_kd"], dtype=np.float32)
            self.lower = np.asarray(config.get("joint_pos_limit_lower", [-math.inf] * 12), dtype=np.float32)
            self.upper = np.asarray(config.get("joint_pos_limit_upper", [math.inf] * 12), dtype=np.float32)
            self.action_lower = np.asarray(config.get("clip_actions_lower", [-math.inf] * 12), dtype=np.float32)
            self.action_upper = np.asarray(config.get("clip_actions_upper", [math.inf] * 12), dtype=np.float32)
            self.history = HistoryBuffer(history_indices, bool(config.get("warm_start_history", False)))
            self.previous_actions = np.zeros(ACTION_DIM, dtype=np.float32)

            if HARDCODED_COMMAND.shape != (3,):
                raise ValueError("HARDCODED_COMMAND must have exactly 3 values: [vx, vy, yaw]")
            if not np.all(np.isfinite(HARDCODED_COMMAND)):
                raise ValueError("HARDCODED_COMMAND contains NaN or infinity")
            if np.any(np.abs(HARDCODED_COMMAND) > HARDCODED_COMMAND_LIMITS):
                raise ValueError(
                    "HARDCODED_COMMAND exceeds limits "
                    f"{HARDCODED_COMMAND_LIMITS.tolist()}: {HARDCODED_COMMAND.tolist()}"
                )

            self.fixed_command = HARDCODED_COMMAND.copy()
            self.command_target = np.zeros(3, dtype=np.float32)
            self.command = np.zeros(3, dtype=np.float32)
            self.lowstate: Any | None = None
            self.last_lowstate_time = -math.inf
            self.last_policy_time = time.monotonic()
            self.last_action_print_time = -math.inf
            self.latest_target = self.default_q.copy()
            self.arm_start_target = self.default_q.copy()
            self.arm_time = 0.0
            self.armed = False
            self.fault_reason = ""

            self.create_subscription(LowState, args.lowstate_topic, self._lowstate_callback, qos)
            self.observation_publisher = self.create_publisher(
                Float32MultiArray, "/quadloco_policy/observation", 10
            )
            self.action_publisher = self.create_publisher(
                Float32MultiArray, "/quadloco_policy/action", 10
            )
            self.lowcmd_publisher = (
                self.create_publisher(LowCmd, args.lowcmd_topic, 10) if args.publish_lowcmd else None
            )
            self.arm_service = self.create_service(SetBool, "/quadloco_policy/arm", self._arm_callback)
            self.create_timer(1.0 / args.policy_rate, self._policy_step)
            if self.lowcmd_publisher is not None:
                self.create_timer(1.0 / args.lowcmd_rate, self._publish_lowcmd)

            mode_text = "LIVE-CAPABLE, DISARMED" if args.publish_lowcmd else "DRY-RUN"
            self.get_logger().info(
                f"Loaded {model_path} ({model.backend}, {mode}); mode={mode_text}; "
                f"state={args.lowstate_topic}; hardcoded_command={self.fixed_command.tolist()}"
            )

        def _lowstate_callback(self, message: Any) -> None:
            self.lowstate = message
            self.last_lowstate_time = time.monotonic()

        def _arm_callback(self, request: Any, response: Any) -> Any:
            if not request.data:
                self.armed = False
                self.command.fill(0.0)
                self.command_target.fill(0.0)
                response.success = True
                response.message = "LowCmd output disarmed; no further motor messages will be published"
                return response
            if self.lowcmd_publisher is None:
                response.success = False
                response.message = "Node was launched in dry-run mode"
                return response
            now = time.monotonic()
            if self.lowstate is None or now - self.last_lowstate_time > args.state_timeout:
                response.success = False
                response.message = "Cannot arm: /lowstate is missing or stale"
                return response
            q_policy, _ = self._joint_state_policy_order()
            pose_error = float(np.max(np.abs(q_policy - self.default_q)))
            if pose_error > args.max_arm_pose_error:
                response.success = False
                response.message = f"Cannot arm: pose error {pose_error:.3f} rad exceeds limit"
                return response
            if self.count_publishers(args.lowcmd_topic) > 1:
                response.success = False
                response.message = "Cannot arm: another ROS publisher is already writing /lowcmd"
                return response
            self.arm_start_target = q_policy.copy()
            self.latest_target = q_policy.copy()
            self.arm_time = now
            self.armed = True
            response.success = True
            response.message = "LowCmd output ARMED"
            return response

        def _joint_state_policy_order(self) -> tuple[np.ndarray, np.ndarray]:
            q_hardware = np.asarray([item.q for item in self.lowstate.motor_state[:12]], dtype=np.float32)
            dq_hardware = np.asarray([item.dq for item in self.lowstate.motor_state[:12]], dtype=np.float32)
            return q_hardware[self.mapping], dq_hardware[self.mapping]

        def _disarm_fault(self, reason: str) -> None:
            if self.armed:
                self.get_logger().error(f"Disarming LowCmd output: {reason}")
            self.armed = False
            self.fault_reason = reason
            self.command.fill(0.0)
            self.command_target.fill(0.0)

        def _policy_step(self) -> None:
            now = time.monotonic()
            dt = max(now - self.last_policy_time, 1.0 / args.policy_rate)
            self.last_policy_time = now
            if self.lowstate is None or now - self.last_lowstate_time > args.state_timeout:
                self._disarm_fault("stale /lowstate")
                return
            # Dry-run: always evaluate the hardcoded command.
            # Live mode: keep command at zero until the node has been explicitly armed.
            if self.lowcmd_publisher is None or self.armed:
                self.command_target[:] = self.fixed_command
            else:
                self.command_target.fill(0.0)

            max_delta = args.max_command_rate * dt
            self.command += np.clip(self.command_target - self.command, -max_delta, max_delta)
            q_policy, dq_policy = self._joint_state_policy_order()
            imu = self.lowstate.imu_state
            observation = build_quadloco_observation(
                q_policy,
                dq_policy,
                self.previous_actions,
                np.asarray(imu.gyroscope, dtype=np.float32),
                np.asarray(imu.quaternion, dtype=np.float32),
                self.command,
                self.config,
            )
            history = self.history.insert_and_select(observation)
            try:
                raw_actions = model.forward(observation, history)
            except Exception as error:
                self._disarm_fault(f"inference failed: {error}")
                return

            actions = np.clip(raw_actions, self.action_lower, self.action_upper)

            if ACTION_PRINT_RATE_HZ > 0.0:
                print_period = 1.0 / ACTION_PRINT_RATE_HZ
                if now - self.last_action_print_time >= print_period:
                    print(
                        "cmd="
                        + np.array2string(self.command, precision=3, suppress_small=True)
                        + " raw_actions="
                        + np.array2string(raw_actions, precision=4, suppress_small=True)
                        + " clipped_actions="
                        + np.array2string(actions, precision=4, suppress_small=True),
                        flush=True,
                    )
                    self.last_action_print_time = now
            policy_target = np.clip(self.default_q + actions * self.action_scale, self.lower, self.upper)
            target_delta = np.clip(
                policy_target - self.latest_target,
                -args.max_target_step,
                args.max_target_step,
            )
            self.latest_target = self.latest_target + target_delta
            self.previous_actions = actions

            observation_message = Float32MultiArray(data=observation.tolist())
            action_message = Float32MultiArray(data=actions.tolist())
            self.observation_publisher.publish(observation_message)
            self.action_publisher.publish(action_message)

        def _publish_lowcmd(self) -> None:
            if not self.armed or self.lowcmd_publisher is None or self.lowstate is None:
                return
            now = time.monotonic()
            if now - self.last_lowstate_time > args.state_timeout:
                self._disarm_fault("stale /lowstate during LowCmd publication")
                return
            blend = min(max((now - self.arm_time) / args.transition_seconds, 0.0), 1.0)
            target = (1.0 - blend) * self.arm_start_target + blend * self.latest_target

            message = LowCmd()
            message.head = [0xFE, 0xEF]
            message.level_flag = 0xFF
            for motor in message.motor_cmd:
                motor.mode = 0x01
                motor.q = POS_STOP_F
                motor.dq = VEL_STOP_F
                motor.kp = 0.0
                motor.kd = 0.0
                motor.tau = 0.0
            for policy_index, hardware_index in enumerate(self.mapping):
                motor = message.motor_cmd[int(hardware_index)]
                motor.mode = 0x01
                motor.q = float(target[policy_index])
                motor.dq = 0.0
                motor.kp = float(self.kp[policy_index])
                motor.kd = float(self.kd[policy_index])
                motor.tau = 0.0
            set_lowcmd_crc(message)
            self.lowcmd_publisher.publish(message)

    rclpy.init()
    node = QuadlocoPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.armed = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
