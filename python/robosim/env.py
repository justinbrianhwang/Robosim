"""Gymnasium-compatible wrapper for RoboSim environments."""

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
    HAS_GYMNASIUM = True
except ImportError:
    HAS_GYMNASIUM = False

try:
    from _robosim_core import Environment as _CppEnv, EnvPool as _CppEnvPool
    HAS_CORE = True
except ImportError:
    HAS_CORE = False


def make(env_id: str, **kwargs):
    """Create a RoboSim environment.

    Args:
        env_id: Format "RobotName-TaskName-v0", e.g. "Go2-Walk-v0"
        **kwargs: Additional arguments passed to the environment.

    Returns:
        RoboSimEnv instance.
    """
    # Parse env_id
    robot_map = {
        "Go2": "unitree_go2",
        "Spot": "boston_dynamics_spot",
        "ANYmal": "anybotics_anymal_c",
        "CyberDog": "xiaomi_cyberdog2",
        "Atlas": "boston_dynamics_atlas",
        "Digit": "agility_digit",
        "Optimus": "tesla_optimus",
        "Figure01": "figure_01",
        "Panda": "franka_panda",
        "UR5e": "universal_robots_ur5e",
        "iiwa": "kuka_iiwa14",
        "Quadrotor": "generic_quadrotor",
        "Hexarotor": "dji_hexarotor",
        "DiffDrive": "differential_drive",
        "Ackermann": "ackermann_robot",
    }

    task_map = {
        "Walk": "walk_forward",
        "FollowPath": "follow_path",
        "Reach": "reach_target",
        "PickPlace": "pick_and_place",
    }

    parts = env_id.replace("-v0", "").split("-")
    robot_key = parts[0] if len(parts) > 0 else "Go2"
    task_key = parts[1] if len(parts) > 1 else "Walk"

    robot_name = robot_map.get(robot_key, robot_key)
    task_name = task_map.get(task_key, task_key)

    return RoboSimEnv(robot_name=robot_name, task_name=task_name, **kwargs)


if HAS_GYMNASIUM and HAS_CORE:
    class RoboSimEnv(gym.Env):
        """Gymnasium-compatible RoboSim environment."""

        metadata = {"render_modes": ["human", "rgb_array"]}

        def __init__(self, robot_name: str = "unitree_go2",
                     task_name: str = "walk_forward",
                     render_mode=None, **kwargs):
            super().__init__()
            self._env = _CppEnv(robot_name, task_name, headless=(render_mode is None))
            self.render_mode = render_mode

            obs_dim = self._env.observation_dim
            act_dim = self._env.action_dim

            self.observation_space = spaces.Box(
                low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float64
            )
            self.action_space = spaces.Box(
                low=-1.0, high=1.0, shape=(act_dim,), dtype=np.float64
            )

        def step(self, action):
            action = np.asarray(action, dtype=np.float64)
            obs, reward, terminated, truncated, info = self._env.step(action)
            return np.array(obs), float(reward), bool(terminated), bool(truncated), dict(info)

        def reset(self, *, seed=None, options=None):
            obs, info = self._env.reset(seed)
            return np.array(obs), dict(info)

        def close(self):
            self._env.close()

        def render(self):
            pass  # Rendering handled internally

elif HAS_CORE:
    class RoboSimEnv:
        """Simplified RoboSim environment (gymnasium not installed)."""

        def __init__(self, robot_name="unitree_go2", task_name="walk_forward", **kwargs):
            self._env = _CppEnv(robot_name, task_name, headless=True)

        def step(self, action):
            return self._env.step(np.asarray(action, dtype=np.float64))

        def reset(self, seed=None):
            return self._env.reset(seed)

        def close(self):
            self._env.close()

        @property
        def observation_dim(self):
            return self._env.observation_dim

        @property
        def action_dim(self):
            return self._env.action_dim

else:
    class RoboSimEnv:
        def __init__(self, **kwargs):
            raise ImportError("RoboSim C++ core not built. Build with -DROBOSIM_BUILD_PYTHON=ON")
