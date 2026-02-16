"""RoboSim - Research-Grade Robot Simulation Engine"""

__version__ = "1.0.0"

from robosim.env import RoboSimEnv, make

try:
    from _robosim_core import (
        Environment,
        EnvPool,
        BatchedState,
        OnnxInference,
        StateSnapshot,
        available_robots,
        RobotInfo,
    )
except ImportError:
    print("Warning: _robosim_core C++ module not found. Build with -DROBOSIM_BUILD_PYTHON=ON")

__all__ = [
    "RoboSimEnv",
    "make",
    "Environment",
    "EnvPool",
    "BatchedState",
    "OnnxInference",
    "StateSnapshot",
    "available_robots",
]
