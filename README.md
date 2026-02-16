# RoboSim

Research-grade robot simulation engine built with C++17. Provides Bullet Physics-based multibody dynamics, OpenGL 3D visualization, and a Gymnasium-compatible Python API.

## Features

- **16 Robot Models** - Quadruped (Go2, Spot, ANYmal C, CyberDog2), Humanoid (Atlas, Digit, Optimus, Figure01), Robot Arm (Panda, UR5e, iiwa14), Drone (Quadrotor, Hexarotor), Mobile Robot (Differential Drive, Ackermann)
- **Bullet Physics** - Featherstone multibody dynamics via btMultiBody, contact and collision handling
- **Multiple Actuator Models** - Position PD, Velocity PI, Direct Torque, SEA, DC Motor
- **Terrain System** - Flat, Stairs, Slope, Rough (Perlin noise), Stepping Stones
- **Sensors** - IMU, Joint Encoder, Contact Sensor, LiDAR (raycast-based)
- **Task/Reward Framework** - WalkForward, FollowPath, ReachTarget, PickAndPlace
- **Domain Randomization** - Randomize ground friction, gravity, and other parameters
- **Vectorized Environment** - ThreadPool-based parallel environments (EnvPool)
- **Python Bindings** - pybind11, Gymnasium-compatible `RoboSimEnv`
- **ONNX Runtime** - Trained policy inference (optional)
- **Checkpoint** - Save/restore simulation state

## Dependencies

| Library | Purpose | Notes |
|---------|---------|-------|
| Eigen3 | Linear algebra | Auto-downloaded (FetchContent) |
| Bullet3 | Physics engine | Auto-downloaded |
| yaml-cpp | YAML config parsing | Auto-downloaded |
| spdlog | Logging | Auto-downloaded |
| GLFW + glad + ImGui | GUI rendering | Auto-downloaded (optional) |
| pybind11 | Python bindings | `pip install pybind11` (optional) |
| ONNX Runtime | Neural network inference | System install required (optional) |

## Build

### System Requirements

- Ubuntu 20.04+ (or WSL2)
- CMake 3.20+
- GCC 9+ or Clang 10+ (C++17 support)
- Python 3.8+ (for glad generation and Python bindings)

### Install Build Tools

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3 python3-pip
```

### Additional Packages for GUI Build

```bash
sudo apt install -y libgl1-mesa-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
```

### Build

```bash
cd ~/coding/robosim
mkdir -p build && cd build

# Default build (with GUI)
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Build without GUI (server/CI environments)
cmake .. -DCMAKE_BUILD_TYPE=Release -DROBOSIM_BUILD_GUI=OFF
make -j$(nproc)

# With Python bindings
pip install pybind11
cmake .. -DCMAKE_BUILD_TYPE=Release -DROBOSIM_BUILD_PYTHON=ON
make -j$(nproc)

# With ONNX Runtime
sudo apt install libonnxruntime-dev
cmake .. -DCMAKE_BUILD_TYPE=Release -DROBOSIM_USE_ONNX=ON
make -j$(nproc)
```

## Usage

### CLI Options

```
Usage: robosim [options]
  --robot <name>    Select robot (e.g., unitree_go2)
  --task <name>     Select task (walk_forward, follow_path, reach_target, pick_and_place)
  --headless        Run without GUI
  --steps <N>       Maximum simulation steps (headless mode)
  --config <file>   Load YAML configuration file
  --list            Print available robot list
  --help            Show help
```

### GUI Mode

```bash
# Launch with robot selection UI
./robosim

# Launch directly with a specific robot
./robosim --robot unitree_go2
```

GUI features:
- Robot selection panel
- Joint manual control sliders
- Simulation play/pause/reset
- Physics parameter adjustment (timestep, gravity)
- Terrain type switching
- Domain Randomization settings
- Real-time performance metrics (FPS, physics Hz)

Camera controls:
- Left-click drag: Rotate
- Middle-button drag: Pan
- Scroll: Zoom

### Headless Mode

```bash
# Basic execution
./robosim --headless --robot unitree_go2 --task walk_forward --steps 10000

# Other robots/tasks
./robosim --headless --robot franka_panda --task reach_target --steps 5000
./robosim --headless --robot boston_dynamics_spot --task follow_path
```

### Python API

```python
import robosim

# List available robots
robots = robosim.available_robots()
for r in robots:
    print(f"{r.id}: {r.display_name} ({r.num_joints} DOF)")

# Single environment
env = robosim.make("Go2-Walk-v0")
obs, info = env.reset(seed=42)

for step in range(1000):
    action = env.action_space.sample()  # or policy output
    obs, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        obs, info = env.reset()

env.close()
```

### Vectorized Environment (Parallel Simulation)

```python
import robosim
import numpy as np

# Run 16 environments in parallel (4 threads)
pool = robosim.EnvPool("unitree_go2", "walk_forward", num_envs=16, num_threads=4)
state = pool.reset()

for step in range(1000):
    actions = np.random.uniform(-0.3, 0.3, size=(16, pool.action_dim))
    state = pool.step(actions)
    # state.observations: [16 x obs_dim]
    # state.rewards: [16]
    # state.dones: [16]
```

### YAML Configuration

Simulation parameters can be configured in `config/default.yaml`:

```yaml
simulation:
  timestep: 0.002          # Physics simulation timestep (500Hz)
  control_decimation: 4     # Control period = timestep * decimation (125Hz)
  max_episode_steps: 1000
  gravity: [0.0, 0.0, -9.81]

robot:
  name: "unitree_go2"

task:
  name: "walk_forward"
  target_velocity: [1.0, 0.0, 0.0]
  reward:
    terms:
      - name: "linear_velocity"
        weight: 1.0
      - name: "orientation"
        weight: 0.5
      - name: "torque_penalty"
        weight: -0.0001

terrain:
  type: "flat"              # flat, stairs, slope, rough, stepping_stones
  friction: 1.0

domain_randomization:
  enabled: false
```

## Available Robots

| ID | Name | Category | DOF | Weight |
|----|------|----------|-----|--------|
| `unitree_go2` | Unitree Go2 | Quadruped | 12 | 15 kg |
| `boston_dynamics_spot` | Boston Dynamics Spot | Quadruped | 12 | 32 kg |
| `anybotics_anymal_c` | ANYbotics ANYmal C | Quadruped | 12 | 50 kg |
| `xiaomi_cyberdog2` | Xiaomi CyberDog 2 | Quadruped | 12 | 8.9 kg |
| `boston_dynamics_atlas` | Boston Dynamics Atlas | Humanoid | 23 | 89 kg |
| `agility_digit` | Agility Digit | Humanoid | 20 | 42 kg |
| `tesla_optimus` | Tesla Optimus | Humanoid | 22 | 57 kg |
| `figure_01` | Figure 01 | Humanoid | 20 | 60 kg |
| `franka_panda` | Franka Panda | Arm | 7+2 | 18 kg |
| `universal_robots_ur5e` | Universal Robots UR5e | Arm | 6 | 20 kg |
| `kuka_iiwa14` | KUKA iiwa 14 | Arm | 7 | 30 kg |
| `generic_quadrotor` | Generic Quadrotor | Drone | 4 | 1.5 kg |
| `dji_hexarotor` | DJI Hexarotor | Drone | 6 | 10 kg |
| `differential_drive` | Differential Drive | Mobile | 2 | 5 kg |
| `ackermann_robot` | Ackermann Robot | Mobile | 2+1 | 15 kg |

## Project Structure

```
robosim/
├── CMakeLists.txt
├── cmake/
│   └── dependencies.cmake          # FetchContent dependency management
├── config/
│   └── default.yaml                # Default simulation settings
├── include/robosim/
│   ├── config/                     # YAML config parsing
│   ├── core/                       # Robot, Joint, Link, Actuator, World, Simulator
│   ├── physics/                    # BulletWorld (Bullet Physics integration)
│   ├── robot/definitions/          # Per-robot kinematic definitions
│   ├── render/                     # OpenGL renderer, Camera, Shader, Mesh, GUI
│   ├── sensor/                     # IMU, JointEncoder, ContactSensor, LiDAR
│   ├── reward/                     # RewardFunction, CompositeReward, reward terms
│   ├── terrain/                    # Heightfield, Terrain generators
│   ├── task/                       # Task interface, WalkForward, etc.
│   ├── env/                        # Environment (Gymnasium-compatible)
│   ├── randomization/              # Domain Randomization
│   ├── parallel/                   # ThreadPool, EnvPool
│   ├── checkpoint/                 # State save/restore
│   ├── logging/                    # DataLogger (CSV)
│   └── ai/                         # ONNX Runtime inference
├── src/                            # Implementation sources (.cpp)
├── shaders/                        # GLSL shaders (basic, grid)
├── python/
│   ├── robosim_bindings.cpp        # pybind11 bindings
│   └── robosim/
│       ├── __init__.py
│       └── env.py                  # Gymnasium wrapper
└── examples/
    ├── basic_sim.cpp               # C++ simulation example
    └── train_ppo.py                # PPO training example
```

## Architecture

```
┌─────────────────────────────────────────────────┐
│                   Environment                    │
│  (Gymnasium-compatible step/reset/obs/reward)    │
├──────────┬──────────┬───────────┬───────────────┤
│ Simulator│  Task    │  Terrain  │   Sensors     │
│          │          │           │ (IMU, Encoder, │
│          │          │           │  Contact, LiDAR)│
├──────────┴──────────┴───────────┴───────────────┤
│              BulletWorld (Physics)                │
│  btMultiBody · Collision · Raycast · Heightfield │
├─────────────────────────────────────────────────┤
│                Robot (Core)                      │
│  Joints · Links · Actuators · Contacts · FK     │
├─────────────────────────────────────────────────┤
│    RobotFactory (16 built-in definitions)        │
└─────────────────────────────────────────────────┘
         ↕                    ↕
  ┌──────────────┐   ┌───────────────┐
  │ OpenGL GUI   │   │ Python API    │
  │ ImGui panels │   │ pybind11      │
  │ Camera/Shader│   │ Gymnasium     │
  └──────────────┘   └───────────────┘
```

## License

Research use only. All robot names and trademarks belong to their respective owners.
