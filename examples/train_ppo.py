#!/usr/bin/env python3
"""
RoboSim PPO Training Example

Demonstrates how to train a locomotion policy using PPO
with the RoboSim vectorized environment.

Requirements:
    pip install torch gymnasium numpy
    Build robosim with: cmake -DROBOSIM_BUILD_PYTHON=ON .. && make
"""

import numpy as np
import time

try:
    import robosim
    print(f"RoboSim v{robosim.__version__}")
except ImportError:
    print("Error: RoboSim not built with Python bindings.")
    print("Build with: cmake -DROBOSIM_BUILD_PYTHON=ON .. && make")
    exit(1)

# List available robots
robots = robosim.available_robots()
print(f"\nAvailable robots ({len(robots)}):")
for r in robots:
    print(f"  {r.id}: {r.display_name} ({r.num_joints} DOF)")

# Create environment
print("\n--- Creating Go2 Walk Environment ---")
env = robosim.make("Go2-Walk-v0")
obs, info = env.reset(seed=42)
print(f"Observation dim: {env.observation_dim}")
print(f"Action dim: {env.action_dim}")

# Simple random policy evaluation
print("\n--- Random Policy Evaluation ---")
num_episodes = 10
total_rewards = []

for ep in range(num_episodes):
    obs, info = env.reset(seed=ep)
    episode_reward = 0.0
    done = False
    steps = 0

    while not done and steps < 1000:
        action = np.random.uniform(-0.3, 0.3, size=(env.action_dim,))
        obs, reward, terminated, truncated, info = env.step(action)
        episode_reward += reward
        done = terminated or truncated
        steps += 1

    total_rewards.append(episode_reward)
    print(f"  Episode {ep+1}: steps={steps}, reward={episode_reward:.3f}")

print(f"\nMean reward: {np.mean(total_rewards):.3f} +/- {np.std(total_rewards):.3f}")

# Vectorized environment benchmark
print("\n--- Vectorized Environment Benchmark ---")
try:
    num_envs = 16
    pool = robosim.EnvPool("unitree_go2", "walk_forward", num_envs=num_envs, num_threads=4)
    state = pool.reset()
    print(f"EnvPool created: {num_envs} environments")
    print(f"Batch observation shape: {state.observations.shape}")

    num_steps = 1000
    start_time = time.time()

    for step in range(num_steps):
        actions = np.random.uniform(-0.3, 0.3, size=(num_envs, pool.action_dim))
        state = pool.step(actions)

    elapsed = time.time() - start_time
    total_steps = num_steps * num_envs
    fps = total_steps / elapsed

    print(f"Benchmark: {total_steps} steps in {elapsed:.2f}s = {fps:.0f} steps/sec")

except Exception as e:
    print(f"EnvPool benchmark failed: {e}")

env.close()
print("\nDone!")
