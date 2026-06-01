"""
Play a trained Bike policy with mjlab viewers.

Examples:
    python tools/play_bike.py --checkpoint tools/logs/bike/log5/model_1799.pt
    python tools/play_bike.py --checkpoint tools/logs/bike/test/model_999.pt --viewer viser
    python tools/play_bike.py --agent random --viewer native
    python tools/play_bike.py --agent zero --viewer headless --steps 500

    python rl/tasks/play.py --checkpoint rl/tasks/logs/bike_balance/log4/model_1999.pt
"""

from __future__ import annotations

import argparse
import numpy as np
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import torch


def load_policy(checkpoint_path: str, obs_dim: int, act_dim: int, device: str):
    import torch
    from rsl_rl.models.mlp_model import MLPModel

    dummy_obs = {"actor": torch.zeros(1, obs_dim, device=device)}
    obs_groups = {"actor": ["actor"]}

    actor = MLPModel(
        obs=dummy_obs,
        obs_groups=obs_groups,
        obs_set="actor",
        output_dim=act_dim,
        hidden_dims=(128, 128),
        activation="elu",
        distribution_cfg={
            "class_name": "rsl_rl.modules.distribution.GaussianDistribution",
            "init_std": 1.0,
            "std_type": "scalar",
        },
    ).to(device)

    checkpoint = torch.load(checkpoint_path, map_location=device)
    actor.load_state_dict(checkpoint["actor_state_dict"])
    actor.eval()

    print(f"[play] Loaded checkpoint: {checkpoint_path}")
    print(f"[play] Iteration: {checkpoint.get('iter', '?')}")
    return actor


def get_action(agent, obs_dict: dict) -> torch.Tensor:
    import torch

    if hasattr(agent, "act"):
        return agent.act(obs_dict["actor"])

    with torch.no_grad():
        return agent(obs_dict)


class PolicyWrapper:
    def __init__(self, agent):
        self.agent = agent

    def __call__(self, obs: torch.Tensor) -> torch.Tensor:
        import torch

        if hasattr(self.agent, "act"):
            return self.agent.act(obs)

        with torch.no_grad():
            return self.agent({"actor": obs})


class EnvWrapper:
    def __init__(self, env):
        self.env = env
        self._obs_dict = None
        self.num_envs = env.num_envs
        try:
            self.max_episode_length = env.max_episode_length
        except (OverflowError, ValueError):
            self.max_episode_length = 999999

    def __getattr__(self, name: str):
        return getattr(self.env, name)

    def reset(self):
        self._obs_dict, _ = self.env.reset()
        return self._obs_dict["actor"]

    def get_observations(self) -> torch.Tensor:
        if self._obs_dict is None:
            self.reset()

        return self._obs_dict["actor"]

    def step(self, actions: torch.Tensor):
        obs_np    = self._obs_dict["actor"][0].cpu().numpy()
        action_np = actions[0].cpu().numpy()
        print(f"[obs: {np.rad2deg(obs_np[1])} | action: {action_np}")

        self._obs_dict, _, terminated, truncated, _ = self.env.step(actions)
        if terminated.any() or truncated.any():
            self._obs_dict, _ = self.env.reset()

    def close(self):
        self.env.close()


class ZeroAgent:
    def __init__(self, act_dim: int):
        self.act_dim = act_dim

    def act(self, obs: torch.Tensor):
        import torch

        return torch.zeros(obs.shape[0], self.act_dim, device=obs.device)


class RandomAgent:
    def __init__(self, act_dim: int):
        self.act_dim = act_dim

    def act(self, obs: torch.Tensor):
        import torch

        return torch.rand(obs.shape[0], self.act_dim, device=obs.device) * 2.0 - 1.0


def run_play(args):
    from tasks.bike_balance.env_cfg import bike_balance_env_cfg
    from mjlab.envs import ManagerBasedRlEnv

    device = f"cuda:{args.gpu_id}" if args.gpu_id >= 0 else "cpu"

    env_cfg = bike_balance_env_cfg(num_envs=args.num_envs)
    env_cfg.episode_length_s = 9999.0

    env = ManagerBasedRlEnv(cfg=env_cfg, device=device)

    obs_dim = env.observation_space.spaces["actor"].shape[-1]
    act_dim = env.action_space.shape[-1]
    print(f"[play] Observation dim: {obs_dim}")
    print(f"[play] Action dim     : {act_dim}")
    print(f"[play] Num envs       : {args.num_envs}")
    print(f"[play] Device         : {device}")

    if args.agent == "zero":
        agent = ZeroAgent(act_dim)
        print("[play] Agent          : zero")
    elif args.agent == "random":
        agent = RandomAgent(act_dim)
        print("[play] Agent          : random")
    else:
        agent = load_policy(args.checkpoint, obs_dim, act_dim, device)
        print("[play] Agent          : policy")

    try:
        if args.viewer == "native":
            _run_with_native_viewer(env, agent)
        elif args.viewer == "viser":
            _run_with_viser(env, agent)
        else:
            _run_headless(env, agent, steps=args.steps)
    finally:
        env.close()


def _run_with_native_viewer(env, agent):
    from mjlab.viewer import NativeMujocoViewer

    print("[play] Starting Native Viewer. Press Ctrl+C or Q to quit.")
    wrapped_env = EnvWrapper(env)
    wrapped_env.reset()
    viewer = NativeMujocoViewer(wrapped_env, PolicyWrapper(agent))
    viewer.run()


def _run_with_viser(env, agent):
    from mjlab.viewer import ViserPlayViewer

    print("[play] Starting Viser Viewer.")
    print("[play] Open http://localhost:8080 in your browser.")
    wrapped_env = EnvWrapper(env)
    wrapped_env.reset()
    viewer = ViserPlayViewer(wrapped_env, PolicyWrapper(agent))
    viewer.run()


def _run_headless(env, agent, steps: int):
    print(f"[play] Running headless for {steps} steps.")
    obs_dict, _ = env.reset()

    total_reward = 0.0
    for step in range(steps):
        action = get_action(agent, obs_dict)
        obs_dict, reward, terminated, truncated, _ = env.step(action)
        total_reward += reward.mean().item()

        if step % 100 == 0:
            print(
                f"  Step {step:5d} | "
                f"reward_mean = {reward.mean().item(): .4f} | "
                f"total_mean = {total_reward: .2f}"
            )

        if terminated.any() or truncated.any():
            obs_dict, _ = env.reset()

    print(f"[play] Done. Total mean reward: {total_reward:.2f}")


def main():
    parser = argparse.ArgumentParser(description="Bike mjlab play/viewer")
    parser.add_argument(
        "--checkpoint",
        type=str,
        default="",
        help="Path to a trained .pt checkpoint.",
    )
    parser.add_argument(
        "--agent",
        choices=["policy", "zero", "random"],
        default="policy",
        help="Agent type to run.",
    )
    parser.add_argument(
        "--viewer",
        choices=["native", "viser", "headless"],
        default="native",
        help="Viewer backend.",
    )
    parser.add_argument(
        "--num-envs",
        type=int,
        default=1,
        help="Number of environments to create for playback.",
    )
    parser.add_argument(
        "--gpu-id",
        type=int,
        default=0,
        help="GPU id. Use -1 for CPU.",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=500,
        help="Number of steps for headless playback.",
    )
    args = parser.parse_args()

    if args.agent == "policy" and not args.checkpoint:
        print("[play] Error: --checkpoint is required when --agent policy.")
        print("[play] Hint: use --agent random or --agent zero for quick checks.")
        return

    run_play(args)


if __name__ == "__main__":
    main()
