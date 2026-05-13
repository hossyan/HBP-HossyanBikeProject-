"""
play.py
========
学習済みモデルをロードして Cartpole の動作を可視化するスクリプト。

mjlab には 2 種類のビューアがある:
  - Native Viewer: MuJoCo 標準の 3D ビューア（mjlab が GPU→CPU に同期して表示）
  - Viser         : ブラウザベースの 3D ビューア（WebGL）

使い方:
    # 最新チェックポイントを Native Viewer で表示
    python play.py --checkpoint logs/cartpole/model_499.pt

    # Viser（ブラウザ）で表示
    python play.py --checkpoint logs/cartpole/model_499.pt --viewer viser

    # ランダムエージェント（学習前動作確認用）
    python play.py --agent random

    # ゼロアクションエージェント（初期状態確認用）
    python play.py --agent zero

    # スウィングアップ課題
    python play.py --checkpoint <path> --task swingup
"""

import argparse
import time
from pathlib import Path

import torch


# ============================================================
# ユーティリティ: チェックポイントのロード
# ============================================================

def load_policy(checkpoint_path: str, obs_dim: int, act_dim: int, device: str):
    """
    保存されたモデルから Actor ネットワークを復元する。

    チェックポイントのキー構成:
        actor_state_dict   : Actor の重み
        critic_state_dict  : Critic の重み
        optimizer_state_dict: Adam のモメンタム等
        iter               : 学習イテレーション数
        infos              : その他の情報

    actor_state_dict のキー構成:
        distribution.std_param : GaussianDistribution の標準偏差
        mlp.0.weight / mlp.0.bias : 1層目 Linear (obs_dim → 64)
        mlp.2.weight / mlp.2.bias : 2層目 Linear (64 → 64)
        mlp.4.weight / mlp.4.bias : 3層目 Linear (64 → act_dim)
    """
    from rsl_rl.models.mlp_model import MLPModel

    # MLPModel の初期化に必要なダミー観測を用意する
    dummy_obs = {"actor": torch.zeros(1, obs_dim, device=device)}
    obs_groups = {"actor": ["actor"]}

    actor = MLPModel(
        obs=dummy_obs,
        obs_groups=obs_groups,
        obs_set="actor",
        output_dim=act_dim,
        hidden_dims=(64, 64),
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

    print(f"[play] チェックポイントをロード: {checkpoint_path}")
    print(f"[play] 学習イテレーション: {checkpoint.get('iter', '?')}")

    return actor


# ============================================================
# 推論ヘルパー
# ============================================================

def get_action(agent, obs_dict: dict, device: str) -> torch.Tensor:
    """
    エージェントの種類に応じてアクションを返す。

    - ZeroAgent / RandomAgent : obs_dict["actor"] を渡して act() を呼ぶ
    - MLPModel (学習済み)     : obs_dict をそのまま forward() に渡す
                                （MLPModel は TensorDict 形式を期待する）
    """
    if hasattr(agent, "act"):
        # ZeroAgent / RandomAgent
        obs = obs_dict["actor"]
        return agent.act(obs)
    else:
        # MLPModel: forward() は決定的出力（平均値）を返す
        with torch.no_grad():
            return agent(obs_dict)


# ============================================================
# ダミーエージェント（動作確認用）
# ============================================================

class PolicyWrapper:
    """
    NativeMujocoViewer が要求する PolicyProtocol に合わせたラッパー。
    __call__(obs: Tensor) -> Tensor の形で呼ばれる。
    """
    def __init__(self, agent, device: str):
        self.agent = agent
        self.device = device

    def __call__(self, obs: torch.Tensor) -> torch.Tensor:
        if hasattr(self.agent, "act"):
            # ZeroAgent / RandomAgent
            return self.agent.act(obs)
        else:
            # MLPModel: {"actor": obs} の形で渡す
            obs_dict = {"actor": obs}
            with torch.no_grad():
                return self.agent(obs_dict)

class EnvWrapper:
    """
    NativeMujocoViewer が要求する EnvProtocol に合わせたラッパー。
    未定義の属性は元の env に自動委譲する。
    """
    def __init__(self, env, device: str):
        self.env = env
        self.device = device
        self._obs_dict = None
        self.num_envs = env.num_envs
        try:
            self.max_episode_length = env.max_episode_length
        except (OverflowError, ValueError):
            self.max_episode_length = 999999

    def __getattr__(self, name: str):
        # 上記で定義していない属性はすべて元の env に委譲する
        return getattr(self.env, name)

    def reset(self):
        self._obs_dict, _ = self.env.reset()
        return self._obs_dict["actor"]

    def get_observations(self) -> torch.Tensor:
        if self._obs_dict is None:
            self.reset()
        return self._obs_dict["actor"]

    def step(self, actions: torch.Tensor):
        self._obs_dict, reward, terminated, truncated, info = self.env.step(actions)
        if terminated.any() or truncated.any():
            self._obs_dict, _ = self.env.reset()

    def close(self):
        self.env.close()

class ZeroAgent:
    """常にゼロアクションを出力するエージェント（初期状態の確認に使う）。"""
    def act(self, obs):
        return torch.zeros(obs.shape[0], 1, device=obs.device)


class RandomAgent:
    """ランダムアクションを出力するエージェント（環境の動作確認に使う）。"""
    def act(self, obs):
        return torch.rand(obs.shape[0], 1, device=obs.device) * 2 - 1  # [-1, 1]


# ============================================================
# メイン実行ループ
# ============================================================

def run_play(args):
    from cartpole_env_cfg import cartpole_env_cfg
    from mjlab.envs import ManagerBasedRlEnv

    device = f"cuda:{args.gpu_id}" if args.gpu_id >= 0 else "cpu"
    swing_up = args.task == "swingup"

    # ── 1. 環境を作る ───────────────────────────────────────────────
    # play 時は 1 環境・エピソード長無制限
    env_cfg = cartpole_env_cfg(swing_up=swing_up, num_envs=1)
    env_cfg.episode_length_s = 9999.0

    env = ManagerBasedRlEnv(cfg=env_cfg, device=device)

    # 観測次元・行動次元を確認
    obs_dim = env.observation_space.spaces["actor"].shape[-1]
    act_dim = env.action_space.shape[-1]
    print(f"[play] 観測次元: {obs_dim}")
    print(f"[play] 行動次元: {act_dim}")

    # ── 2. エージェントを選ぶ ────────────────────────────────────────
    if args.agent == "zero":
        agent = ZeroAgent()
        print("[play] エージェント: ゼロアクション")
    elif args.agent == "random":
        agent = RandomAgent()
        print("[play] エージェント: ランダム")
    else:
        # 学習済みモデルをロード
        agent = load_policy(args.checkpoint, obs_dim, act_dim, device)

    # ── 3. ビューアを起動する ────────────────────────────────────────
    if args.viewer == "native":
        _run_with_native_viewer(env, agent, device, args)
    elif args.viewer == "viser":
        _run_with_viser(env, agent, device, args)
    else:
        # ビューアなしで一定ステップだけ実行（デバッグ用）
        _run_headless(env, agent, device, steps=args.steps)

    env.close()

def _run_with_native_viewer(env, agent, device, args):
    from mjlab.viewer import NativeMujocoViewer

    print("[play] Native Viewer を起動します（ウィンドウが開きます）")
    print("[play] Ctrl+C または Q キーで終了")

    policy = PolicyWrapper(agent, device)
    wrapped_env = EnvWrapper(env, device)
    wrapped_env.reset()

    viewer = NativeMujocoViewer(wrapped_env, policy)
    viewer.run()

def _run_with_viser(env, agent, device, args):
    from mjlab.viewer import ViserPlayViewer

    print("[play] Viser Viewer を起動します")
    print("[play] ブラウザで http://localhost:8080 を開いてください")
    print("[play] Ctrl+C で終了")

    policy = PolicyWrapper(agent, device)
    wrapped_env = EnvWrapper(env, device)
    wrapped_env.reset()

    viewer = ViserPlayViewer(wrapped_env, policy)
    viewer.run()

def _run_headless(env, agent, device, steps: int = 500):
    """
    ビューアなしで環境を動かし、報酬をコンソールに表示する（デバッグ用）。
    """
    print(f"[play] ヘッドレスモードで {steps} ステップ実行します")

    obs_dict, _ = env.reset()

    total_reward = 0.0
    for step in range(steps):
        action = get_action(agent, obs_dict, device)

        obs_dict, reward, terminated, truncated, info = env.step(action)
        total_reward += reward.item()

        if step % 100 == 0:
            print(f"  Step {step:4d} | reward = {reward.item():.4f} | total = {total_reward:.2f}")

        if terminated.any() or truncated.any():
            obs_dict, _ = env.reset()

    print(f"[play] 完了。累計報酬: {total_reward:.2f}")


# ============================================================
# CLI エントリポイント
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="Cartpole mjlab play/viewer")
    parser.add_argument(
        "--checkpoint",
        type=str,
        default="",
        help="学習済みモデルのパス (.pt ファイル)",
    )
    parser.add_argument(
        "--agent",
        choices=["policy", "zero", "random"],
        default="policy",
        help="使用するエージェント。policy=学習済み, zero=ゼロ, random=ランダム",
    )
    parser.add_argument(
        "--viewer",
        choices=["native", "viser", "headless"],
        default="native",
        help="使用するビューア",
    )
    parser.add_argument(
        "--task",
        choices=["balance", "swingup"],
        default="swingup",
        help="課題の種類",
    )
    parser.add_argument(
        "--gpu-id",
        type=int,
        default=0,
        help="使用する GPU の ID。-1 で CPU",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=0,
        help="実行するステップ数（0=無制限）",
    )
    args = parser.parse_args()

    # agent=policy のときはチェックポイントが必要
    if args.agent == "policy" and not args.checkpoint:
        print("[play] エラー: --agent policy のときは --checkpoint が必要です")
        print("[play] ヒント: --agent random で動作確認できます")
        return

    run_play(args)


if __name__ == "__main__":
    main()