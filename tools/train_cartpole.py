"""
train.py
=========
Cartpole 環境を mjlab + RSL-RL (PPO) で学習するスクリプト。

使い方:
    # バランス課題（デフォルト）
    python train.py

    # スウィングアップ課題
    python train.py --task swingup

    # 並列環境数を変える
    python train.py --num-envs 512

    # GPU ID を指定する（デフォルト: 0）
    python train.py --gpu-id 0

    # 学習イテレーション数を変える
    python train.py --max-iters 1000

注意:
    mjlab は NVIDIA GPU が必要（学習時）。
    CPU のみの場合は `--gpu-id -1` を指定してください（速度は遅くなります）。
"""

import argparse
from dataclasses import dataclass
from pathlib import Path
from datetime import datetime

# ── mjlab ─────────────────────────────────────────────────────────────
from mjlab.tasks.registry import register_mjlab_task
from mjlab.rl import RslRlOnPolicyRunnerCfg, RslRlModelCfg, RslRlPpoAlgorithmCfg

# 今回作った環境設定をインポート
from cartpole_env_cfg import cartpole_env_cfg



# ============================================================
# PPO ハイパーパラメータ設定
# ============================================================

def cartpole_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
    """
    Cartpole 用の RSL-RL PPO 設定。

    ネットワーク:
        Actor / Critic ともに 2 層 MLP (64 ユニット, ELU 活性化)
        Cartpole は状態空間が小さいので軽量なネットワークで十分。

    PPO ハイパーパラメータ:
        学習率: 1e-3  ← Adam オプティマイザ
        ミニバッチ数: 4
        エポック数: 5  ← 1 ロールアウトを 5 回学習
        クリップ比率: 0.2  ← 標準的な PPO 設定
    """
    return RslRlOnPolicyRunnerCfg(
        # ── ネットワーク構造 ─────────────────────────────────────────
        # Actor と Critic は別々の RslRlModelCfg で設定する（旧 RslRlPpoActorCriticCfg は廃止）
        actor=RslRlModelCfg(
            class_name="MLPModel",
            hidden_dims=(64, 64),        # MLP: obs_dim → 64 → 64 → act_dim
            activation="elu",
            distribution_cfg={
                "class_name": "GaussianDistribution",
                "init_std": 1.0,
                "std_type": "scalar",
            },
        ),
        critic=RslRlModelCfg(
            class_name="MLPModel",
            hidden_dims=(64, 64),        # MLP: obs_dim → 64 → 64 → 1
            activation="elu",
            distribution_cfg=None,
        ),
        # ── PPO アルゴリズム ─────────────────────────────────────────
        algorithm=RslRlPpoAlgorithmCfg(
            value_loss_coef=1.0,
            use_clipped_value_loss=True,
            clip_param=0.2,
            entropy_coef=0.005,          # 小さいエントロピー係数（探索促進）
            num_learning_epochs=5,
            num_mini_batches=4,
            learning_rate=1e-3,
            schedule="adaptive",         # KL ダイバージェンスに基づいてlrを調整
            gamma=0.99,                  # 割引率
            lam=0.95,                    # GAE の λ
            desired_kl=0.01,
            max_grad_norm=1.0,
        ),
        # ── トレーニングループ ────────────────────────────────────────
        num_steps_per_env=24,            # 1 イテレーションで各環境から収集するステップ数
        max_iterations=1000,              # 総イテレーション数
        # ── ロギング・保存 ─────────────────────────────────────────
        save_interval=200,               # 100 イテレーションごとにモデルを保存
        experiment_name="cartpole",
        run_name="test",                     # 空にすると日時から自動生成
        logger="tensorboard",            # "tensorboard" / "wandb"
    )


# ============================================================
# メイン
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="Cartpole mjlab training")
    parser.add_argument(
        "--task",
        choices=["balance", "swingup"],
        default="swingup",
        help="学習する課題。balance=バランス, swingup=スウィングアップ",
    )
    parser.add_argument(
        "--num-envs",
        type=int,
        default=512,
        help="並列環境数。GPU 学習時は 512〜4096 推奨",
    )
    parser.add_argument(
        "--gpu-id",
        type=int,
        default=0,
        help="使用する GPU の ID。-1 で CPU 学習（低速）",
    )
    parser.add_argument(
        "--max-iters",
        type=int,
        default=1000,
        help="学習の総イテレーション数",
    )
    args = parser.parse_args()

    swing_up = args.task == "swingup"
    task_id  = "MyCartpole-Swingup" if swing_up else "MyCartpole-Balance"

    print(f"[train] Task      : {task_id}")
    print(f"[train] Num envs  : {args.num_envs}")
    print(f"[train] GPU ID    : {args.gpu_id}")
    print(f"[train] Max iters : {args.max_iters}")

    # ── 1. 環境・PPO 設定を作る ─────────────────────────────────────
    env_cfg = cartpole_env_cfg(swing_up=swing_up, num_envs=args.num_envs)
    rl_cfg  = cartpole_ppo_runner_cfg()

    # max_iterations を CLI 引数で上書き
    rl_cfg.max_iterations = args.max_iters

    # play 用設定（ランダム化なし・エピソード長無制限）
    play_cfg = cartpole_env_cfg(swing_up=swing_up, num_envs=1)
    play_cfg.episode_length_s = float("inf")

    # ── 2. タスクを登録する ──────────────────────────────────────────
    # register_mjlab_task は タスク名 → (env_cfg, rl_cfg) のペアを登録する
    # register_mjlab_task(
    #     task_id=task_id,
    #     env_cfg=env_cfg,
    #     play_env_cfg=play_cfg,
    #     rl_cfg=rl_cfg,
    # )

    # ── 3. 学習を実行する ────────────────────────────────────────────
    import torch
    # MjlabOnPolicyRunner と RslRlVecEnvWrapper は mjlab.rl のトップレベルに公開されている
    from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
    from mjlab.envs import ManagerBasedRlEnv

    device = f"cuda:{args.gpu_id}" if args.gpu_id >= 0 else "cpu"

    # 環境を生成
    env = ManagerBasedRlEnv(cfg=env_cfg, device=device)
    # RSL-RL が期待する VecEnv インターフェースにラップ
    vec_env = RslRlVecEnvWrapper(env)

    # PPO ランナーを作る
    run_name = rl_cfg.run_name or datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = str(Path(__file__).parent / "logs" / rl_cfg.experiment_name / run_name)
    import dataclasses
    runner = MjlabOnPolicyRunner(
        env=vec_env,
        train_cfg=dataclasses.asdict(rl_cfg),  # dataclass → dict に変換
        log_dir=log_dir,
        device=device,
    )

    # 学習開始
    print(f"\n[train] 学習を開始します... (ログ: {log_dir})")
    print("[train] TensorBoard: tensorboard --logdir logs\n")
    runner.learn(num_learning_iterations=rl_cfg.max_iterations)

    print("[train] 学習完了！")
    env.close()


if __name__ == "__main__":
    main()