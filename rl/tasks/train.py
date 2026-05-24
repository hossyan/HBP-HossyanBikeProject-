"""
train.py
=========
CLIコマンド（uv run train）を使わず直接実行したい場合の補助スクリプト。

【推奨】CLIコマンドで実行:
    cd rl
    pip install -e .
    uv run train MyBike-Balance --env.scene.num-envs 512

【このスクリプトで実行】:
    cd rl
    python train.py
    python train.py --num-envs 64 --max-iters 500

CLIコマンドが使える場合はそちらを推奨。
デバッグや学習ループに独自処理を挟みたい場合にこのスクリプトを使う。
"""

import argparse
import dataclasses
from datetime import datetime
from pathlib import Path

import torch

from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.envs import ManagerBasedRlEnv

# タスクのインポート（レジストリへの登録も行われる）
from tasks.bike_balance.env_cfg import bike_balance_env_cfg, bike_balance_runner_cfg


def main():
    parser = argparse.ArgumentParser(description="Bike Balance Training")
    parser.add_argument("--num-envs",  type=int, default=512)
    parser.add_argument("--gpu-id",    type=int, default=0)
    parser.add_argument("--max-iters", type=int, default=1800)
    args = parser.parse_args()

    device = f"cuda:{args.gpu_id}" if args.gpu_id >= 0 else "cpu"

    print(f"[train] Num envs  : {args.num_envs}")
    print(f"[train] GPU ID    : {args.gpu_id}")
    print(f"[train] Max iters : {args.max_iters}")

    # 設定を作る
    env_cfg = bike_balance_env_cfg(num_envs=args.num_envs)
    rl_cfg  = bike_balance_runner_cfg()
    rl_cfg.max_iterations = args.max_iters

    # 環境を生成
    env     = ManagerBasedRlEnv(cfg=env_cfg, device=device)
    vec_env = RslRlVecEnvWrapper(env)

    # ログディレクトリ
    run_name = rl_cfg.run_name or datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir  = str(Path(__file__).parent / "logs" / rl_cfg.experiment_name / "log1")

    # ランナーを作る
    runner = MjlabOnPolicyRunner(
        env=vec_env,
        train_cfg=dataclasses.asdict(rl_cfg),
        log_dir=log_dir,
        device=device,
    )

    print(f"\n[train] 学習を開始します... (ログ: {log_dir})")
    print("[train] TensorBoard: tensorboard --logdir logs\n")
    runner.learn(num_learning_iterations=rl_cfg.max_iterations)

    print("[train] 学習完了！")
    env.close()


if __name__ == "__main__":
    main()