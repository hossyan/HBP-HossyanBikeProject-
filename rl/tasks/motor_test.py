from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import torch

import threading
import time
import random

# keyboard入力
class SharedTarget:
    """スレッド間で安全に target_velocity を共有するためのコンテナ。"""

    def __init__(self, initial: float):
        self._lock = threading.Lock()
        self._value = initial

    def get(self) -> float:
        with self._lock:
            return self._value

    def set(self, value: float) -> None:
        with self._lock:
            self._value = value

    def add(self, delta: float) -> float:
        with self._lock:
            self._value += delta
            return self._value


def _keyboard_listener(shared_target: SharedTarget, step: float = 3.0):
    """
    別スレッドでキー入力を監視し、shared_target を書き換える。
    Up/Down : ±step
    Shift+Up/Down : ±step*5 (大きく動かす)
    0 : 0にリセット
    """
    import keyboard  # pip install keyboard

    print("[keyboard] Up/Down: ±%.2f | Shift+Up/Down: ±%.2f | 0: reset" % (step, step * 5))

    def on_up():
        v = shared_target.add(step)
        # print(f"[keyboard] target_velocity = {v:.3f}")

    def on_down():
        v = shared_target.add(-step)
        print(f"[keyboard] target_velocity = {v:.3f}")

    def on_up_big():
        v = shared_target.add(step * 5)
        print(f"[keyboard] target_velocity = {v:.3f}")

    def on_down_big():
        v = shared_target.add(-step * 5)
        print(f"[keyboard] target_velocity = {v:.3f}")

    def on_reset():
        shared_target.set(0.0)
        print("[keyboard] target_velocity reset to 0")

    keyboard.add_hotkey("up", on_up)
    keyboard.add_hotkey("down", on_down)
    keyboard.add_hotkey("shift+up", on_up_big)
    keyboard.add_hotkey("shift+down", on_down_big)
    keyboard.add_hotkey("0", on_reset)

    # このスレッドを生かし続ける(daemon threadなのでmain終了時に自動終了)
    keyboard.wait()
        

# ============================================================
# 設定(ここを直接書き換えて実行する)
# ============================================================

# 目標速度の基準軸。"motor"(モーター軸) または "wheel"(ホイール軸)
TARGET_MODE = "motor"

# 目標速度 [rad/s]。TARGET_MODE で指定した軸の速度として解釈される。
TARGET_VELOCITY = 0.0

# env_cfg の VelocityPiActionTermCfg と合わせること
SCALE = 10.0
GEAR_RATIO = 2.0

# "native" / "viser" / "headless"
VIEWER = "native"

NUM_ENVS = 1
GPU_ID = 0          # -1 でCPU
STEPS = 200          # headless時のみ使用
BENCH_MODE = False

PROFILE = False

# ============================================================
# 固定速度を出し続けるAgent
# ============================================================

class FixedVelocityAgent:
    """
    back_tire_motor (VelocityPiActionTerm) に一定の目標速度を送り続けるAgent。

    VelocityPiActionTerm.process_actions() の実装により、
        target_vel(モーター軸) = action * scale
    として扱われるため、モーター軸のrad/sを直接指定したい場合は

        action = target_velocity_motor / scale

    とすればよい。

    一方、ホイール軸のrad/sを基準に指定したい場合は、
    apply_actions() 内で
        current_vel_joint_motor = current_vel_joint_wheel * gear_ratio
    としてモーター軸換算されているため、

        action = (target_velocity_wheel * gear_ratio) / scale

    と変換してから渡す必要がある。
    """

    def __init__(
        self,
        act_dim: int,
        shared_target: SharedTarget,
        scale: float,
        gear_ratio: float,
        target_mode: str = "motor",  # "motor" or "wheel"
    ):
        self.act_dim = act_dim
        self.shared_target = shared_target
        self.scale = scale
        self.gear_ratio = gear_ratio
        self.target_mode = target_mode

    def act(self, obs: "torch.Tensor") -> "torch.Tensor":
        import torch

        action = torch.zeros(obs.shape[0], self.act_dim, device=obs.device)
        target_velocity = self.shared_target.get()

        if self.target_mode == "motor":
            # --- モーター軸側の rad/s を直接指定する場合 ---
            action[:, 0] = target_velocity / self.scale

        elif self.target_mode == "wheel":
            # --- ホイール軸側の rad/s を指定する場合 ---
            action[:, 0] = (target_velocity * self.gear_ratio) / self.scale

        else:
            raise ValueError(f"Unknown target_mode: {self.target_mode}")

        return action


# ============================================================
# 実行ロジック
# ============================================================

def run_motor_test():
    from tasks.bike_balance.env_cfg import bike_balance_env_cfg
    from mjlab.envs import ManagerBasedRlEnv

    device = f"cuda:{GPU_ID}" if GPU_ID >= 0 else "cpu"

    env_cfg = bike_balance_env_cfg(num_envs=NUM_ENVS)
    env_cfg.episode_length_s = 9999.0

    env = ManagerBasedRlEnv(cfg=env_cfg, device=device)

    env.reset()  # プロファイル前に必ず1回reset(内部バッファを正しい状態にするため)

    if PROFILE:
        term = env.action_manager.get_term("back_tire_motor")
        profile_apply_actions(term)
        env.close()
        return  # プロファイルだけして終了。viewerは起動しない

    obs_dim = env.observation_space.spaces["actor"].shape[-1]
    act_dim = env.action_space.shape[-1]
    print(f"[motor_test] Observation dim : {obs_dim}")
    print(f"[motor_test] Action dim      : {act_dim}")
    print(f"[motor_test] Num envs        : {NUM_ENVS}")
    print(f"[motor_test] Device          : {device}")

    # back_tire_motor が action[0] であることを前提にしている
    # (fork_motor などが有効化された場合はインデックスがずれるため要注意)
    if act_dim != 1:
        print(
            f"[motor_test] 警告: act_dim={act_dim} です。"
            f" back_tire_motor 以外のactionも有効になっている可能性があります。"
        )

    # --- 共有状態を作成し、キーボードリスナーを起動 ---
    shared_target = SharedTarget(TARGET_VELOCITY)
    listener_thread = threading.Thread(
        target=_keyboard_listener, args=(shared_target,), daemon=True
    )
    listener_thread.start()

    agent = FixedVelocityAgent(
        act_dim=act_dim,
        shared_target=shared_target,
        scale=SCALE,
        gear_ratio=GEAR_RATIO,
        target_mode=TARGET_MODE,
    )
    print(
        f"[motor_test] Agent           : fixed "
        f"(mode={TARGET_MODE}, target={TARGET_VELOCITY} rad/s, "
        f"scale={SCALE}, gear_ratio={GEAR_RATIO})"
    )

    try:
        if BENCH_MODE:
            _run_headless_bench(env, agent, steps=STEPS)
        elif VIEWER == "native":
            _run_with_native_viewer(env, agent)
        elif VIEWER == "viser":
            _run_with_viser(env, agent)
        else:
            _run_headless(env, agent, steps=STEPS)
    finally:
        env.close()

def profile_apply_actions(term, n_calls: int = 2000, warmup: int = 100):
    import torch

    for _ in range(warmup):
        term.apply_actions()

    times_ms = []
    for _ in range(n_calls):
        term._ev_start.record()
        term.apply_actions()      # ← ここを外側から挟む。中身は無変更
        term._ev_end.record()
        torch.cuda.synchronize()
        times_ms.append(term._ev_start.elapsed_time(term._ev_end))

    avg_us = sum(times_ms) / len(times_ms) * 1000
    print(f"apply_actions: avg {avg_us:.2f} us/call over {n_calls} calls")

    

# ============================================================
# viewer / headless 実行ヘルパー
# ============================================================

class EnvWrapper:
    """viewer側が期待するインターフェースに合わせるための薄いラッパー"""

    def __init__(self, env, log_every: int = 100, joint_name: str | None = None, shared_target: SharedTarget | None = None):
        self.env = env
        self._obs_dict = None
        self.num_envs = env.num_envs
        try:
            self.max_episode_length = env.max_episode_length
        except (OverflowError, ValueError):
            self.max_episode_length = 999999

        # --- ログ用に追加 ---
        self._step_count = 0
        self._log_every = log_every
        self._joint_idx = None
        self.shared_target = shared_target
        if joint_name is not None:
            articulation = self.env.scene["bike"]
            self._joint_idx = articulation.joint_names.index(joint_name)

    def __getattr__(self, name: str):
        return getattr(self.env, name)

    def reset(self):
        self._obs_dict, _ = self.env.reset()
        return self._obs_dict["actor"]

    def get_observations(self):
        if self._obs_dict is None:
            self.reset()
        return self._obs_dict["actor"]

    def step(self, actions):
        self._obs_dict, _, terminated, truncated, _ = self.env.step(actions)

        # if self._log_every > 0 and self._step_count % self._log_every == 0:
        #     joint_vel = self.env.scene["bike"].data.joint_vel
        #     target = self.shared_target.get() if self.shared_target is not None else None
        #     target = target / 2 
        #     if self._joint_idx is not None:
        #         val = joint_vel[0, self._joint_idx].item()
        #         val_noisy = val + random.gauss(0.0, 0.0487)
        #         print(f"{self._step_count * 1000:5d},{val * 2:6.4f},{target:2.1f},{val_noisy * 2:6.4f}")
        #     else:
        #         print(f"{self._step_count:5d}, target={target}, joint_vel={joint_vel[0].cpu().numpy()}")

        # self._step_count += 1

        if terminated.any() or truncated.any():
            self._obs_dict, _ = self.env.reset()

class PolicyWrapper:
    def __init__(self, agent):
        self.agent = agent

    def __call__(self, obs: "torch.Tensor") -> "torch.Tensor":
        return self.agent.act(obs)

# 特定関節のログを出したい場合の設定
LOG_EVERY_N_STEPS = 1
LOG_JOINT_NAME = "back_tire_pitch"
def _run_with_native_viewer(env, agent):
    from mjlab.viewer import NativeMujocoViewer

    print("[motor_test] Starting Native Viewer. Press Ctrl+C or Q to quit.")
    wrapped_env = EnvWrapper(
        env,
        log_every=LOG_EVERY_N_STEPS,
        joint_name=LOG_JOINT_NAME,
        shared_target=agent.shared_target,
        )
    wrapped_env.reset()
    viewer = NativeMujocoViewer(wrapped_env, PolicyWrapper(agent))
    viewer.run()

def _run_with_viser(env, agent):
    from mjlab.viewer import ViserPlayViewer

    print("[motor_test] Starting Viser Viewer.")
    print("[motor_test] Open http://localhost:8080 in your browser.")
    wrapped_env = EnvWrapper(env)
    wrapped_env.reset()
    viewer = ViserPlayViewer(wrapped_env, PolicyWrapper(agent))
    viewer.run()


def _run_headless(env, agent, steps: int):
    print(f"[motor_test] Running headless for {steps} steps.")
    obs_dict, _ = env.reset()

    for step in range(steps):
        action = agent.act(obs_dict["actor"])
        obs_dict, reward, terminated, truncated, _ = env.step(action)

        if step % 100 == 0:
            joint_vel = env.scene["bike"].data.joint_vel
            print(
                f"  Step {step:5d} | "
                f"action = {action[0].cpu().numpy()} | "
                f"joint_vel(sample) = {joint_vel[0].cpu().numpy()}"
            )

        if terminated.any() or truncated.any():
            obs_dict, _ = env.reset()

    print("[motor_test] Done.")

def _run_headless_bench(env, agent, steps: int = 2000, warmup: int = 200):
    """headlessでsteps/secを計測する(printやviewerなし、純粋な速度計測用)"""
    import time
    import torch

    obs_dict, _ = env.reset()

    # ウォームアップ：GPUカーネルのキャッシュ/JITコンパイルを済ませる
    for _ in range(warmup):
        action = agent.act(obs_dict["actor"])
        obs_dict, _, terminated, truncated, _ = env.step(action)
        if terminated.any() or truncated.any():
            obs_dict, _ = env.reset()

    torch.cuda.synchronize()
    t0 = time.perf_counter()

    for _ in range(steps):
        action = agent.act(obs_dict["actor"])
        obs_dict, _, terminated, truncated, _ = env.step(action)
        if terminated.any() or truncated.any():
            obs_dict, _ = env.reset()

    torch.cuda.synchronize()
    t1 = time.perf_counter()

    elapsed = t1 - t0
    sps = steps / elapsed
    env_steps_per_sec = sps * env.num_envs  # 全env合計のスループット

    print("=" * 50)
    print(f"[bench] num_envs        : {env.num_envs}")
    print(f"[bench] steps           : {steps}")
    print(f"[bench] elapsed         : {elapsed:.3f} sec")
    print(f"[bench] steps/sec       : {sps:.1f}")
    print(f"[bench] env-steps/sec   : {env_steps_per_sec:.1f}")
    print("=" * 50)


if __name__ == "__main__":
    run_motor_test()