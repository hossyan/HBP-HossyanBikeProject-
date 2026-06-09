from __future__ import annotations
from dataclasses import dataclass, field
import torch
from mjlab.managers import ActionTerm, ActionTermCfg


# ============================================================
# 設定クラス
# ============================================================

@dataclass
class VelocityPiActionTermCfg(ActionTermCfg):
    entity_name: str = "bike"
    actuator_names: tuple[str, ...] = ("back_tire_pitch",)
    scale: float = 4.0
    gear_ratio: float = 2.0      # 減速比
    kp_nominal: float = 2.0
    ki_nominal: float = 0.5
    max_current: float = 23.0     # モータの最大電流 [Nm]
    torque_constant: float = 0.615  # モータのトルク定数 [Nm/A]
    vel_noise_std: float = 0.0
    torque_noise_std: float = 0.0
    pid_interval_min: int = 2
    pid_interval_max: int = 4

    def build(self, env) -> VelocityPiActionTerm:
        return VelocityPiActionTerm(self, env)


# ============================================================
# ActionTerm 本体
# ============================================================

class VelocityPiActionTerm(ActionTerm):
    """
    BLDCモータ用 速度型PIコントローラー。

    ポリシーは「関節目標速度」を出力する。
    PIは関節速度誤差で計算し、関節トルクをMuJoCoに渡す。
    減速比は速度変換・トルク変換の両方で考慮する。
    """

    cfg: VelocityPiActionTermCfg

    def __init__(self, cfg: VelocityPiActionTermCfg, env):
        # 親クラスが self._entity = env.scene[cfg.entity_name] をセット
        super().__init__(cfg, env)

        # actuator名からjoint IDを取得（もとのコードに合わせた方法）
        self._joint_ids, _ = self._entity.find_joints_by_actuator_names(
            list(cfg.actuator_names)
        )

        N = env.num_envs
        device = env.device
        self._dt_base = env.physics_dt # 0.001s
        self._gear = cfg.gear_ratio     # 減速比
        self._torque_const = cfg.torque_constant

        # PIゲインバッファ
        self._kp = torch.full((N,), cfg.kp_nominal, device=device)
        self._ki = torch.full((N,), cfg.ki_nominal, device=device)

        # PI状態バッファ
        self._e_prev  = torch.zeros(N, device=device)
        self._u_prev  = torch.zeros(N, device=device)

        # 目標速度・生出力バッファ
        self._target_vel  = torch.zeros(N, device=device)
        self._raw_actions = torch.zeros(N, len(self._joint_ids), device=device)

        # 制御周期
        self._step_counter = 0
        self._next_pid_step = torch.randint(cfg.pid_interval_min, cfg.pid_interval_max, (N,), device=device)
        self._elapsed_steps = torch.zeros(N, dtype=torch.int, device=device)

        # 出力トルクバッファ
        self._last_torque_wheel = torch.zeros(N, device=device)

        # 遅延バッファ
        self._delayed_actions = torch.zeros(N, len(self._joint_ids), device=device)

    # ── abstractmethod の実装 ─────────────────────────────────────

    @property
    def action_dim(self) -> int:
        return len(self._joint_ids)

    @property
    def raw_action(self) -> torch.Tensor:
        return self._raw_actions

    def process_actions(self, actions: torch.Tensor) -> None:
        self._raw_actions = actions.clone()
        # self._target_vel = actions.squeeze(-1) * self.cfg.scale

        # 1ステップ前のactionを目標速度に使う
        self._target_vel = self._delayed_actions.squeeze(-1) * self.cfg.scale
        # 今回のactionを保存
        self._delayed_actions = actions.clone()

    def apply_actions(self) -> None:
        self._elapsed_steps += 1
        execute_mask = self._elapsed_steps >= self._next_pid_step

        if execute_mask.any():
            current_vel_joint_wheel = self._entity.data.joint_vel[
                :, self._joint_ids
            ].squeeze(-1)

            current_vel_joint_motor = current_vel_joint_wheel * self._gear
            current_vel_joint_motor = current_vel_joint_motor + torch.randn_like(current_vel_joint_motor) * self.cfg.vel_noise_std
            e_motor = self._target_vel - current_vel_joint_motor

            dt = self._elapsed_steps.float() * self._dt_base

            delta_u = (
                self._kp * (e_motor - self._e_prev)
                + self._ki * e_motor * dt
            )
            u = self._u_prev + delta_u
            u_clipped = torch.clamp(u, -self.cfg.max_current, self.cfg.max_current)

            torque_motor = u_clipped * self._torque_const
            torque_motor = torque_motor + torch.randn_like(torque_motor) * self.cfg.torque_noise_std
            torque_wheel = torque_motor * self._gear

            self._last_torque_wheel = torch.where(execute_mask, torque_wheel, self._last_torque_wheel)
            self._e_prev = torch.where(execute_mask, e_motor, self._e_prev)
            self._u_prev = torch.where(execute_mask, u_clipped, self._u_prev)

            self._next_pid_step = torch.where(
                execute_mask,
                torch.randint(self.cfg.pid_interval_min, self.cfg.pid_interval_max, (self._u_prev.shape[0],), device=self._u_prev.device),
                self._next_pid_step
            )
            self._elapsed_steps = torch.where(
                execute_mask,
                torch.zeros_like(self._elapsed_steps),
                self._elapsed_steps
            )

        self._entity.set_joint_effort_target(
            self._last_torque_wheel.unsqueeze(-1),
            joint_ids=self._joint_ids,
    )

    def reset(self, env_ids: torch.Tensor) -> None:
        """エピソードリセット時にPI状態をゼロクリア。"""
        self._e_prev[env_ids]      = 0.0
        self._u_prev[env_ids]      = 0.0
        self._target_vel[env_ids]  = 0.0
        self._raw_actions[env_ids] = 0.0
        self._delayed_actions[env_ids] = 0.0

def randomize_pid_gains(
    env,
    env_ids: torch.Tensor,
    kp_range: tuple,
    ki_range: tuple,
) -> None:
    """
    エピソードリセット時にPIDゲインをランダム化する。
    VelocityPIDActionTerm のインスタンスに直接アクセスして値を書き換える。
    """
    # ActionManagerからVelocityPIDActionTermのインスタンスを取得
    pid_term = env.action_manager.get_term("back_tire_motor")

    n = len(env_ids)
    pid_term._kp[env_ids] = torch.empty(n, device=env.device).uniform_(*kp_range)
    pid_term._ki[env_ids] = torch.empty(n, device=env.device).uniform_(*ki_range)