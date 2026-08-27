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
    scale: float = 1.0
    gear_ratio: float = 2.0      # 減速比
    kp_nominal: float = 0.0
    ki_nominal: float = 0.0
    max_current: float = 23.0     # モータの最大電流 [Nm]
    torque_constant: float = 0.615  # モータのトルク定数 [Nm/A]
    vel_noise_std: float = 0.0
    torque_noise_std: float = 0.0

    def build(self, env) -> VelocityPiActionTerm:
        return VelocityPiActionTerm(self, env)


# ============================================================
# ActionTerm 本体
# ============================================================

class VelocityPiActionTerm(ActionTerm):
    cfg: VelocityPiActionTermCfg

    def __init__(self, cfg: VelocityPiActionTermCfg, env):
        super().__init__(cfg, env)

        joint_ids, _ = self._entity.find_joints_by_actuator_names(
            list(cfg.actuator_names)
        )
        # ① joint_idsを一度だけGPU上のテンソルにしておく
        self._joint_ids = torch.as_tensor(
            joint_ids, dtype=torch.long, device=env.device
        )

        N = env.num_envs
        device = env.device
        self._decimation = env.cfg.decimation # 15

        self._dt = env.physics_dt
        self._gear = cfg.gear_ratio
        self._torque_const = cfg.torque_constant
        self._max_current = cfg.max_current 

        self._kp = torch.full((N,), cfg.kp_nominal, device=device)
        self._ki = torch.full((N,), cfg.ki_nominal, device=device)
        # ② kp + ki*dt をあらかじめ計算しておく（kp/kiが変わった時だけ更新すればよい）
        self._kp_plus_kidt = self._kp + self._ki * self._dt

        self._e_prev = torch.zeros(N, device=device)
        self._u_prev = torch.zeros(N, device=device)
        self._target_vel = torch.zeros(N, device=device)
        self._raw_actions = torch.zeros(N, len(joint_ids), device=device)

        self._vel_noise_buf = torch.zeros(self._decimation, N, device=device)
        self._torque_noise_buf = torch.zeros(self._decimation, N, device=device)
        self._substep_idx = 0  # 現在何回目の物理サブステップか(0〜decimation-1)

        self._ev_start = torch.cuda.Event(enable_timing=True)  # 使い回し用
        self._ev_end = torch.cuda.Event(enable_timing=True)

    def _refresh_pid_coeffs(self) -> None:
        """kp/kiを変更した時(resetやランダマイズ時)はこれを呼ぶ。"""
        self._kp_plus_kidt = self._kp + self._ki * self._dt

    @property
    def action_dim(self) -> int:
        return self._joint_ids.numel()

    @property
    def raw_action(self) -> torch.Tensor:
        return self._raw_actions

    def process_actions(self, actions: torch.Tensor) -> None:
        self._raw_actions.copy_(actions)
        self._target_vel.copy_(actions.squeeze(-1) * self.cfg.scale)

        # ── 15回分のノイズをdecimation1回のRNG呼び出しでまとめて生成 ──
        if self.cfg.vel_noise_std > 0.0:
            self._vel_noise_buf.normal_(0.0, self.cfg.vel_noise_std)  # in-place生成
        if self.cfg.torque_noise_std > 0.0:
            self._torque_noise_buf.normal_(0.0, self.cfg.torque_noise_std)

        self._substep_idx = 0  # 新しいポリシー周期の先頭に戻す

    def apply_actions(self) -> None:
        current_vel_wheel = self._entity.data.joint_vel.index_select(1, self._joint_ids).squeeze(-1)
        current_vel_motor = current_vel_wheel * self._gear
        current_vel_motor = current_vel_motor + self._vel_noise_buf[self._substep_idx]
        e_motor = self._target_vel - current_vel_motor

        delta_u = self._kp_plus_kidt * e_motor - self._kp * self._e_prev
        u = self._u_prev + delta_u
        u.clamp_(-self._max_current, self._max_current)

        torque_motor = u * self._torque_const
        torque_motor = torque_motor + self._torque_noise_buf[self._substep_idx]
        torque_wheel = torque_motor * self._gear

        self._e_prev.copy_(e_motor)
        self._u_prev.copy_(u)

        self._entity.set_joint_effort_target(torque_wheel.unsqueeze(-1), joint_ids=self._joint_ids)

        self._substep_idx = min(self._substep_idx + 1, self._decimation - 1)


    def reset(self, env_ids: torch.Tensor) -> None:
        self._e_prev[env_ids] = 0.0
        self._u_prev[env_ids] = 0.0
        self._target_vel[env_ids] = 0.0
        self._raw_actions[env_ids] = 0.0 

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
    pid_term._refresh_pid_coeffs()