"""
actions.py
===========
BLDCモータ用 速度型PIコントローラーの ActionTerm 実装。

対象: back_tire_pitch（後輪）
減速比: 2（モータが関節の2倍速く回る）

変換関係:
    vel_motor  = vel_joint × gear_ratio        # モータ速度
    torque_joint = torque_motor × gear_ratio   # 関節トルク（増力）
    ただし MuJoCo の gear が XML で設定済みの場合は
    set_joint_effort_target に関節トルクをそのまま渡せばよい。

速度型PI:
    e[k]  = vel_target_joint - vel_current_joint
    Δu    = Kp*(e[k] - e[k-1]) + Ki*e[k]*dt
    u[k]  = clamp(u[k-1] + Δu, -max_torque, max_torque)
"""

from __future__ import annotations
from dataclasses import dataclass, field
import torch
from mjlab.managers import ActionTerm, ActionTermCfg


# ============================================================
# 設定クラス
# ============================================================

@dataclass
class VelocityPiActionTermCfg(ActionTermCfg):
    """
    モータ 速度型PIアクションの設定。

    entity_name   : シーン内のエンティティ名
    actuator_names: XML のアクチュエータ名（内部でjoint IDに変換）
    scale         : ポリシー出力 [-1, 1] → 関節目標速度 [rad/s]
    gear_ratio    : 減速比（モータ速度 = 関節速度 × gear_ratio）
    kp_nominal    : 比例ゲイン（関節速度誤差に対して）
    ki_nominal    : 積分ゲイン
    max_torque    : 関節トルク上限 [Nm]
                    （モータトルク上限 = max_torque / gear_ratio）
    """
    entity_name: str = "bike"
    actuator_names: tuple[str, ...] = ("back_tire_pitch",)
    scale: float = 4.0
    gear_ratio: float = 2.0      # 減速比
    kp_nominal: float = 2.0
    ki_nominal: float = 0.5
    max_torque: float = 12.0     # 関節トルク上限 [Nm]
    torque_constant: float = 0.615  # モータのトルク定数 [Nm/A]

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
        self._dt = env.physics_dt       # 物理ステップ時間 [s]
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

    # ── abstractmethod の実装 ─────────────────────────────────────

    @property
    def action_dim(self) -> int:
        return len(self._joint_ids)

    @property
    def raw_action(self) -> torch.Tensor:
        return self._raw_actions

    def process_actions(self, actions: torch.Tensor) -> None:
        """
        ポリシー出力を関節目標速度に変換して保持する。
        1ポリシーステップに1回呼ばれる。

        actions: [num_envs, action_dim]  ← [-1, 1]
        目標速度 = actions * scale  [rad/s]（関節速度）
        """
        self._raw_actions = actions.clone()
        # ポリシー出力 → 関節目標速度 [rad/s]
        # target_vel_motor = actions.squeeze(-1) * self.cfg.scale
        # self._target_vel = target_vel_motor / self._gear

        self._target_vel = actions.squeeze(-1) * self.cfg.scale

    def apply_actions(self) -> None:
        """
        速度型PIで関節トルクを計算してMuJoCoに渡す。
        デシメーションループ内で物理ステップごとに呼ばれる。

        減速比の考慮:
            - 誤差はすべて関節速度ベースで計算
            - PIゲインはモータ側の応答を想定して設定するため
              関節誤差に対して gear_ratio 倍した感度になる
            - 出力トルクは関節トルクとしてそのまま渡す
              （MuJoCo の gear 設定が XML にある場合は二重にならないよう注意）
        """
        # 現在の関節速度 [N]
        current_vel_joint_wheel = self._entity.data.joint_vel[
            :, self._joint_ids
        ].squeeze(-1)

        # 関節速度誤差
        current_vel_joint_motor = current_vel_joint_wheel * self._gear
        e_motor = self._target_vel - current_vel_joint_motor
        # e_wheel = self._target_vel - current_vel_joint
        # e_motor = e_wheel * self._gear

        # 速度型PI
        delta_u = (
            self._kp * (e_motor - self._e_prev)
            + self._ki * e_motor * self._dt
        )
        u = self._u_prev + delta_u  # [N]（関節トルク）

        # 電流出力(u) -> トルクに変換してクランプ
        torque_motor = u * self._torque_const
        torque_motor = torch.clamp(torque_motor, -self.cfg.max_torque, self.cfg.max_torque)
        torque_wheel = torque_motor * self._gear

        # MuJoCo に関節トルクを渡す
        self._entity.set_joint_effort_target(
            torque_wheel.unsqueeze(-1),
            joint_ids=self._joint_ids,
        )

        # 状態更新: 内部では電流(u)を保持するため、クリップ後の電流値に合わせて更新
        u_clipped = torque_motor/ self._torque_const
        self._e_prev = e_motor.clone()
        self._u_prev = u_clipped.clone()

    def reset(self, env_ids: torch.Tensor) -> None:
        """エピソードリセット時にPI状態をゼロクリア。"""
        self._e_prev[env_ids]      = 0.0
        self._u_prev[env_ids]      = 0.0
        self._target_vel[env_ids]  = 0.0
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