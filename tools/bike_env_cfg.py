"""
cartpole_env_cfg.py
====================
mjlab を使った倒立振子 (Cartpole) 環境の設定ファイル。

ここでは「Manager-Based RL 環境」を構成する 6 つのピースを定義する。
  1. Entity        … 何をシミュレートするか
  2. Observations  … エージェントが何を見るか
  3. Actions       … エージェントが何をするか
  4. Rewards       … 何が報酬になるか
  5. Terminations  … いつエピソードを終えるか
  6. Events        … リセット時に何をするか

添付された cartpole.xml をそのまま使う。
"""

import math
from pathlib import Path

import mujoco
import torch

# ── mjlab コア ──────────────────────────────────────────────────────────
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp import (
    joint_pos_rel,
    joint_vel_rel,
    reset_joints_by_offset,
    time_out,
    JointEffortActionCfg,
)
from mjlab.managers import (
    EventTermCfg,
    ObservationGroupCfg,
    ObservationTermCfg,
    RewardTermCfg,
    SceneEntityCfg,
    TerminationTermCfg,
)
from mjlab.scene import SceneCfg
from mjlab.terrains import TerrainEntityCfg
from mjlab.sim import MujocoCfg, SimulationCfg

# ── Entity 関連 ─────────────────────────────────────────────────────────
from mjlab.entity import EntityCfg, EntityArticulationInfoCfg
from mjlab.actuator import XmlActuatorCfg

# ============================================================
# 0. XML パス
# ============================================================
_BIKE_XML = Path(__file__).parent / "bike_scene.xml"


# ============================================================
# 1. Entity 設定
# ============================================================

def _get_spec() -> mujoco.MjSpec:
    """MuJoCo モデルスペックを XML から読み込む。"""
    return mujoco.MjSpec.from_file(str(_BIKE_XML))

# 試用アクチュエータ定義
_BIKE_ARTICULATION = EntityArticulationInfoCfg(
    actuators=(
        XmlActuatorCfg(target_names_expr=("back_tire_pitch",)),
        XmlActuatorCfg(target_names_expr=("fork_yaw",)),
    ),
)

# 初期状態
_BIKE_INIT = EntityCfg.InitialStateCfg(
    joint_pos={
        "back_tire_pitch": 0.0, 
        "fork_yaw": 0.0},  
    joint_vel={".*": 0.0},
)

# Entity config
def _get_bike_entity_cfg() -> EntityCfg:
    return EntityCfg(
        spec_fn=_get_spec,
        articulation=_BIKE_ARTICULATION,
        init_state=_BIKE_INIT,
    )


# ============================================================
# 2. カスタム観測関数
# ============================================================
class ComplementaryRollFilter:
    def __init__(
        self,
        accel_sensor_name: str = "bike/body_accel",
        gyro_sensor_name: str = "bike/body_gyro",
        alpha: float = 0.98,
        dt: float = 0.05,
    ):
        self.accel_sensor_name = accel_sensor_name
        self.gyro_sensor_name = gyro_sensor_name
        self.alpha = alpha
        self.dt = dt
        self.roll = None
        self.roll_rate_value = None

    def update(self, env) -> torch.Tensor:
        accel = env.scene[self.accel_sensor_name].data  # [N, 3]
        gyro = env.scene[self.gyro_sensor_name].data    # [N, 3]

        ay = accel[:, 1]
        az = accel[:, 2]
        gx = gyro[:, 0]  # x軸回り roll rate [rad/s]

        roll_acc = torch.atan2(ay, az)

        if self.roll is None or self.roll.shape[0] != env.num_envs:
            self.roll = roll_acc.clone()

        if hasattr(env, "episode_length_buf"):
            reset_mask = env.episode_length_buf == 0
            self.roll[reset_mask] = roll_acc[reset_mask]

        self.roll = self.alpha * (self.roll + gx * self.dt) + (1.0 - self.alpha) * roll_acc
        self.roll_rate_value = gx

        return self.roll.unsqueeze(-1)

    def roll_rate(self, env) -> torch.Tensor:
        gyro = env.scene[self.gyro_sensor_name].data
        self.roll_rate_value = gyro[:, 0]
        return self.roll_rate_value.unsqueeze(-1)

    def get_roll(self, env) -> torch.Tensor:
        if self.roll is None:
            accel = env.scene[self.accel_sensor_name].data
            self.roll = torch.atan2(accel[:, 1], accel[:, 2])
        return self.roll

def body_roll(env, roll_filter: ComplementaryRollFilter) -> torch.Tensor:
    return roll_filter.update(env)


def body_roll_vel(env, roll_filter: ComplementaryRollFilter) -> torch.Tensor:
    return roll_filter.roll_rate(env)


def roll_exceeded(env, roll_filter: ComplementaryRollFilter, limit_rad: float) -> torch.Tensor:
    roll = roll_filter.get_roll(env)
    return torch.abs(roll) > limit_rad


# ============================================================
# 3. カスタム報酬関数
# ============================================================

def bike_reward(
    env,
    cart_cfg: SceneEntityCfg,
    hinge_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """
    dm_control の CartPole と同じ滑らかな積和報酬。

    r = upright × centered × small_ctrl × small_vel
        ↑ [0,1]    ↑ [0,1]   ↑ [0,1]     ↑ [0,1]

    すべての条件が同時に満たされたときだけ高い報酬になる。
    """
    from mjlab.entity import Entity

    cart: Entity = env.scene[cart_cfg.name]

    # ポール角度 → cos
    angle = cart.data.joint_pos[:, hinge_cfg.joint_ids]  # [N, 1]
    cos_angle = torch.cos(angle).squeeze(-1)              # [N]

    # カート位置 (slider joint)
    cart_pos = cart.data.joint_pos[:, cart_cfg.joint_ids].squeeze(-1)  # [N]

    # ポール角速度
    pole_vel = cart.data.joint_vel[:, hinge_cfg.joint_ids].squeeze(-1)  # [N]

    # アクション (最後に適用された ctrl 値)
    # env.action_manager.action は [N, act_dim]
    ctrl = env.action_manager.action.squeeze(-1)  # [N]

    # 各ファクター ─ すべて [0, 1] の範囲
    upright    = (cos_angle + 1.0) / 2.0                      # 直立度
    centered   = (1.0 + _tolerance(cart_pos, margin=1.0)) / 2.0  # 中央度
    small_ctrl = (4.0 + _tolerance(ctrl,     margin=1.0)) / 5.0  # 制御量の小ささ
    small_vel  = (1.0 + _tolerance(pole_vel, margin=5.0)) / 2.0  # 角速度の小ささ

    return upright * centered * small_ctrl * small_vel


def _tolerance(x: torch.Tensor, margin: float) -> torch.Tensor:
    """
    |x| が margin 以下のとき 1、それ以上になるほど 0 に近づくガウス型関数。
    dm_control の `tolerance()` の簡略版。
    """
    return torch.exp(-0.5 * (x / margin) ** 2)


# ============================================================
# 4. 環境設定を組み立てる関数
# ============================================================

def bike_env_cfg(num_envs: int = 1,) -> ManagerBasedRlEnvCfg:
    # 特定のEnityの関節を取得する
    back_tire_cfg  = SceneEntityCfg("bike", joint_names=("back_tire_pitch",))
    fork_cfg = SceneEntityCfg("bike", joint_names=("fork_yaw",))

    roll_filter = ComplementaryRollFilter(
        accel_sensor_name="body_accel",
        gyro_sensor_name="body_gyro",
        alpha=0.98,
        dt=0.01 * 5,
    )

    # ── Observations ────────────────────────────────────────────────
    actor_terms = {
        "back_tire_vel": ObservationTermCfg(
            func=joint_vel_rel,
            params={"asset_cfg": back_tire_cfg},
        ),
        "body_roll": ObservationTermCfg(
            func=body_roll,
            params={"roll_filter": roll_filter},
        ),
        "body_roll_vel": ObservationTermCfg(
            func=body_roll_vel,
            params={"roll_filter": roll_filter},
        ),
    }
    observations = {
        "actor":  ObservationGroupCfg(actor_terms),
        "critic": ObservationGroupCfg({**actor_terms}),  # critic は同じ観測を使う
    }

    # ── Actions ─────────────────────────────────────────────────────
    actions = {
        "effort": JointEffortActionCfg(
            entity_name="bike",
            actuator_names=("back_tire_motor", "fork_motor"),
            scale=1.0,
        ),
    }

    # ── Rewards ─────────────────────────────────────────────────────
    rewards = {
        "smooth_reward": RewardTermCfg(
            func=bike_reward,
            weight=1.0,
            params={"cart_cfg": cart_cfg, "hinge_cfg": hinge_cfg},
        ),
    }

    # ── Terminations ────────────────────────────────────────────────
    # タイムアウトのみ（time_out=True で「打ち切り」扱いになり価値関数をブートストラップ）
    terminations = {
        "roll_exceeded": TerminationTermCfg(
            func=roll_exceeded,
            params={
                "roll_filter": roll_filter,
                "limit_rad": math.radians(45.0),
            },
        ),
        "time_out": TerminationTermCfg(
            func=time_out,
            time_out=True,
        ),
    }

    # ── Events (エピソードリセット時のランダム化) ────────────────────
    # 毎エピソード、初期状態に少しノイズを加えて多様な状態から学習させる
    events = {
        "reset_slider": EventTermCfg(
            func=reset_joints_by_offset,
            mode="reset",
            params={
                "position_range": (-0.1, 0.1),
                "velocity_range": (-0.01, 0.01),
                "asset_cfg": SceneEntityCfg("cartpole", joint_names=("slider",)),
            },
        ),
        "reset_hinge": EventTermCfg(
            func=reset_joints_by_offset,
            mode="reset",
            params={
                "position_range": (-0.1, 0.1),
                "velocity_range": (-0.01, 0.01),
                "asset_cfg": SceneEntityCfg("cartpole", joint_names=("hinge_1",)),
            },
        ),
    }

    # ── 全体を ManagerBasedRlEnvCfg にまとめる ───────────────────────
    return ManagerBasedRlEnvCfg(
        scene=SceneCfg(
            terrain=TerrainEntityCfg(terrain_type="plane"),
            entities={"bike": _get_bike_entity_cfg()},
            num_envs=num_envs,
            env_spacing=4.0,    # 並列環境の配置間隔 [m]
        ),
        observations=observations,
        actions=actions,
        events=events,
        rewards=rewards,
        terminations=terminations,
        sim=SimulationCfg(
            mujoco=MujocoCfg(
                timestep=0.001,
            ),
        ),
        decimation=5,           # 物理 5 ステップに 1 回ポリシーを呼ぶ (20 Hz 制御)
        episode_length_s=50.0,  # 1 エピソードの最大長さ [秒]
    )
