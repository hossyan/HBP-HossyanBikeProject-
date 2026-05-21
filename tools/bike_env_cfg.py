"""
bike_env_cfg.py
================
mjlab を使ったバイク倒立制御の環境設定ファイル。
"""

import math
from pathlib import Path

import mujoco
import torch

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp import (
    joint_pos_rel,
    joint_vel_rel,
    reset_joints_by_offset,
    reset_scene_to_default,
    time_out,
    JointEffortActionCfg,
    # JointPositionActionCfg,  # fork を動かす場合はコメントアウトを解除
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
from mjlab.entity import EntityCfg, EntityArticulationInfoCfg
from mjlab.actuator import XmlActuatorCfg

# ============================================================
# 0. XML パス
# ============================================================
_BIKE_XML = Path(__file__).parent / "bike_V3_mjcf.xml"


# ============================================================
# 1. Entity 設定
# ============================================================
def _get_spec() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_file(str(_BIKE_XML))

# [FIX1] XmlActuatorCfg には joint名を指定する（確認済み）
_BIKE_ARTICULATION = EntityArticulationInfoCfg(
    actuators=(
        XmlActuatorCfg(target_names_expr=("back_tire_pitch",)),
        XmlActuatorCfg(target_names_expr=("fork_yaw",)),
    ),
)

_BIKE_INIT = EntityCfg.InitialStateCfg(
    joint_pos={
        "back_tire_pitch": 0.0,
        "fork_yaw": math.radians(60),   # 60deg = π/3
    },
    joint_vel={".*": 0.0},
)

def _get_bike_entity_cfg() -> EntityCfg:
    return EntityCfg(
        spec_fn=_get_spec,
        articulation=_BIKE_ARTICULATION,
        init_state=_BIKE_INIT,
    )


# ============================================================
# 2. カスタム観測関数（クラスベース term）
# ============================================================

class ComplementaryRollFilter:
    """
    mjlab クラスベース ObservationTerm として動作する相補フィルタ。

    [FIX2] __init__(self, cfg, env) シグネチャに修正。
           mjlab が環境生成時に自動でインスタンス化する。

    [FIX3] センサー名を "bike/body_accel" 形式に修正。
    """

    def __init__(self, cfg: ObservationTermCfg, env):
        self.alpha = cfg.params.get("alpha", 0.98)
        self.dt    = env.step_dt  # ポリシーステップ間隔 [s]
        # roll 推定値バッファ [num_envs]
        self._roll = torch.zeros(env.num_envs, device=env.device)

    def reset(self, env_ids: torch.Tensor) -> None:
        """エピソードリセット時に指定環境のrollをゼロクリア"""
        self._roll[env_ids] = 0.0

    def __call__(
        self,
        env,
        accel_sensor_name: str,
        gyro_sensor_name: str,
        alpha: float = 0.98,
    ) -> torch.Tensor:
        """
        相補フィルタで roll を推定し [N, 1] で返す。

        相補フィルタ:
          roll_acc = atan2(ay, az)                  (accelから静的推定)
          roll_gyro = roll_prev + gx * dt           (gyro積分)
          roll = alpha * roll_gyro + (1-alpha) * roll_acc
        """
        accel = env.scene[accel_sensor_name].data  # [N, 3]
        gyro  = env.scene[gyro_sensor_name].data   # [N, 3]

        ay = accel[:, 1]
        az = accel[:, 2]
        gx = gyro[:, 0]   # roll角速度 [rad/s]

        roll_acc  = torch.atan2(ay, az)
        self._roll = alpha * (self._roll + gx * self.dt) + (1.0 - alpha) * roll_acc

        return self._roll.unsqueeze(-1)  # [N, 1]


class RollRateObservation:
    """
    gyro の x 軸成分（roll角速度）を返す ObservationTerm。
    ComplementaryRollFilter と独立したクラスにすることで
    ObservationTermCfg に個別に登録できる。
    """

    def __init__(self, cfg: ObservationTermCfg, env):
        pass  # 状態なし

    def __call__(
        self,
        env,
        gyro_sensor_name: str,
    ) -> torch.Tensor:
        gyro = env.scene[gyro_sensor_name].data  # [N, 3]
        return gyro[:, 0].unsqueeze(-1)           # [N, 1]


# ============================================================
# 3. カスタム報酬・終了条件関数
# ============================================================

def body_roll_reward(
    env,
    accel_sensor_name: str,
    # gyro_sensor_name: str,
    margin: float,
) -> torch.Tensor:
    """
    [FIX4] 報酬関数はシンプルな関数として定義。
           ComplementaryRollFilterのインスタンスを報酬関数に渡すと
           観測と報酬で異なるインスタンスになり状態が二重管理になるため、
           報酬ではaccelから直接roll推定する。
    """
    accel = env.scene[accel_sensor_name].data
    roll  = torch.atan2(accel[:, 1], accel[:, 2])
    return _tolerance(roll, margin=margin)


def body_roll_vel_penalty(
    env,
    gyro_sensor_name: str,
    margin: float,
) -> torch.Tensor:
    gyro     = env.scene[gyro_sensor_name].data
    roll_vel = gyro[:, 0]
    return 1.0 - _tolerance(roll_vel, margin=margin)


def back_tire_vel_penalty(
    env,
    asset_cfg: SceneEntityCfg,
    margin: float,
) -> torch.Tensor:
    from mjlab.entity import Entity
    bike: Entity = env.scene[asset_cfg.name]
    back_tire_vel = bike.data.joint_vel[:, asset_cfg.joint_ids].squeeze(-1)
    return 1.0 - _tolerance(back_tire_vel, margin=margin)


def roll_exceeded(
    env,
    accel_sensor_name: str,
    limit_rad: float,
) -> torch.Tensor:
    accel = env.scene[accel_sensor_name].data
    roll  = torch.atan2(accel[:, 1], accel[:, 2])
    return torch.abs(roll) > limit_rad

# forkmotorの初期目標角度を60degにするため
def set_joint_position_target(
    env,
    env_ids: torch.Tensor | None,
    target_position: float,
    asset_cfg: SceneEntityCfg,
) -> None:
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device, dtype=torch.int)

    asset = env.scene[asset_cfg.name]
    joint_ids = asset_cfg.joint_ids
    if isinstance(joint_ids, list):
        joint_ids = torch.tensor(joint_ids, device=env.device)

    target = torch.full((len(env_ids),), target_position, device=env.device)
    asset.set_joint_position_target(target, joint_ids=joint_ids, env_ids=env_ids)


def _tolerance(x: torch.Tensor, margin: float) -> torch.Tensor:
    return torch.exp(-0.5 * (x / margin) ** 2)


# ============================================================
# 4. 環境設定
# ============================================================

def bike_env_cfg(num_envs: int = 1) -> ManagerBasedRlEnvCfg:

    back_tire_cfg = SceneEntityCfg("bike", joint_names=("back_tire_pitch",))

    # センサー名（エンティティ名/センサー名）
    ACCEL = "bike/body_accel"
    GYRO  = "bike/body_gyro"

    # ── Observations ────────────────────────────────────────────────
    actor_terms = {
        "back_tire_vel": ObservationTermCfg(
            func=joint_vel_rel,
            params={"asset_cfg": back_tire_cfg},
        ),
        # [FIX2] クラスベース term: func にクラスを渡す
        "body_roll": ObservationTermCfg(
            func=ComplementaryRollFilter,
            params={
                "accel_sensor_name": ACCEL,
                "gyro_sensor_name":  GYRO,
                "alpha": 0.98,
            },
        ),
        "body_roll_vel": ObservationTermCfg(
            func=RollRateObservation,
            params={
                "gyro_sensor_name": GYRO,
            },
        ),
    }
    observations = {
        "actor":  ObservationGroupCfg(actor_terms),
        "critic": ObservationGroupCfg({**actor_terms}),
    }

    # ── Actions ─────────────────────────────────────────────────────
    # [FIX5] actuator_names はタプル形式（末尾カンマ必須）
    #        XML の motor名ではなく joint名を使う（cartpoleで確認済み）
    actions = {
        # back_tire: motor アクチュエータ（力制御）
        "back_tire_motor": JointEffortActionCfg(
            entity_name="bike",
            actuator_names=("back_tire_pitch",),
            scale=4.0,
        ),
        # fork: position アクチュエータ（位置制御）
        # 現在は固定（60deg）のためコメントアウト
        # fork を動かす場合は JointPositionActionCfg のimportも解除すること
        # "fork_motor": JointPositionActionCfg(
        #     entity_name="bike",
        #     actuator_names=("fork_yaw",),
        #     scale=math.radians(90),
        # ),
    }

    # ── Rewards ─────────────────────────────────────────────────────
    rewards = {
        "body_roll": RewardTermCfg(
            func=body_roll_reward,
            weight=1.0,
            params={
                "accel_sensor_name": ACCEL,
                "margin": math.radians(15.0),
            },
        ),
        "body_roll_vel": RewardTermCfg(
            func=body_roll_vel_penalty,
            weight=-0.1,
            params={
                "gyro_sensor_name": GYRO,
                "margin": 1.0,
            },
        ),
        # "back_tire_vel": RewardTermCfg(
        #     func=back_tire_vel_penalty,
        #     weight=-0.05,
        #     params={"asset_cfg": back_tire_cfg, "margin": 10.0},
        # ),
    }

    # ── Terminations ────────────────────────────────────────────────
    terminations = {
        "roll_exceeded": TerminationTermCfg(
            func=roll_exceeded,
            params={
                "accel_sensor_name": ACCEL,
                "limit_rad": math.radians(45.0),
            },
        ),
        "time_out": TerminationTermCfg(
            func=time_out,
            time_out=True,
        ),
    }

    # ── Events ──────────────────────────────────────────────────────
    # [FIX1] mode="reset" に修正
    events = {
        "reset_scene": EventTermCfg(
            func=reset_scene_to_default,
            mode="reset",
        ),
        "reset_fork": EventTermCfg(
            func=reset_joints_by_offset,
            mode="reset",                  
            params={
                "position_range": (0.0, 0.0),
                "velocity_range": (0.0, 0.0),
                "asset_cfg": SceneEntityCfg("bike", joint_names=("fork_yaw",)),
            },
        ),
        "reset_fork_target": EventTermCfg(
            func=set_joint_position_target,
            mode="reset",
            params={
                "target_position": math.radians(60),
                "asset_cfg": SceneEntityCfg("bike", joint_names=("fork_yaw",)),
            },
        ),
        "reset_back_tire": EventTermCfg(
            func=reset_joints_by_offset,
            mode="reset",
            params={
                "position_range": (0.0, 0.0),
                "velocity_range": (-0.01, 0.01),
                "asset_cfg": SceneEntityCfg("bike", joint_names=("back_tire_pitch",)),
            },
        ),
    }

    return ManagerBasedRlEnvCfg(
        scene=SceneCfg(
            terrain=TerrainEntityCfg(terrain_type="plane"),
            entities={"bike": _get_bike_entity_cfg()},
            num_envs=num_envs,
            env_spacing=4.0,
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
        decimation=5,
        episode_length_s=50.0,
    )
