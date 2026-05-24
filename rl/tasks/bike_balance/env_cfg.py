import math
from pathlib import Path

import mujoco
import torch

# ── mjlab コア ──────────────────────────────────────────────────────────
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp import (
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
from mjlab.utils.noise import GaussianNoiseCfg
from mjlab.rl import RslRlOnPolicyRunnerCfg, RslRlModelCfg, RslRlPpoAlgorithmCfg

# ── MDPコンポーネント ────────────────────────────────────────────────────
from .mdp import (
    ComplementaryRollFilter,
    RollRateObservation,
    body_roll_reward,
    body_roll_vel_penalty,
    back_tire_vel_penalty,
    roll_exceeded,
    set_joint_position_target,
)

# ============================================================
# 0. XML パス
# ============================================================
_ASSETS_DIR = Path(__file__).parent.parent.parent / "assets"
_BIKE_XML   = _ASSETS_DIR / "bike_V3_mjcf.xml"


# ============================================================
# 1. Entity 設定（静的定数として定義）
# ============================================================
def _get_spec() -> mujoco.MjSpec:
    """MuJoCo モデルスペックをXMLから読み込む。"""
    return mujoco.MjSpec.from_file(str(_BIKE_XML))


_BIKE_ARTICULATION = EntityArticulationInfoCfg(
    actuators=(
        XmlActuatorCfg(target_names_expr=("back_tire_pitch",)),  # joint名で指定
        XmlActuatorCfg(target_names_expr=("fork_yaw",)),         # joint名で指定
    ),
)

_BIKE_INIT = EntityCfg.InitialStateCfg(
    joint_pos={
        "back_tire_pitch": 0.0,
        "fork_yaw": math.radians(60),
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
# 2. 環境設定
# ============================================================

def bike_balance_env_cfg(num_envs: int = 1) -> ManagerBasedRlEnvCfg:

    # ── SceneEntityCfg（観測・報酬で参照するjointを指定）──────────────
    back_tire_cfg = SceneEntityCfg("bike", joint_names=("back_tire_pitch",))

    # ── センサー名（エンティティ名/センサー名の形式）─────────────────
    ACCEL = "bike/body_accel"
    GYRO  = "bike/body_gyro"

    # ── Observations ────────────────────────────────────────────────
    actor_terms = {
        "back_tire_vel": ObservationTermCfg(
            func=joint_vel_rel,
            params={"asset_cfg": back_tire_cfg},
            noise=GaussianNoiseCfg(mean=0.0, std=0.01),   # [rad/s]
        ),
        "body_roll": ObservationTermCfg(
            func=ComplementaryRollFilter,
            params={
                "accel_sensor_name": ACCEL,
                "gyro_sensor_name":  GYRO,
                "alpha": 0.98,
            },
            noise=GaussianNoiseCfg(mean=0.0, std=0.005),  # [rad]
        ),
        "body_roll_vel": ObservationTermCfg(
            func=RollRateObservation,
            params={
                "gyro_sensor_name": GYRO,
            },
            noise=GaussianNoiseCfg(mean=0.0, std=0.01),   # [rad/s]
        ),
    }

    observations = {
        "actor":  ObservationGroupCfg(actor_terms, enable_corruption=True),
        "critic": ObservationGroupCfg({**actor_terms}, enable_corruption=False),
    }

    # ── Actions ─────────────────────────────────────────────────────
    actions = {
        "back_tire_motor": JointEffortActionCfg(
            entity_name="bike",
            actuator_names=("back_tire_pitch",),
            scale=4.0,
        ),
        # fork: position アクチュエータ（位置制御）
        # 現在は60degで固定のためコメントアウト
        # 動かす場合は JointPositionActionCfg のimportも解除すること
        # "fork_motor": JointPositionActionCfg(
        #     entity_name="bike",
        #     actuator_names=("fork_yaw",),
        #     scale=math.radians(90),   # ctrlrange="-1.570796 1.570796"
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
                "margin": 1.0,  # [rad/s]
            },
        ),
        # タイヤ速度ペナルティ（必要に応じてコメントアウトを解除）
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
                "limit_rad": math.radians(10.0),
            },
        ),
        "time_out": TerminationTermCfg(
            func=time_out,
            time_out=True,
        ),
    }

    # ── Events ──────────────────────────────────────────────────────
    events = {
        # シーン全体をデフォルト状態にリセット
        "reset_scene": EventTermCfg(
            func=reset_scene_to_default,
            mode="reset",
        ),
        # fork: 60deg固定（ノイズなし）
        "reset_fork": EventTermCfg(
            func=reset_joints_by_offset,
            mode="reset",
            params={
                "position_range": (0.0, 0.0),
                "velocity_range": (0.0, 0.0),
                "asset_cfg": SceneEntityCfg("bike", joint_names=("fork_yaw",)),
            },
        ),
        # forkのpositionアクチュエータ目標を60degに設定
        "reset_fork_target": EventTermCfg(
            func=set_joint_position_target,
            mode="reset",
            params={
                "target_position": math.radians(60),
                "asset_cfg": SceneEntityCfg("bike", joint_names=("fork_yaw",)),
            },
        ),
        # back_tire: 初期速度にわずかなノイズ
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
            env_spacing=2.0,
        ),
        observations=observations,
        actions=actions,
        events=events,
        rewards=rewards,
        terminations=terminations,
        sim=SimulationCfg(
            mujoco=MujocoCfg(
                timestep=0.001,   # 物理ステップ: 1ms
            ),
        ),
        decimation=10,            # ポリシー周期: 1ms × 10 = 10ms (100Hz)
        episode_length_s=50.0,
    )


# ============================================================
# 3. PPO 学習設定
# ============================================================

def bike_balance_runner_cfg() -> RslRlOnPolicyRunnerCfg:
    """
    バイクバランス課題のPPO学習設定を返す。
    観測次元: back_tire_vel(1) + body_roll(1) + body_roll_vel(1) = 3次元
    行動次元: back_tire_motor(1) = 1次元
    """
    return RslRlOnPolicyRunnerCfg(
        actor=RslRlModelCfg(
            class_name="MLPModel",
            hidden_dims=(128, 64),
            activation="elu",
            distribution_cfg={
                "class_name": "GaussianDistribution",
                "init_std": 1.0,
                "std_type": "scalar",
            },
        ),
        critic=RslRlModelCfg(
            class_name="MLPModel",
            hidden_dims=(128, 64),
            activation="elu",
            distribution_cfg=None,
        ),
        algorithm=RslRlPpoAlgorithmCfg(
            value_loss_coef=1.0,
            use_clipped_value_loss=True,
            clip_param=0.2,
            entropy_coef=0.005,
            num_learning_epochs=5,
            num_mini_batches=4,
            learning_rate=1e-3,
            schedule="adaptive",
            gamma=0.99,
            lam=0.95,
            desired_kl=0.01,
            max_grad_norm=1.0,
        ),
        num_steps_per_env=24,
        max_iterations=1000,
        save_interval=200,
        experiment_name="bike_balance",
        run_name="",           # 空にすると日時から自動生成
        logger="tensorboard",
    )