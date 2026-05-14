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


_BIKE_ARTICULATION = EntityArticulationInfoCfg(
    actuators=(
        XmlActuatorCfg(target_names_expr=("slider",)),
    ),
)

# ── 初期状態: バランス課題（ポールが上を向いてスタート）
_BALANCE_INIT = EntityCfg.InitialStateCfg(
    joint_pos={"slider": 0.0, "hinge_1": 0.0},   # ポール直立
    joint_vel={".*": 0.0},
)

# ── 初期状態: スウィングアップ課題（ポールが下を向いてスタート）
_SWINGUP_INIT = EntityCfg.InitialStateCfg(
    joint_pos={"slider": 0.0, "hinge_1": math.pi},  # ポール倒れた状態
    joint_vel={".*": 0.0},
)


def _get_bike_entity_cfg(swing_up: bool = True) -> EntityCfg:
    """Entity 設定を返す。swing_up=True でスウィングアップ課題になる。"""
    return EntityCfg(
        spec_fn=_get_spec,
        articulation=_BIKE_ARTICULATION,
        init_state=_SWINGUP_INIT if swing_up else _BALANCE_INIT,
    )


# ============================================================
# 2. カスタム観測関数
# ============================================================

def pole_angle_cos_sin(env, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """
    ポール角度を cos と sin で返す（形状: [num_envs, 2]）。

    生角度ではなく cos/sin を使う理由:
      MuJoCo の unlimited hinge は角度が累積されるため、
      同じ姿勢でも値が異なる場合がある。
      cos/sin にすると何回転していても同じ姿勢は同じ値になる。
    """
    from mjlab.entity import Entity
    asset: Entity = env.scene[asset_cfg.name]
    angle = asset.data.joint_pos[:, asset_cfg.joint_ids]   # [N, 1]
    return torch.cat([torch.cos(angle), torch.sin(angle)], dim=-1)  # [N, 2]


# ============================================================
# 3. カスタム報酬関数
# ============================================================

def cartpole_smooth_reward(
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

def cartpole_env_cfg(
    swing_up: bool = True,
    num_envs: int = 1,
) -> ManagerBasedRlEnvCfg:
    """
    Cartpole 環境の完全な設定を返す。

    Parameters
    ----------
    swing_up : bool
        True → スウィングアップ課題 (hinge_1 = π からスタート)
        False → バランス課題      (hinge_1 = 0 からスタート)
    num_envs : int
        並列環境数（GPU学習時は 512 〜 4096 程度を指定する）
    """

    # ── SceneEntityCfg: 観測・報酬で「どの関節を見るか」を指定 ──────
    cart_cfg  = SceneEntityCfg("cartpole", joint_names=("slider",))
    hinge_cfg = SceneEntityCfg("cartpole", joint_names=("hinge_1",))

    # ── Observations ────────────────────────────────────────────────
    # 観測ベクトル: [cart_pos(1), cos(θ)(1), sin(θ)(1), cart_vel(1), pole_vel(1)] = 5次元
    actor_terms = {
        "cart_pos": ObservationTermCfg(
            func=joint_pos_rel,
            params={"asset_cfg": cart_cfg},
        ),
        "pole_angle": ObservationTermCfg(
            func=pole_angle_cos_sin,
            params={"asset_cfg": hinge_cfg},
        ),
        "cart_vel": ObservationTermCfg(
            func=joint_vel_rel,
            params={"asset_cfg": cart_cfg},
        ),
        "pole_vel": ObservationTermCfg(
            func=joint_vel_rel,
            params={"asset_cfg": hinge_cfg},
        ),
    }
    observations = {
        "actor":  ObservationGroupCfg(actor_terms),
        "critic": ObservationGroupCfg({**actor_terms}),  # critic は同じ観測を使う
    }

    # ── Actions ─────────────────────────────────────────────────────
    # ポリシー出力 → スライダー (cart) に力を加える
    # XmlActuator が ctrl を [-1, 1] にクランプし gear=10 をかけて MuJoCo に渡す
    actions = {
        "effort": JointEffortActionCfg(
            entity_name="cartpole",
            actuator_names=("slider",),
            scale=1.0,
        ),
    }

    # ── Rewards ─────────────────────────────────────────────────────
    rewards = {
        "smooth_reward": RewardTermCfg(
            func=cartpole_smooth_reward,
            weight=1.0,
            params={"cart_cfg": cart_cfg, "hinge_cfg": hinge_cfg},
        ),
    }

    # ── Terminations ────────────────────────────────────────────────
    # タイムアウトのみ（time_out=True で「打ち切り」扱いになり価値関数をブートストラップ）
    terminations = {
        "time_out": TerminationTermCfg(func=time_out, time_out=True),
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
                # スウィングアップ: π ± 0.1 rad、バランス: 0 ± 0.1 rad
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
            entities={"bike": _get_bike_entity_cfg(swing_up=swing_up)},
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
                timestep=0.01,
                disableflags=("contact",),  # Cartpole は接触不要なので無効化
            ),
        ),
        decimation=5,           # 物理 5 ステップに 1 回ポリシーを呼ぶ (20 Hz 制御)
        episode_length_s=50.0,  # 1 エピソードの最大長さ [秒]
    )
