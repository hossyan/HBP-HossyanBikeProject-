import torch
from mjlab.managers import SceneEntityCfg

def _tolerance(x: torch.Tensor, margin: float) -> torch.Tensor:
    return torch.exp(-0.5 * (x / margin) ** 2)

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
