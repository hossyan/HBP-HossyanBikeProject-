import torch
 
def roll_exceeded(
    env,
    accel_sensor_name: str,
    limit_rad: float,
) -> torch.Tensor:
    """
    roll角度が閾値を超えたらエピソードを終了する。
 
    Args:
        accel_sensor_name: 加速度センサー名（例: "bike/body_accel"）
        limit_rad: roll角度の閾値 [rad]
    Returns:
        終了フラグ [N] bool
    """
    accel = env.scene[accel_sensor_name].data       # [N, 3]
    roll  = torch.atan2(accel[:, 1], accel[:, 2])  # [N]
    return torch.abs(roll) > limit_rad