import torch
from mjlab.managers import ObservationTermCfg, SceneEntityCfg

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