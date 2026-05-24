import torch
from mjlab.managers import SceneEntityCfg
 
def set_joint_position_target(
    env,
    env_ids: torch.Tensor | None,
    target_position: float,
    asset_cfg: SceneEntityCfg,
) -> None:
    """
    指定jointのpositionアクチュエータの目標角度を設定する。
    forkを60degで固定するためのリセット時処理。
 
    Args:
        target_position: 目標角度 [rad]
        asset_cfg: 対象jointの設定
    """
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device, dtype=torch.int)
 
    asset     = env.scene[asset_cfg.name]
    joint_ids = asset_cfg.joint_ids
    if isinstance(joint_ids, list):
        joint_ids = torch.tensor(joint_ids, device=env.device)
 
    target = torch.full((len(env_ids),), target_position, device=env.device)
    asset.set_joint_position_target(target, joint_ids=joint_ids, env_ids=env_ids)