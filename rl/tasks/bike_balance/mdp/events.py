import torch
from mjlab.managers import SceneEntityCfg

from mjlab.envs import ManagerBasedRlEnv
from mjlab.envs.mdp import dr
from mjlab.managers.event_manager import EventTermCfg, requires_model_fields
 
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

    num_joints = asset.num_joints if isinstance(joint_ids, slice) else len(joint_ids)
    target = torch.full((len(env_ids), num_joints), target_position, device=env.device)
    asset.set_joint_position_target(target, joint_ids=joint_ids, env_ids=env_ids)

@requires_model_fields("actuator_gainprm", "actuator_biasprm")
def randomize_velocity_kv(
    env: ManagerBasedRlEnv,
    env_ids: torch.Tensor | None,
    ranges: tuple[float, float],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    operation: str = "scale",
    distribution: dr.Distribution = dr.uniform,
) -> None:
    asset = env.scene[asset_cfg.name]

    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device, dtype=torch.int)
    else:
        env_ids = env_ids.to(env.device, dtype=torch.int)

    # asset_cfg.actuator_ids は resolve() 済みで entity.actuator_names への
    # ローカルインデックス。indexing.ctrl_ids が同じ順序でグローバル ID を持つ。
    local_ids = asset_cfg.actuator_ids
    ctrl_ids = asset.indexing.ctrl_ids[local_ids].long()

    lo = torch.tensor(ranges[0], device=env.device)
    hi = torch.tensor(ranges[1], device=env.device)
    samples = distribution.sample(lo, hi, (len(env_ids), len(ctrl_ids)), env.device)

    if operation == "scale":
        default_gainprm = env.sim.get_default_field("actuator_gainprm")
        kv = default_gainprm[ctrl_ids, 0] * samples
    elif operation == "abs":
        kv = samples
    else:
        raise ValueError(f"unsupported operation: {operation}")

    env.sim.model.actuator_gainprm[env_ids[:, None], ctrl_ids, 0] = kv
    env.sim.model.actuator_biasprm[env_ids[:, None], ctrl_ids, 2] = -kv