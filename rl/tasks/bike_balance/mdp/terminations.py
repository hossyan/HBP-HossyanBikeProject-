import torch

def body_contact(env, sensor_name: str) -> torch.Tensor:
    found = env.scene[sensor_name].data      # [N, 1]
    return found[:, 0] > 0.0