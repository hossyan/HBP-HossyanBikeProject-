"""
"MyBike-Balance" タスクが登録され、
以下のCLIコマンドが使えるようになる:
 
    uv run train Bike-Balance --env.scene.num-envs 512
    uv run play  Bike-Balance --agent random
    uv run play  Bike-Balance --agent zero
"""

from mjlab.tasks.registry import register_mjlab_task
from .env_cfg import bike_balance_env_cfg, bike_balance_runner_cfg

register_mjlab_task(
    task_id="Mjlab-Bike-Balance",
    env_cfg=bike_balance_env_cfg(num_envs=512),
    play_env_cfg=bike_balance_env_cfg(num_envs=512),
    rl_cfg=bike_balance_runner_cfg(),
)