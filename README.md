# mjlab Cartpole 学習教材

mjlab (Isaac Lab API × MuJoCo Warp) を使ってCartpole（倒立振子）を深層強化学習で制御する教材です。

---

## ファイル構成

```
cartpole_mjlab/
├── cartpole.xml          ← MuJoCo モデル（添付の XML をここに置く）
├── cartpole_env_cfg.py   ← 環境設定（Entity / Obs / Reward / ...）
├── train.py              ← 学習実行スクリプト
├── play.py               ← 学習済みモデルの可視化スクリプト
└── README.md             ← この教材
```

---

## セットアップ

```bash
# mjlab のインストール
pip install mjlab

# または uv を使う場合
git clone https://github.com/mujocolab/mjlab.git && cd mjlab
uv sync
```

---

## 実行方法

### 学習

```bash
# バランス課題（ポール直立を維持）
python train.py

# スウィングアップ課題（ポールを下から振り上げて直立）
python train.py --task swingup

# 並列環境数を増やして高速学習（GPU推奨）
python train.py --num-envs 1024

# 1000 イテレーション学習
python train.py --max-iters 1000
```

### 可視化

```bash
# 学習済みモデルを Native Viewer で表示
python play.py --checkpoint logs/cartpole/XXX/model_500.pt

# ブラウザ（Viser）で表示
python play.py --checkpoint logs/cartpole/XXX/model_500.pt --viewer viser

# ランダムエージェント（環境の確認用）
python play.py --agent random

# TensorBoard で学習曲線を確認
tensorboard --logdir logs
```

---

# 第1章: サンプルコード解説

## 1-1. mjlab の全体アーキテクチャ

```
ManagerBasedRlEnvCfg（環境設定）
  │
  ├── SceneCfg          ← シーン（地形 + エンティティを配置）
  │     └── EntityCfg   ← シミュレート対象（XML + アクチュエータ + 初期状態）
  │
  ├── ObservationsCfg   ← エージェントが見るもの（→ ポリシー入力）
  ├── ActionsCfg        ← エージェントがするもの（← ポリシー出力）
  ├── RewardsCfg        ← 報酬シグナル
  ├── TerminationsCfg   ← エピソード終了条件
  └── EventsCfg         ← リセット時のランダム化など
```

Gymnasium の `env.step()` インターフェースに相当するが、
mjlab では **すべての演算が PyTorch テンソルでバッチ処理**される。
`[num_envs, ...]` の形状のテンソルが GPU 上で並列に処理される。

---

## 1-2. Entity: シミュレーション対象の定義

```python
# cartpole_env_cfg.py より

def _get_spec() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_file(str(_CARTPOLE_XML))  # XML を読み込む

_CARTPOLE_ARTICULATION = EntityArticulationInfoCfg(
    actuators=(
        XmlActuatorCfg(target_names_expr=("slider",)),  # XML の motor をそのまま使う
    ),
)

_BALANCE_INIT = EntityCfg.InitialStateCfg(
    joint_pos={"slider": 0.0, "hinge_1": 0.0},   # ポール直立
    joint_vel={".*": 0.0},                         # 速度ゼロ
)
```

### ポイント

| 要素 | 役割 |
|------|------|
| `spec_fn` | XML を MjSpec として返す関数 |
| `articulation` | アクチュエータの種類（XML定義・理想PD・組み込みなど） |
| `init_state` | エピソード開始時の関節位置・速度の基準値 |

`XmlActuatorCfg` は XML の `<motor>` 要素をそのまま使うので、
gear 比・ctrlrange などの設定は XML 側で管理できる。

---

## 1-3. Observations: 観測の定義

```python
# 5次元の観測ベクトル
# [cart_pos, cos(θ), sin(θ), cart_vel, pole_vel]
#   1次元     1次元   1次元   1次元     1次元  = 5次元

cart_cfg  = SceneEntityCfg("cartpole", joint_names=("slider",))
hinge_cfg = SceneEntityCfg("cartpole", joint_names=("hinge_1",))

actor_terms = {
    "cart_pos":   ObservationTermCfg(func=joint_pos_rel, params={"asset_cfg": cart_cfg}),
    "pole_angle": ObservationTermCfg(func=pole_angle_cos_sin, params={"asset_cfg": hinge_cfg}),
    "cart_vel":   ObservationTermCfg(func=joint_vel_rel, params={"asset_cfg": cart_cfg}),
    "pole_vel":   ObservationTermCfg(func=joint_vel_rel, params={"asset_cfg": hinge_cfg}),
}
```

### なぜ cos/sin を使うか

- MuJoCo の unlimited hinge（リミットなしのヒンジ）は角度が累積する
- ポールが 2π 回転すると生角度は 2π だが物理的には 0 と同じ姿勢
- cos/sin なら何回転しても同じ姿勢は同じ観測値になる

### `SceneEntityCfg` の役割

```python
SceneEntityCfg("cartpole", joint_names=("slider",))
```

- `"cartpole"` → scene 上のエンティティ名
- `joint_names` → 対象の関節（正規表現も使える: `".*"` で全関節）

これにより同じ観測関数(`joint_pos_rel`)でも、
`cart_cfg` と `hinge_cfg` で異なる関節を参照できる。

---

## 1-4. Actions: 行動の定義

```python
actions = {
    "effort": JointEffortActionCfg(
        entity_name="cartpole",
        actuator_names=("slider",),
        scale=1.0,
    ),
}
```

ポリシーが出力したスカラー値が `slider` アクチュエータに渡る。
`XmlActuator` は XML の `ctrlrange="-1 1"` でクランプし、
`gear=10` をかけて最大 10N の力をカートに加える。

---

## 1-5. Rewards: 報酬の定義

```python
r = upright × centered × small_ctrl × small_vel
```

各ファクターは [0, 1] の範囲。4条件が同時に満たされたときのみ高い報酬。

| ファクター | 意味 | 高い値になる条件 |
|-----------|------|----------------|
| `upright` | ポールの直立度 | cos(θ) ≈ 1 (θ ≈ 0) |
| `centered` | カートの中央度 | cart_pos ≈ 0 |
| `small_ctrl` | 制御量の小ささ | action ≈ 0 |
| `small_vel` | 角速度の小ささ | pole_vel ≈ 0 |

積算にする利点:
- 加算だと「1つだけ最大化して他を無視する」戦略が有効になりやすい
- 積算なら全条件を同時に満たさないと最大値にならない

---

## 1-6. Terminations: 終了条件

```python
terminations = {
    "time_out": TerminationTermCfg(func=time_out, time_out=True),
}
```

Cartpole の終了条件はタイムアウトのみ。
`time_out=True` は **「打ち切り（truncation）」** を意味する。

Gymnasium の `terminated` と `truncated` の区別と同じ:
- `terminated=True`: 真の終端状態（転倒など）→ 価値関数を 0 に
- `truncated=True` : 時間切れ → 価値関数をブートストラップ（次状態の V を使う）

---

## 1-7. Events: リセット時のランダム化

```python
events = {
    "reset_hinge": EventTermCfg(
        func=reset_joints_by_offset,
        mode="reset",          # エピソードリセット時に実行
        params={
            "position_range": (-0.1, 0.1),   # 初期角度に ±0.1 rad のノイズ
            "velocity_range": (-0.01, 0.01), # 初期速度に ±0.01 rad/s のノイズ
            "asset_cfg": SceneEntityCfg("cartpole", joint_names=("hinge_1",)),
        },
    ),
}
```

`mode` には3種類ある:
| mode | タイミング |
|------|----------|
| `"startup"` | 環境作成時に1回だけ |
| `"reset"` | エピソードリセットのたびに |
| `"interval"` | 指定間隔ごとに |

---

# 第2章: マネージャー解説

## 2-1. マネージャーとは

mjlab では「何かを管理するモジュール」を **マネージャー** と呼ぶ。
Isaac Lab と同じ設計思想で、環境の各機能が独立したマネージャーとして実装されている。

```
ManagerBasedRlEnv
  ├── ObservationManager   ← 観測の計算・ノイズ付加
  ├── ActionManager        ← アクションの前処理・アクチュエータへの適用
  ├── RewardManager        ← 報酬の計算・集約
  ├── TerminationManager   ← 終了条件の判定
  ├── EventManager         ← リセット・ドメインランダム化
  └── (CommandManager)     ← 速度指令など（今回は未使用）
```

各マネージャーは **TermCfg（項目設定）** の辞書を受け取り、
`env.step()` や `env.reset()` のたびに各項目の関数を呼び出す。

---

## 2-2. ObservationManager の仕組み

```
ObservationManager.compute()
  ├── term: "cart_pos"   → joint_pos_rel(env, cart_cfg)  → [N, 1]
  ├── term: "pole_angle" → pole_angle_cos_sin(env, cfg)  → [N, 2]
  ├── term: "cart_vel"   → joint_vel_rel(env, cart_cfg)  → [N, 1]
  └── term: "pole_vel"   → joint_vel_rel(env, hinge_cfg) → [N, 1]
           → torch.cat([...], dim=-1) → [N, 5]  ← ポリシー入力
```

各 term の関数は `(env, **params)` のシグネチャを持つ。
`ObservationTermCfg` の `params` が `**params` として渡される。

### ノイズの追加（発展）

```python
from mjlab.utils.noise import UniformNoiseCfg

ObservationTermCfg(
    func=joint_pos_rel,
    params={"asset_cfg": cart_cfg},
    noise=UniformNoiseCfg(n_min=-0.05, n_max=0.05),  # ±5cm のノイズ
)
```

---

## 2-3. EventManager とドメインランダム化

`EventManager` は「物理パラメータのランダム化」も担う。
`mjlab.envs.mdp.dr` モジュールに標準的なランダム化関数が用意されている。

### 例: ポールの質量をランダム化

```python
from mjlab.envs.mdp import dr
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg

events = {
    # エピソードごとにポール質量を 0.05〜0.2 kg にランダム化
    "pole_mass": EventTermCfg(
        func=dr.body_mass,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("cartpole", body_names=["pole_1"]),
            "ranges": (0.05, 0.2),
            "operation": "abs",  # "abs"=直接設定 / "scale"=倍率 / "add"=加算
        },
    ),
    # 関節ダンピングもランダム化（±50%）
    "joint_damping": EventTermCfg(
        func=dr.joint_damping,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("cartpole", joint_names=[".*"]),
            "ranges": (0.5, 1.5),
            "operation": "scale",
        },
    ),
}
```

### per-world ストレージ（mjlab の核心機能）

mjlab は何千もの環境を GPU 上で並列実行する。
通常、MuJoCo Warp はモデルパラメータを全環境で共有するが、
`EventManager` は自動的に「per-world 展開」を行う。

```
初期: geom_friction → shape (1, ngeom, 3)  ← 全環境共有
             ↓ EventManager が自動展開
展開後:      shape (num_envs, ngeom, 3)   ← 環境ごとに独立
```

これにより環境 0 の摩擦係数を変えても環境 1 には影響しない。

---

## 2-4. RewardManager の仕組み

```python
rewards = {
    "smooth_reward": RewardTermCfg(
        func=cartpole_smooth_reward,
        weight=1.0,           # 重み（複数の報酬項を加重和する）
        params={...},
    ),
    # 複数の報酬を加えることもできる
    "survival_bonus": RewardTermCfg(
        func=lambda env: torch.ones(env.num_envs, device=env.device),
        weight=0.01,
    ),
}
```

最終報酬 = Σ (weight_i × func_i(env))

---

## 2-5. 自作マネージャー関数の書き方

すべての Term 関数は共通のシグネチャ:

```python
def my_obs_term(env: ManagerBasedRlEnv, **params) -> torch.Tensor:
    #                                    ↑ ObservationTermCfg.params
    return tensor  # shape: [num_envs, dim]

def my_reward_term(env: ManagerBasedRlEnv, **params) -> torch.Tensor:
    return tensor  # shape: [num_envs]  (スカラー報酬)

def my_event_term(env: ManagerBasedRlEnv, env_ids: torch.Tensor, **params) -> None:
    #                                      ↑ リセットする環境のインデックス
    pass  # 戻り値なし
```

`env_ids` はリセット対象の環境インデックスで、
部分リセット（一部の環境だけリセット）に対応するために渡される。

---

## 第3章: 次のステップ

### 3-1. より複雑な機構への応用

このコードを他のロボットに拡張するには:

1. `_get_spec()` を変更して別の XML を読み込む
2. `EntityArticulationInfoCfg` のアクチュエータを変更
3. 観測・報酬の term 関数を新しいジョイント名に合わせる

XML さえ用意すれば、残りのコードの構造は同じ。

### 3-2. 参考リンク

- [mjlab 公式ドキュメント](https://mujocolab.github.io/mjlab/)
- [Cartpole チュートリアル](https://mujocolab.github.io/mjlab/main/source/tutorials/cartpole.html)
- [ドメインランダム化](https://mujocolab.github.io/mjlab/main/source/randomization.html)
- [マネージャー一覧](https://mujocolab.github.io/mjlab/main/source/api/managers.html)
- [RSL-RL 学習設定](https://mujocolab.github.io/mjlab/main/source/training/rsl_rl.html)
