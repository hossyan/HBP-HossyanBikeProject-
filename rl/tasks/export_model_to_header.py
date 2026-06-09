"""
export_model_to_header.py
==========================
学習済みの .pt ファイルから ESP32 向けの .h / .cpp を生成する。

使い方:
    python rl/tasks/export_model_to_header.py --checkpoint rl/tasks/logs/bike_balance/log17/model_1999.pt
    python export_model_to_header.py --checkpoint logs/bike-balance/model_499.pt --obs-dim 3 --act-dim 1

出力:
    policy.h   : 重み配列の宣言 + 推論関数のプロトタイプ
    policy.cpp : 重み配列の定義 + 推論関数の実装

ESP32 での使い方:
    #include "policy.h"
    float obs[OBS_DIM] = {roll, roll_rate, steer};
    float action = policy_infer(obs);
"""

import argparse
import struct
import torch
import torch.nn.functional as F
from pathlib import Path


# ============================================================
# ユーティリティ
# ============================================================

def load_actor_weights(checkpoint_path: str):
    """
    チェックポイントから actor の重みを取得する。
    戻り値: [(W0, b0), (W1, b1), ...] の順序付きリスト
    """
    ck = torch.load(checkpoint_path, map_location="cpu")
    sd = ck["actor_state_dict"]

    # mlp.0, mlp.2, mlp.4 ... の順で Linear 層を取り出す
    layers = []
    idx = 0
    while True:
        w_key = f"mlp.{idx}.weight"
        b_key = f"mlp.{idx}.bias"
        if w_key not in sd:
            break
        W = sd[w_key].float().numpy()   # shape: [out, in]
        b = sd[b_key].float().numpy()   # shape: [out]
        layers.append((W, b))
        idx += 2  # Linear の次は活性化関数（パラメータなし）なので +2

    print(f"[export] チェックポイント: {checkpoint_path}")
    print(f"[export] 層数: {len(layers)}")
    for i, (W, b) in enumerate(layers):
        print(f"  layer{i}: ({W.shape[1]}, {W.shape[0]})")

    return layers


def array_to_c_float(arr, name: str, indent: str = "    ") -> str:
    """numpy 配列を C の float 配列リテラルに変換する。"""
    flat = arr.flatten().tolist()
    vals = ", ".join(f"{v:.8f}f" for v in flat)
    rows, cols = arr.shape if arr.ndim == 2 else (1, arr.shape[0])
    return f"static const float {name}[{rows * cols}] = {{\n{indent}{vals}\n}};"


# ============================================================
# .h ファイル生成
# ============================================================

def generate_header(layers, obs_dim: int, act_dim: int) -> str:
    n_layers = len(layers)
    hidden_dims = [W.shape[0] for W, _ in layers[:-1]]
    hidden_str = ", ".join(str(d) for d in hidden_dims)

    lines = []
    lines.append("#pragma once")
    lines.append("")
    lines.append("/*")
    lines.append(" * policy.h")
    lines.append(" * =========")
    lines.append(" * 学習済みポリシーの推論関数。")
    lines.append(" *")
    lines.append(f" * ネットワーク構造: {obs_dim} -> {hidden_str} -> {act_dim}")
    lines.append(" * 活性化関数: ELU（最終層のみ線形）")
    lines.append(" *")
    lines.append(" * 使い方:")
    lines.append(" *   #include \"policy.h\"")
    lines.append(f" *   float obs[OBS_DIM] = {{...}};  // 観測値 ({obs_dim}次元)")
    lines.append(" *   float action = policy_infer(obs);")
    lines.append(" */")
    lines.append("")
    lines.append("#ifdef __cplusplus")
    lines.append('extern "C" {')
    lines.append("#endif")
    lines.append("")
    lines.append(f"#define OBS_DIM {obs_dim}")
    lines.append(f"#define ACT_DIM {act_dim}")
    lines.append(f"#define N_LAYERS {n_layers}")
    lines.append("")

    # 各層の次元定数
    for i, (W, b) in enumerate(layers):
        in_dim  = W.shape[1]
        out_dim = W.shape[0]
        lines.append(f"#define LAYER{i}_IN  {in_dim}")
        lines.append(f"#define LAYER{i}_OUT {out_dim}")

    lines.append("")
    lines.append("/**")
    lines.append(" * ポリシー推論関数。")
    lines.append(" * @param obs  観測値の配列 (長さ OBS_DIM)")
    lines.append(" * @return     アクション値 (スカラー, [-1, 1] 付近)")
    lines.append(" */")
    lines.append("float policy_infer(const float* obs);")
    lines.append("")
    lines.append("#ifdef __cplusplus")
    lines.append("}")
    lines.append("#endif")

    return "\n".join(lines)


# ============================================================
# .cpp ファイル生成
# ============================================================

def generate_cpp(layers, obs_dim: int, act_dim: int) -> str:
    lines = []
    lines.append('#include "policy.h"')
    lines.append("#include <math.h>")
    lines.append("")
    lines.append("/* ---- 重み配列 ---- */")
    lines.append("")

    for i, (W, b) in enumerate(layers):
        in_dim  = W.shape[1]
        out_dim = W.shape[0]

        # Weight 行列（行優先: W[out][in]）
        flat_w = W.flatten().tolist()
        w_vals = ", ".join(f"{v:.8f}f" for v in flat_w)
        lines.append(f"/* layer{i} weight: [{out_dim}][{in_dim}] (row-major) */")
        lines.append(f"static const float W{i}[{out_dim * in_dim}] = {{")
        # 行ごとに改行して見やすくする
        for row_idx in range(out_dim):
            row = W[row_idx].tolist()
            row_str = ", ".join(f"{v:.8f}f" for v in row)
            comma = "," if row_idx < out_dim - 1 else ""
            lines.append(f"    /* [{row_idx}] */ {row_str}{comma}")
        lines.append("};")
        lines.append("")

        # Bias
        flat_b = b.flatten().tolist()
        b_vals = ", ".join(f"{v:.8f}f" for v in flat_b)
        lines.append(f"/* layer{i} bias: [{out_dim}] */")
        lines.append(f"static const float b{i}[{out_dim}] = {{")
        lines.append(f"    {b_vals}")
        lines.append("};")
        lines.append("")

    # ELU 関数
    lines.append("/* ---- ELU 活性化関数 ---- */")
    lines.append("static inline float elu(float x) {")
    lines.append("    return x >= 0.0f ? x : (expf(x) - 1.0f);")
    lines.append("}")
    lines.append("")

    # 推論関数
    # バッファサイズ = 最大隠れ層次元
    max_hidden = max(W.shape[0] for W, _ in layers)
    lines.append("/* ---- 推論関数 ---- */")
    lines.append("float policy_infer(const float* obs) {")
    lines.append(f"    float buf_a[{max_hidden}];")
    lines.append(f"    float buf_b[{max_hidden}];")
    lines.append("    const float* in_ptr  = obs;")
    lines.append("    float*       out_ptr = buf_a;")
    lines.append("")

    for i, (W, b) in enumerate(layers):
        in_dim  = W.shape[1]
        out_dim = W.shape[0]
        is_last = (i == len(layers) - 1)

        lines.append(f"    /* layer{i}: {in_dim} -> {out_dim} */")
        lines.append(f"    for (int o = 0; o < {out_dim}; o++) {{")
        lines.append(f"        float sum = b{i}[o];")
        lines.append(f"        for (int j = 0; j < {in_dim}; j++) {{")
        lines.append(f"            sum += W{i}[o * {in_dim} + j] * in_ptr[j];")
        lines.append(f"        }}")
        if is_last:
            lines.append(f"        out_ptr[o] = sum;  /* 最終層: 線形（活性化なし）*/")
        else:
            lines.append(f"        out_ptr[o] = elu(sum);")
        lines.append(f"    }}")
        lines.append("")

        if not is_last:
            # バッファを入れ替える
            if i % 2 == 0:
                lines.append("    in_ptr  = buf_a;")
                lines.append("    out_ptr = buf_b;")
            else:
                lines.append("    in_ptr  = buf_b;")
                lines.append("    out_ptr = buf_a;")
            lines.append("")

    lines.append("    return out_ptr[0];  /* ACT_DIM=1 なのでスカラーで返す */")
    lines.append("}")

    return "\n".join(lines)


# ============================================================
# メイン
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="Export .pt model to C header")
    parser.add_argument("--checkpoint", type=str, required=True,
                        help="学習済みモデルのパス (.pt)")
    parser.add_argument("--obs-dim", type=int, default=None,
                        help="観測次元（省略時はチェックポイントから自動取得）")
    parser.add_argument("--act-dim", type=int, default=1,
                        help="行動次元（デフォルト: 1）")
    parser.add_argument("--out-dir", type=str, default=".",
                        help="出力先ディレクトリ")
    args = parser.parse_args()

    # 重みを読み込む
    layers = load_actor_weights(args.checkpoint)

    # obs_dim を自動取得（最初の層の入力次元）
    obs_dim = args.obs_dim or layers[0][0].shape[1]
    act_dim = args.act_dim

    print(f"[export] obs_dim: {obs_dim}, act_dim: {act_dim}")

    # 出力先
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # .h を生成
    h_path = out_dir / "policy.h"
    h_path.write_text(generate_header(layers, obs_dim, act_dim), encoding="utf-8")
    print(f"[export] 生成: {h_path}")

    # .cpp を生成
    cpp_path = out_dir / "policy.cpp"
    cpp_path.write_text(generate_cpp(layers, obs_dim, act_dim), encoding="utf-8")
    print(f"[export] 生成: {cpp_path}")

    print("\n[export] 完了！")
    print(f"  ESP32 プロジェクトに {h_path} と {cpp_path} をコピーしてください。")
    print("  使い方:")
    print('    #include "policy.h"')
    print(f'    float obs[{obs_dim}] = {{roll, roll_rate, steer}};')
    print('    float action = policy_infer(obs);')


if __name__ == "__main__":
    main()