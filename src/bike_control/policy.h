#pragma once

/*
 * policy.h
 * =========
 * 学習済みポリシーの推論関数。
 *
 * ネットワーク構造: 10 -> 128, 128, 128 -> 1
 * 活性化関数: ELU（最終層のみ線形）
 *
 * 使い方:
 *   #include "policy.h"
 *   float obs[OBS_DIM] = {...};  // 観測値 (10次元)
 *   float action = policy_infer(obs);
 */

#ifdef __cplusplus
extern "C" {
#endif

#define OBS_DIM 10
#define ACT_DIM 1
#define N_LAYERS 4

#define LAYER0_IN  10
#define LAYER0_OUT 128
#define LAYER1_IN  128
#define LAYER1_OUT 128
#define LAYER2_IN  128
#define LAYER2_OUT 128
#define LAYER3_IN  128
#define LAYER3_OUT 1

/**
 * ポリシー推論関数。
 * @param obs  観測値の配列 (長さ OBS_DIM)
 * @return     アクション値 (スカラー, [-1, 1] 付近)
 */
float policy_infer(const float* obs);

#ifdef __cplusplus
}
#endif