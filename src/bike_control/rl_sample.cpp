// #include "parameters.h"
// #include <math.h>

// // AIへの入力用バッファ (5つの観測値)
// float input[5];   
// // 中間層のバッファ
// float layer1[64];
// float layer2[64];
// // AIからの出力 (右トルク, 左トルク)
// float output[2];  

// // AIの計算実行 (推論)
// void run_inference() {
//     // --- 第1層: input(5) -> layer1(64) ---
//     for (int i = 0; i < 64; i++) {
//         float sum = b1[i];
//         for (int j = 0; j < 5; j++) {
//             sum += W1[i * 5 + j] * input[j];
//         }
//         layer1[i] = tanhf(sum); 
//     }

//     // --- 第2層: layer1(64) -> layer2(64) ---
//     for (int i = 0; i < 64; i++) {
//         float sum = b2[i];
//         for (int j = 0; j < 64; j++) {
//             sum += W2[i * 64 + j] * layer1[j];
//         }
//         layer2[i] = tanhf(sum);
//     }

//     // --- 第3層: layer2(64) -> output(2) ---
//     for (int i = 0; i < 2; i++) {
//         float sum = b3[i];
//         for (int j = 0; j < 64; j++) {
//             sum += W3[i * 64 + j] * layer2[j];
//         }
//         output[i] = sum; 
//     }
// }

// // メインループ内での使用イメージ
// void loop() {
//     // 1. inputにセンサー値を流し込む (単位は rad, rad/s に合わせる)
//     input[0] = get_robot_pitch_rad();      // ロボットの傾き
//     input[1] = get_robot_pitch_vel_rads(); // ロボットの角速度
//     input[2] = get_robot_yaw_rad();        // 横回転角度
//     input[3] = get_left_wheel_vel_rads();  // 左タイヤ角速度
//     input[4] = get_right_wheel_vel_rads(); // 右タイヤ角速度

//     // 2. 推論実行
//     run_inference();

//     // 3. output[0], output[1] をモータードライバへ渡す
//     // ※ outputは「トルク」なので、モーターの特性に合わせてPWM等にスケーリングしてください
//     drive_motor_left(output[0]);
//     drive_motor_right(output[1]);

//     delay(10); // シミュレーションの dt=0.01s (10ms) に合わせる
// }