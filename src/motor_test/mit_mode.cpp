// // bike_pid_cyber_mit.cpp
// // back_motor を CyberGear の運控モード（MITモード, run_mode = 0）で動かす版。
// // front_motor は元のコードと同じ位置モードのまま。
// //
// // 制御則（back_motor）:
// //   ドライバ内部 : tau = Kp_mit*(p_set - p) + Kd*(v_set - v) + t_ff
// //   本コードでは Kp_mit = 0 にして、位置項を外部で計算し t_ff に入れる（方法A）
// //     t_ff = kp_ext * e_pos （後で角度依存FFを足す）
// //     e_pos は「目標の進み − 実際の進み」を毎周期足し込んだ位置偏差。絶対角は持たない。
// //
// // [追加] / [変更] のコメントが元コードからの変更点。

// #include <M5Unified.h>
// #include <mcp_can.h>
// #include <SPI.h>
// #include <PS4Controller.h>
// #include <math.h>

// // --- ピン・ハードウェア設定 ---
// #define CAN0_INT 15
// const int SPI_CS_PIN = 27;
// MCP_CAN CAN0(SPI_CS_PIN);
// long unsigned int rxId;
// unsigned char len = 0;
// unsigned char buf[8];

// // --- モーター基本設定 ---
// #define FRONT_MOTOR_ID  0x7E
// #define BACK_MOTOR_ID   0x7F
// #define MASTER_ID       0x00

// // --- CyberGear 通信タイプ (拡張ID bit28〜24) ---
// #define MODE_MOTION_CONTROL   0x01   // [追加] 運控モード制御指令（通信タイプ1）
// #define MODE_FEEDBACK         0x02   // [追加] モータからのフィードバック（通信タイプ2）
// #define MODE_MOTOR_ENABLE     0x03
// #define MODE_MOTOR_STOP       0x04   // [追加] 停止（通信タイプ4, Byte0=1で故障クリア）
// #define MODE_SET_ZERO_POS     0x06
// #define MODE_PARAM_WRITE      0x12

// // --- CyberGear 内部レジスタインデックス ---
// #define INDEX_RUN_MODE        0x7005 // 0:運控, 1:位置, 2:速度, 3:電流
// #define INDEX_TARGET_CUR      0x7006 // 目標電流 (float, A)
// #define INDEX_TARGET_SPD      0x700A // 目標速度 (float, rad/s)
// #define INDEX_LIMIT_TORQUE    0x700B // [追加] トルク制限 (float, 0〜12 Nm)
// #define INDEX_TARGET_POS      0x7016 // 目標位置 (float, rad)

// // --- モード定義 ---
// #define CONTROL_MODE_MIT      0      // [追加] 運控モード
// #define CONTROL_MODE_POS      1
// #define CONTROL_MODE_SPD      2
// #define CONTROL_MODE_CUR      3

// // --- [追加] 運控モードの値の範囲（各値を 16bit 0〜65535 に線形で割り当てる） ---
// // 角度の範囲はマニュアル記載の ±4π を採用。
// // 一部のライブラリは ±12.5 を使っている（差は約0.5%）。実機で違いが見えたらここを 12.5f に変える。
// const float POS_RANGE = 4.0f * (float)M_PI;  // ±4π [rad]
// const float P_MIN  = -POS_RANGE, P_MAX  = POS_RANGE;
// const float V_MIN  = -30.0f,     V_MAX  = 30.0f;   // [rad/s]
// const float KP_MIN = 0.0f,       KP_MAX = 500.0f;  // [Nm/rad]
// const float KD_MIN = 0.0f,       KD_MAX = 5.0f;    // [Nm·s/rad]
// const float T_MIN  = -12.0f,     T_MAX  = 12.0f;   // [Nm]
// const float POS_LSB = (P_MAX - P_MIN) / 65535.0f;  // フィードバック角度 1カウントの大きさ [rad]

// // --- 制御目標値 (front) ---
// float front_motor_target = 60 * M_PI / 180.0f; // 60degree in radian
// float offset_pos = 0.0f;

// bool power_on = false;

// // --- [追加] back_motor 運控モードの設定 ---
// const unsigned long CONTROL_PERIOD_US = 2000;  // 制御周期 2 ms (500 Hz)
// float w_ref_set = 0.2f;        // 目標速度 [rad/s]   PS4 右/左 で ±W_REF_INC
// float kp_ext    = 0.0f;        // 外部の位置ゲイン [Nm/rad]  PS4 上/下
// float kd_mit    = 0.0f;        // ドライバ内の Kd [Nm·s/rad]  PS4 △/×
// const float W_REF_INC  = 0.1f;
// const float KP_EXT_INC = 1.0f;
// const float KD_INC     = 0.02f;
// const float KP_EXT_MAX = 100.0f;
// const float W_REF_MAX  = 5.0f;

// // 安全設定
// const float TAU_LIMIT   = 3.0f;   // t_ff の上限と limit_torque [Nm]（慣れるまで小さめ）
// const float E_MAX       = 0.5f;   // 位置偏差がこれを超えたら停止 [rad]
// const float REZERO_POS  = 9.0f;   // フィードバック角がこれを超えたらゼロ点を再設定 [rad]
// const float JUMP_DETECT = 1.0f;   // 1フレームでこれ以上動いたら「ゼロ点の付け替え」とみなす [rad]
// const unsigned long REZERO_TIMEOUT_US = 100000; // ゼロ点再設定が反映されるまでの待ち上限

// // back_motor の状態
// struct BackMotorState {
//     bool     fb_valid = false;     // フィードバックを1回以上受けたか
//     uint16_t raw_prev = 0;         // 前回の角度（生の16bit値）
//     float    pos = 0.0f;           // 現在角 [rad]（ゼロ点からの角度, ±4π で飽和）
//     float    vel = 0.0f;           // 速度 [rad/s]
//     float    trq = 0.0f;           // トルク [Nm]
//     uint8_t  fault = 0;            // 故障ビット（ID bit21〜16）
//     uint8_t  run_state = 0;        // 0:リセット 1:キャリブレーション 2:運転（ID bit23〜22）
//     float    d_accum = 0.0f;       // 前回の制御周期以降の移動量 [rad]
//     double   pos_total = 0.0;      // 累積角（ログ・確認用）[rad]。大きくなるので double（float だと丸め誤差が溜まる）
//     unsigned long last_fb_us = 0;
//     bool     rezero_pending = false;
//     unsigned long rezero_sent_us = 0;
//     bool     enabled = false;
// };
// BackMotorState back;
// float e_pos = 0.0f;               // 位置偏差 [rad]
// float t_ff_cmd = 0.0f;            // 送ったトルクFF [Nm]
// bool  trip_request = false;
// const char* trip_reason = "-";
// unsigned long last_control_us = 0;

// //  関数プロトタイプ
// void init_can();
// void enable_motor(uint8_t motor_id);
// void stop_motor(uint8_t motor_id, uint8_t clear_fault);                   // [追加]
// void set_zero_position(uint8_t motor_id);                                 // [追加]
// void send_parameter_write(uint8_t motor_id, uint16_t param_index, float value, uint8_t is_byte = 0);
// void send_motion_control(uint8_t motor_id, float torque, float pos,
//                          float vel, float kp_m, float kd_m);              // [追加]
// void control_position(uint8_t motor_id, float rad);
// void control_current(uint8_t motor_id, float ampere);
// void control_velocity(uint8_t motor_id, float rad_s);
// void change_mode(uint8_t motor_id, uint8_t mode);
// float uint_to_float(uint16_t x, float x_min, float x_max, int bits);
// uint16_t float_to_uint(float x, float x_min, float x_max, int bits);     // [追加]
// void receive_feedback();                                                  // [追加]
// void handle_back_feedback(uint32_t cleanId, const unsigned char* b);      // [追加]
// void back_control_step(float dt);                                         // [追加]
// void request_trip(const char* reason);                                    // [追加]
// float clampf(float x, float lo, float hi);                                // [追加]

// struct TaskTimer {
//     unsigned long last_time = 0;
//     unsigned long interval;

//     TaskTimer(unsigned long iv) : interval(iv) {}

//     bool check() {
//         unsigned long now = micros();
//         if (now - last_time >= interval) {
//             last_time = now;
//             return true;
//         }
//         return false;
//     }
// };
// TaskTimer task100ms(100000);              // 100ms周期（表示）
// TaskTimer task10ms(10000);                // 10ms周期（シリアルログ）
// TaskTimer taskControl(CONTROL_PERIOD_US); // [追加] 制御周期

// struct PS4ButtonEdge {
//     bool last_state = false;

//     bool isPressedOnce(bool current_state) {
//         if(!last_state && current_state) {
//             last_state = current_state;
//             return true;
//         }
//         last_state = current_state;
//         return false;
//     }
// };
// PS4ButtonEdge btnUp, btnDown, btnLeft, btnRight, btnTriangle, btnCross, btnCircle;

// void setup() {
//     auto cfg = M5.config();
//     M5.begin(cfg);
//     M5.Imu.begin();
//     Serial.begin(115200);
//     PS4.begin("08:F9:E0:F5:E7:D6");
//     init_can();
//     delay(100);

//     M5.Display.setTextSize(3);

//     // [追加] back_motor を運控モードにする。モード変更は停止中に行う必要があるので先に停止。
//     stop_motor(BACK_MOTOR_ID, 0);
//     delay(10);
//     change_mode(BACK_MOTOR_ID, CONTROL_MODE_MIT);
//     send_parameter_write(BACK_MOTOR_ID, INDEX_LIMIT_TORQUE, TAU_LIMIT, 0);
//     delay(10);

//     // モーターの初期化
//     enable_motor(FRONT_MOTOR_ID);
//     enable_motor(BACK_MOTOR_ID);
//     back.enabled = true;
//     delay(100);

//     unsigned long timeout = millis();
//     while (true) {
//         if (millis() - timeout > 3000) break;
//         if (CAN0.checkReceive() == CAN_MSGAVAIL) {
//             CAN0.readMsgBuf(&rxId, &len, buf);

//             uint32_t cleanId = rxId & 0x1FFFFFFF;
//             uint8_t source_motor_id = (cleanId >> 8) & 0xFF;
//             uint8_t mode = (cleanId >> 24) & 0x1F;

//             if (mode == MODE_FEEDBACK && source_motor_id == FRONT_MOTOR_ID) {
//                 uint16_t pos_raw = (buf[0] << 8) | buf[1];
//                 offset_pos = uint_to_float(pos_raw, P_MIN, P_MAX, 16);   // [変更] ±12.5 → ±4π
//                 break;
//             }
//         }
//         delay(1);
//     }

//     change_mode(FRONT_MOTOR_ID, CONTROL_MODE_POS);
//     // [変更] back_motor は速度モードにしない（運控モードのまま）
// }

// void loop() {
//     M5.update();

//     // cybergearの角度・角速度取得
//     receive_feedback();

//     // PS4コントローラの入力処理
//     if(PS4.isConnected()) {
//         if(btnUp.isPressedOnce(PS4.Up()))             kp_ext += KP_EXT_INC;
//         if(btnDown.isPressedOnce(PS4.Down()))         kp_ext -= KP_EXT_INC;
//         if(btnRight.isPressedOnce(PS4.Right()))       w_ref_set += W_REF_INC;
//         if(btnLeft.isPressedOnce(PS4.Left()))         w_ref_set -= W_REF_INC;
//         if(btnTriangle.isPressedOnce(PS4.Triangle())) kd_mit += KD_INC;
//         if(btnCross.isPressedOnce(PS4.Cross()))       kd_mit -= KD_INC;
//         if(btnCircle.isPressedOnce(PS4.Circle())) {
//             power_on = !power_on;
//             if (power_on) {
//                 // [追加] 停止していたら（安全停止の後など）故障をクリアして再び有効化
//                 if (!back.enabled || back.fault) {
//                     stop_motor(BACK_MOTOR_ID, 1);
//                     enable_motor(BACK_MOTOR_ID);
//                     back.enabled = true;
//                     back.fb_valid = false;  // 停止中に手で回された分を「飛び」と誤判定しない
//                     back.rezero_pending = false;
//                 }
//                 e_pos = 0.0f;       // 現在位置から追従を始める
//                 trip_reason = "-";
//             }
//             M5.Display.setCursor(0, 10);
//             M5.Display.printf("Power: %d\n", power_on);
//         }
//         kp_ext    = clampf(kp_ext, 0.0f, KP_EXT_MAX);
//         kd_mit    = clampf(kd_mit, KD_MIN, KD_MAX);
//         w_ref_set = clampf(w_ref_set, -W_REF_MAX, W_REF_MAX);
//         if (fabsf(w_ref_set) < 1e-4f) w_ref_set = 0.0f;
//     } else if (power_on) {
//         // [追加] モータ側に通信途絶時の自動停止がないので、コントローラが切れたら止める
//         request_trip("PS4");
//     }

//     // ディスプレイ表示
//     if(!power_on) {
//         if(task100ms.check()) {
//             M5.Display.setCursor(0, 10);
//             M5.Display.printf("Power: %d\n", power_on);
//             M5.Display.printf("Kp: %7.3f\n", kp_ext);
//             M5.Display.printf("Kd: %7.3f\n", kd_mit);
//             M5.Display.printf("w : %7.3f\n", w_ref_set);
//             M5.Display.printf("Trip: %-6s\n", trip_reason);
//         }
//     }

//     // [変更] モータへの指令は一定周期で送る（運控モードは周期 dt を使って位置偏差を更新するため）
//     if (taskControl.check()) {
//         unsigned long now = micros();
//         float dt = (last_control_us == 0) ? CONTROL_PERIOD_US * 1e-6f
//                                           : (now - last_control_us) * 1e-6f;
//         last_control_us = now;

//         control_position(FRONT_MOTOR_ID, front_motor_target + offset_pos);
//         back_control_step(dt);
//     }

//     // [変更] ログは 10 ms ごと（毎ループ出力すると 115200bps では送信待ちで制御周期が乱れる）
//     if (task10ms.check()) {
//         Serial.printf("%lu, %.3f, %.4f, %.5f, %.3f, %.4f\n",
//                       micros() / 1000UL,
//                       power_on ? w_ref_set : 0.0f,  // 目標速度
//                       back.vel,                     // 速度フィードバック
//                       e_pos,                        // 位置偏差
//                       t_ff_cmd,                     // 送ったトルクFF
//                       (float)back.pos_total);       // 累積角

//      // teleplot
//         // Serial.print(">back_spd:");
//         // Serial.println(back.vel);
//     }
// }

// // ============================================================
// // [追加] フィードバック受信
// // ============================================================
// void receive_feedback() {
//     while (CAN0.checkReceive() == CAN_MSGAVAIL) {
//         CAN0.readMsgBuf(&rxId, &len, buf);
//         uint32_t cleanId = rxId & 0x1FFFFFFF;
//         uint8_t source_motor_id = (cleanId >> 8) & 0xFF;
//         uint8_t mode = (cleanId >> 24) & 0x1F;

//         if (mode != MODE_FEEDBACK) continue;
//         if (source_motor_id == BACK_MOTOR_ID) {
//             handle_back_feedback(cleanId, buf);
//         }
//         // front のフィードバックは元コードでも loop 内では使っていないので読み捨てる
//     }
// }

// // フィードバック（通信タイプ2）
// //   ID  : bit28〜24=2, bit23〜22=運転状態, bit21〜16=故障, bit15〜8=モータID
// //   Data: Byte0〜1 角度, Byte2〜3 速度, Byte4〜5 トルク, Byte6〜7 温度×10（上位バイトが先）
// void handle_back_feedback(uint32_t cleanId, const unsigned char* b) {
//     uint16_t pos_raw = (b[0] << 8) | b[1];
//     uint16_t spd_raw = (b[2] << 8) | b[3];
//     uint16_t trq_raw = (b[4] << 8) | b[5];

//     back.pos = uint_to_float(pos_raw, P_MIN, P_MAX, 16);
//     back.vel = uint_to_float(spd_raw, V_MIN, V_MAX, 16);
//     back.trq = uint_to_float(trq_raw, T_MIN, T_MAX, 16);
//     back.fault     = (cleanId >> 16) & 0x3F;
//     back.run_state = (cleanId >> 22) & 0x03;

//     unsigned long now = micros();
//     if (back.fb_valid) {
//         // 生の値の差で移動量を求める（量子化誤差は差を足し合わせると打ち消し合い、溜まらない）
//         float d = ((int32_t)pos_raw - (int32_t)back.raw_prev) * POS_LSB;
//         if (fabsf(d) > JUMP_DETECT) {
//             if (back.rezero_pending) {
//                 // ゼロ点を付け替えた直後の飛び → このフレーム間の移動は速度×時間で埋める
//                 float dt_fb = (now - back.last_fb_us) * 1e-6f;
//                 d = back.vel * dt_fb;
//                 back.rezero_pending = false;
//             } else {
//                 request_trip("JUMP");   // 想定外の角度の飛び
//                 d = 0.0f;
//             }
//         }
//         back.d_accum   += d;
//         back.pos_total += d;
//     }
//     back.raw_prev   = pos_raw;
//     back.last_fb_us = now;
//     back.fb_valid   = true;
// }

// // ============================================================
// // [追加] back_motor の制御（CONTROL_PERIOD_US ごと）
// // ============================================================
// void back_control_step(float dt) {
//     // 1) 安全停止の処理
//     if (back.fault) request_trip("FAULT");
//     if (trip_request) {
//         stop_motor(BACK_MOTOR_ID, 0);
//         back.enabled = false;
//         back.fb_valid = false;
//         back.rezero_pending = false;
//         power_on = false;
//         trip_request = false;
//         e_pos = 0.0f;
//         t_ff_cmd = 0.0f;
//         back.d_accum = 0.0f;
//         return;
//     }
//     if (!back.enabled) {           // 停止中は指令を送らない
//         back.d_accum = 0.0f;
//         return;
//     }

//     // 2) 角度が飽和する（±4π付近）前にゼロ点を付け替える
//     if (back.fb_valid && !back.rezero_pending && fabsf(back.pos) > REZERO_POS) {
//         set_zero_position(BACK_MOTOR_ID);
//         back.rezero_pending = true;
//         back.rezero_sent_us = micros();
//     }
//     if (back.rezero_pending && (micros() - back.rezero_sent_us > REZERO_TIMEOUT_US)) {
//         back.rezero_pending = false;
//         request_trip("REZERO");     // 運転中のゼロ点設定が反映されなかった
//         return;                     // 次の周期で停止処理
//     }

//     // 3) 位置偏差の更新：e += 目標の進み − 実際の進み
//     float d = back.d_accum;
//     back.d_accum = 0.0f;
//     float w_ref = power_on ? w_ref_set : 0.0f;
//     if (power_on) {
//         e_pos += w_ref * dt - d;
//         if (fabsf(e_pos) > E_MAX) {
//             request_trip("E_MAX");
//             return;
//         }
//     } else {
//         e_pos = 0.0f;               // 待機中は偏差を溜めない
//     }

//     // 4) トルクFF = 位置項（+ 後で角度依存FFを足す）
//     float tau = kp_ext * e_pos;     // + ff_table(phase)
//     t_ff_cmd = clampf(tau, -TAU_LIMIT, TAU_LIMIT);

//     // 5) 運控モード指令：Kp_mit = 0, p_set は使われないので 0
//     send_motion_control(BACK_MOTOR_ID, t_ff_cmd, 0.0f, w_ref, 0.0f, kd_mit);
// }

// void request_trip(const char* reason) {
//     trip_request = true;
//     trip_reason = reason;
// }

// // ============================================================
// // CAN 送信
// // ============================================================
// void init_can() {
//     if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
//         CAN0.setMode(MCP_NORMAL);
//     } else {
//         while(1) delay(10);
//     }
// }

// void enable_motor(uint8_t motor_id) {
//     uint32_t id = ((uint32_t)MODE_MOTOR_ENABLE << 24) | ((uint32_t)MASTER_ID << 8) | motor_id;
//     uint8_t dummy[8] = {0};
//     CAN0.sendMsgBuf(id, 1, 0, dummy);
// }

// // [追加] 停止（通信タイプ4）: ID = 4<<24 | マスタID<<8 | モータID, Byte0 = 1 で故障クリア
// void stop_motor(uint8_t motor_id, uint8_t clear_fault) {
//     uint32_t id = ((uint32_t)MODE_MOTOR_STOP << 24) | ((uint32_t)MASTER_ID << 8) | motor_id;
//     uint8_t data[8] = {0};
//     data[0] = clear_fault;
//     CAN0.sendMsgBuf(id, 1, 8, data);
// }

// // [追加] 機械ゼロ設定（通信タイプ6）: Byte0 = 1。電源を切ると消える（不揮発メモリには書かれない）
// void set_zero_position(uint8_t motor_id) {
//     uint32_t id = ((uint32_t)MODE_SET_ZERO_POS << 24) | ((uint32_t)MASTER_ID << 8) | motor_id;
//     uint8_t data[8] = {0};
//     data[0] = 1;
//     CAN0.sendMsgBuf(id, 1, 8, data);
// }

// // [追加] 運控モード制御指令（通信タイプ1）
// //   ID  : bit28〜24 = 1, bit23〜8 = トルク(16bit, −12〜12 Nm), bit7〜0 = モータID
// //   Data: Byte0〜1 目標角度(−4π〜4π), Byte2〜3 目標速度(−30〜30 rad/s),
// //         Byte4〜5 Kp(0〜500), Byte6〜7 Kd(0〜5)。いずれも上位バイトが先。
// //   応答: 通信タイプ2のフィードバック
// void send_motion_control(uint8_t motor_id, float torque, float pos,
//                          float vel, float kp_m, float kd_m) {
//     uint16_t t_raw  = float_to_uint(torque, T_MIN,  T_MAX,  16);
//     uint16_t p_raw  = float_to_uint(pos,    P_MIN,  P_MAX,  16);
//     uint16_t v_raw  = float_to_uint(vel,    V_MIN,  V_MAX,  16);
//     uint16_t kp_raw = float_to_uint(kp_m,   KP_MIN, KP_MAX, 16);
//     uint16_t kd_raw = float_to_uint(kd_m,   KD_MIN, KD_MAX, 16);

//     uint32_t id = ((uint32_t)MODE_MOTION_CONTROL << 24) | ((uint32_t)t_raw << 8) | motor_id;

//     uint8_t data[8];
//     data[0] = (uint8_t)(p_raw >> 8);  data[1] = (uint8_t)(p_raw & 0xFF);
//     data[2] = (uint8_t)(v_raw >> 8);  data[3] = (uint8_t)(v_raw & 0xFF);
//     data[4] = (uint8_t)(kp_raw >> 8); data[5] = (uint8_t)(kp_raw & 0xFF);
//     data[6] = (uint8_t)(kd_raw >> 8); data[7] = (uint8_t)(kd_raw & 0xFF);
//     CAN0.sendMsgBuf(id, 1, 8, data);
// }

// // パラメータ書き込み（通信タイプ18）: Byte0〜1 インデックス, Byte4〜7 値（どちらも下位バイトが先）
// void send_parameter_write(uint8_t motor_id, uint16_t param_index, float value, uint8_t is_byte) {
//     uint32_t id = ((uint32_t)MODE_PARAM_WRITE << 24) | ((uint32_t)MASTER_ID << 8) | motor_id;
//     uint8_t data[8] = {0};

//     data[0] = param_index & 0xFF;
//     data[1] = (param_index >> 8) & 0xFF;

//     if (is_byte) {
//         data[4] = (uint8_t)value;
//     } else {
//         memcpy(&data[4], &value, 4);
//     }
//     CAN0.sendMsgBuf(id, 1, 8, data);
// }

// void control_position(uint8_t motor_id, float rad) {
//     send_parameter_write(motor_id, INDEX_TARGET_POS, rad, 0);
// }

// void control_current(uint8_t motor_id, float ampere) {
//     send_parameter_write(motor_id, INDEX_TARGET_CUR, ampere, 0);
// }

// void control_velocity(uint8_t motor_id, float rad_s) {
//     send_parameter_write(motor_id, INDEX_TARGET_SPD, rad_s, 0);
// }

// void change_mode(uint8_t motor_id, uint8_t mode) {
//     send_parameter_write(motor_id, INDEX_RUN_MODE, (float)mode, 1);
//     delay(50);
// }

// // ============================================================
// // 変換
// // ============================================================
// float uint_to_float(uint16_t x, float x_min, float x_max, int bits) {
//     float span = x_max - x_min;
//     float offset = x_min;
//     return (float)x * span / ((1 << bits) - 1) + offset;
// }

// // [追加] float → 16bit（範囲外は端に丸め、最も近い整数に四捨五入）
// uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
//     x = clampf(x, x_min, x_max);
//     float span = x_max - x_min;
//     return (uint16_t)lroundf((x - x_min) * (float)((1 << bits) - 1) / span);
// }

// float clampf(float x, float lo, float hi) {
//     if (x < lo) return lo;
//     if (x > hi) return hi;
//     return x;
// }