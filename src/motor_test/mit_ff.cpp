// // bike_pid_cyber_mit_ff.cpp
// // back_motor を CyberGear の運控モード（MITモード, run_mode = 0）で動かし、
// // 極数周期の角度依存トルクをフィードフォワード（FF）で打ち消す版。
// // front_motor は元のコードと同じ位置モードのまま。
// //
// // 制御則（back_motor）:
// //   ドライバ内部 : tau = Kp_mit*(p_set - p) + Kd*(v_set - v) + t_ff
// //   本コードでは Kp_mit = 0 にして、位置項とFFを外部で計算し t_ff に入れる
// //     t_ff = kp_ext * e_pos + 表1(108.5周期の位相) + 表2(186周期の位相)
// //
// // FFの作り方（同定）:
// //   □ボタンで ±0.04 rad/s の低速で 正転18秒 → 逆転18秒 回し、位相ごとにモータトルクの平均をとる。
// //   低速なら加速度がほぼ0なので「モータトルク ≒ 摩擦 + 角度依存トルク」。
// //   各方向で全体の平均（≒摩擦）を引き、正転と逆転を平均したものを表にする。
// //   L1ボタンでFFのON/OFF。
// //
// // [学習追加] 運転速度での学習（推奨）:
// //   実機では低速同定の表が 0.2 rad/s での外乱より約5倍大きく、FF ON で揺れが増えた。
// //   R1ボタンで、目標速度で回したまま表を0から学習する。もう一度R1で学習を止めて表を固定。
// //
// // [追加] / [変更] / [FF追加] のコメントが変更点。

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
// // [修正] 角度の範囲は ±12.5 rad（多くのライブラリと同じ）。マニュアル記載の ±4π ではなかった。
// //   根拠: ±4π で読んだとき、2つの外乱の周期が 107.91 と 185.00 周期/出力1回転と測れたが、
// //   これはロータ1回転あたり 14（極対数）と 24 周期の外乱 = 108.5 と 186 を、ちょうど 4π/12.5 = 1.0053 倍
// //   長く測った値と一致する（2つとも同じ0.53%ずれ）。
// const float POS_RANGE = 12.5f;               // ±12.5 [rad]
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
// // [追加] フィードバックが途切れたときの扱い（実機で約80 ms途切れ、位置項が膨らんでモータが急加速した）
// const unsigned long FB_STALE_US = 10000;   // これ以上新しいフィードバックが来なければ位置項を固定
// const unsigned long FB_LOST_US  = 300000;  // これ以上来なければ安全停止
// const float REZERO_POS  = 9.0f;   // フィードバック角がこれを超えたらゼロ点を再設定 [rad]
// const float JUMP_DETECT = 1.0f;   // 1フレームでこれ以上動いたら「ゼロ点の付け替え」とみなす [rad]
// const unsigned long REZERO_TIMEOUT_US = 100000; // ゼロ点再設定が反映されるまでの待ち上限

// // --- [FF追加] 角度依存トルクの補償 ---
// // [修正4] 外乱は2つあり、どちらもロータ1回転あたりの整数周期:
// //   表1: 14周期/ロータ1回転（極対数 = 電気角1周期）→ 出力1回転あたり 14 × 7.75 = 108.5 周期
// //   表2: 24周期/ロータ1回転                         → 出力1回転あたり 24 × 7.75 = 186 周期
// //   （角度の範囲を ±12.5 に直したことで、実測の 107.91 / 185.00 がこの値に一致する）
// const double DIST_CYCLES_PER_REV  = 14.0 * 7.75;  // = 108.5
// const double DIST2_CYCLES_PER_REV = 24.0 * 7.75;  // = 186
// const int   FF_BINS     = 32;                 // 1周期を何分割するか
// const float W_ID        = 0.04f;              // 同定時の速度 [rad/s]（外乱 約0.7 Hz。15 sで約10周期）
// // 同定は 正転 → 逆転 の2段。各段で 3 s 待ってから 15 s 記録（約10周期）。
// // 正転と逆転の結果を平均すると、摩擦や加速度による「位相のずれ」が打ち消し合う。
// const unsigned long ID_SETTLE_US = 3000000UL;    // 各段の記録開始までの待ち 3 s
// const unsigned long ID_STAGE_US  = 18000000UL;   // 各段の長さ 18 s
// const float FF_LEAD_S   = 0.003f;             // 通信・制御の遅れを見越して先の角度で表を引く [s]

// // [学習追加] 運転速度のまま表を学習する（R1で開始／停止）
// //   PD（位置項 + Kd項）が毎回同じ位相で出しているトルク ＝ 表に足りない分。
// //   それを少しずつ表へ移す。表が外乱と一致すると、PDトルクの周期成分が0になり表は動かなくなる。
// //   低速の同定と違い、実際に使う速度で合わせるので振幅と位相がその速度に合う。
// const float LEARN_GAIN       = 0.02f;         // 1回の更新で表へ移す割合（最初の10 s, 制御周期ごと）
// const float LEARN_GAIN_FINE  = 0.005f;        // 10 s 以降の割合（速度の雑音を表に溜めにくくする）
// const unsigned long LEARN_FAST_US = 10000000UL;
// const float FF_TABLE_MAX = 0.10f;             // 表の値の上限 [Nm]。超えたら学習とFFを止める（発散の保護）
// bool     learning = false;                    // 学習中か
// unsigned long learn_start_us = 0;

// float    ff_table[FF_BINS] = {0};             // 表1: 108.5周期の位相ごとの補償トルク [Nm]
// float    ff_table2[FF_BINS] = {0};            // 表2: 186周期の位相ごとの補償トルク [Nm]（学習のみ）
// double   id_sum[2][FF_BINS];                  // [0]:正転, [1]:逆転
// uint32_t id_cnt[2][FF_BINS];
// int      id_dir = 0;                          // 同定中の段（0:正転, 1:逆転）
// bool     ff_ready   = false;                  // 同定済みか
// bool     ff_enabled = false;                  // FFを足すか（L1で切替）
// bool     id_active  = false;                  // 同定中か
// unsigned long id_start_us = 0;
// float    w_ref_saved = 0.0f;
// float    ff_cmd = 0.0f;                       // 今回足したFF [Nm]

// // back_motor の状態
// struct BackMotorState {
//     bool     fb_valid = false;     // フィードバックを1回以上受けたか
//     uint16_t raw_prev = 0;         // 前回の角度（生の16bit値）
//     float    pos = 0.0f;           // 現在角 [rad]（ゼロ点からの角度, ±12.5 で飽和）
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
//     bool     new_frame = false;    // [追加] 前回の制御周期以降に新しいフィードバックが来たか
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
// float pole_phase(double pos);                                             // [FF追加]
// float phase_of(double pos, double cycles_per_rev);                        // [FF追加]
// float table_lookup(const float* tbl, float phase);                        // [FF追加]
// bool  learn_table(float* tbl, float phase, float g);                      // [学習追加]
// void start_identification();                                              // [FF追加]
// void finish_identification();                                             // [FF追加]
// void abort_identification();                                              // [FF追加]
// void start_learning();                                                    // [学習追加]
// void stop_learning(const char* why);                                      // [学習追加]
// void learn_update(double pos, float u);                                   // [学習追加]
// bool direction_profile(int dir, float out[], float* mean_out, int* empty_out); // [FF追加]

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
// PS4ButtonEdge btnSquare, btnL1;   // [FF追加] □:同定開始, L1:FF ON/OFF
// PS4ButtonEdge btnR1;              // [学習追加] R1:学習の開始／停止

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
//                 offset_pos = uint_to_float(pos_raw, P_MIN, P_MAX, 16);   // 元コードと同じ ±12.5
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
//         bool right = btnRight.isPressedOnce(PS4.Right());
//         bool left  = btnLeft.isPressedOnce(PS4.Left());
//         if (!id_active) {                             // 同定中は目標速度を変えない
//             if (right) w_ref_set += W_REF_INC;
//             if (left)  w_ref_set -= W_REF_INC;
//         }
//         // [FF追加] □: 同定開始（運転中のみ）, L1: FF ON/OFF（同定済みのみ）
//         if (btnSquare.isPressedOnce(PS4.Square()) && power_on && !id_active) {
//             if (learning) stop_learning("ID");
//             start_identification();
//         }
//         if (btnL1.isPressedOnce(PS4.L1()) && ff_ready && !id_active) {
//             ff_enabled = !ff_enabled;
//             if (!ff_enabled && learning) stop_learning("FF off");   // 学習はFF ONのときだけ
//         }
//         // [学習追加] R1: 表を0から学習開始 → もう一度押すと学習を止めて表を固定
//         if (btnR1.isPressedOnce(PS4.R1()) && power_on && !id_active) {
//             if (!learning) start_learning();
//             else           stop_learning("R1");
//         }
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
//             } else {
//                 abort_identification();   // [FF追加] 同定中に止めたら同定は中止
//                 if (learning) stop_learning("power off");
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
//             M5.Display.printf("FF: %-4s\n", !ff_ready ? "--" : (learning ? "LRN" : (ff_enabled ? "ON" : "OFF")));
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
//         // [学習追加] 累積角の変化から求めた速度（50 ms 区間）。モータの速度フィードバックより雑音が小さい。
//         static double        pos_hist[5];
//         static unsigned long t_hist[5];
//         static int  hidx  = 0;
//         static bool hfull = false;
//         unsigned long now_us = micros();
//         float v_pos = 0.0f;
//         if (hfull) v_pos = (float)((back.pos_total - pos_hist[hidx]) / ((now_us - t_hist[hidx]) * 1e-6));
//         pos_hist[hidx] = back.pos_total;
//         t_hist[hidx]   = now_us;
//         hidx = (hidx + 1) % 5;
//         if (hidx == 0) hfull = true;

//         Serial.printf("%lu, %.3f, %.4f, %.5f, %.3f, %.4f, %.4f, %.3f, %d, %.4f\n",
//                       now_us / 1000UL,
//                       power_on ? w_ref_set : 0.0f,  // 目標速度
//                       back.vel,                     // 速度フィードバック
//                       e_pos,                        // 位置偏差
//                       t_ff_cmd,                     // 送ったトルクFF（位置項+FF）
//                       (float)back.pos_total,        // 累積角
//                       ff_cmd,                       // [FF追加] うちFF分
//                       pole_phase(back.pos_total),   // [FF追加] 外乱1周期内の位相 (0〜1)
//                       id_active ? 2 : (learning ? 3 : (ff_enabled ? 1 : 0)), // 0:FFなし 1:FFあり 2:同定中 3:学習中
//                       v_pos);                       // [学習追加] 累積角から求めた速度
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

//         // [FF追加] 同定中は「今の位相」に「今のモータトルク」を記録する
//         if (id_active && (now - id_start_us) > ID_SETTLE_US) {
//             int k = (int)lroundf(pole_phase(back.pos_total) * FF_BINS) % FF_BINS;
//             id_sum[id_dir][k] += back.trq;
//             id_cnt[id_dir][k]++;
//         }
//     }
//     back.raw_prev   = pos_raw;
//     back.last_fb_us = now;
//     back.fb_valid   = true;
//     back.new_frame  = true;
// }

// // ============================================================
// // [追加] back_motor の制御（CONTROL_PERIOD_US ごと）
// // ============================================================
// void back_control_step(float dt) {
//     // 1) 安全停止の処理
//     if (back.fault) request_trip("FAULT");
//     if (trip_request) {
//         abort_identification();     // [FF追加]
//         if (learning) stop_learning("trip");  // [学習追加]
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

//     // 2) 角度が飽和する（±12.5付近）前にゼロ点を付け替える
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

//     // [FF追加] 同定の段の切替と終了判定
//     if (id_active && (micros() - id_start_us) > ID_STAGE_US) {
//         if (id_dir == 0) {          // 正転が終わったら逆転へ
//             id_dir = 1;
//             w_ref_set = -W_ID;
//             id_start_us = micros();
//         } else {
//             finish_identification();
//         }
//     }

//     // [追加] フィードバックの鮮度。途切れている間は位置項を最後の値で固定し、学習も止める。
//     //   目標の進み（e の更新）は続けるので、フィードバックが戻ればその間の実際の移動分で正しく打ち消される。
//     static float e_hold = 0.0f;
//     unsigned long fb_age = micros() - back.last_fb_us;
//     bool fb_stale = back.fb_valid && fb_age > FB_STALE_US;
//     if (back.fb_valid && fb_age > FB_LOST_US) {
//         request_trip("FB");
//         return;
//     }
//     if (back.new_frame) {           // 実測で更新された直後の e を覚えておく
//         e_hold = e_pos;
//         back.new_frame = false;
//     }

//     // 4) トルクFF = 位置項 + 角度依存FF
//     ff_cmd = 0.0f;
//     if (ff_enabled && ff_ready && power_on && !id_active) {
//         // 指令がモータに効くまでの遅れ分だけ先の角度で表を引く
//         double pos_pred = back.pos_total + back.vel * (FF_LEAD_S + (fb_stale ? fb_age * 1e-6f : 0.0f));
//         ff_cmd = table_lookup(ff_table,  phase_of(pos_pred, DIST_CYCLES_PER_REV))
//                + table_lookup(ff_table2, phase_of(pos_pred, DIST2_CYCLES_PER_REV));
//     }
//     // [学習追加] PD が今出しているトルク（位置項 + ドライバ内のKd項）を、今の位相の表へ少し移す。
//     //   2つの表は周期が違うので、それぞれ自分の周期に同期した成分だけが溜まり、他方の成分は平均されて消える。
//     if (learning && ff_enabled && power_on && !id_active && !fb_stale) {
//         float u = kp_ext * e_pos + kd_mit * (w_ref - back.vel);
//         learn_update(back.pos_total, u);
//     }
//     float tau = kp_ext * (fb_stale ? e_hold : e_pos) + ff_cmd;
//     t_ff_cmd = clampf(tau, -TAU_LIMIT, TAU_LIMIT);

//     // 5) 運控モード指令：Kp_mit = 0, p_set は使われないので 0
//     send_motion_control(BACK_MOTOR_ID, t_ff_cmd, 0.0f, w_ref, 0.0f, kd_mit);
// }

// void request_trip(const char* reason) {
//     trip_request = true;
//     trip_reason = reason;
// }

// // ============================================================
// // [FF追加] 角度依存トルクの同定と補償
// // ============================================================

// // 外乱1周期内の位相 (0〜1)。累積角から計算するので、ゼロ点の付け替えに影響されない。
// // 原点は電源投入ごとに変わるため、表は毎回の運転で学習し直す。
// float phase_of(double pos, double cycles_per_rev) {
//     double c = pos * cycles_per_rev / (2.0 * M_PI);
//     return (float)(c - floor(c));
// }
// float pole_phase(double pos) { return phase_of(pos, DIST_CYCLES_PER_REV); }   // 表1の位相

// // 表を線形補間で引く（格子点 i は位相 i/FF_BINS）
// float table_lookup(const float* tbl, float phase) {
//     float x = phase * FF_BINS;
//     int i0 = (int)floorf(x);
//     float a = x - i0;
//     i0 = ((i0 % FF_BINS) + FF_BINS) % FF_BINS;
//     int i1 = (i0 + 1) % FF_BINS;
//     return tbl[i0] * (1.0f - a) + tbl[i1] * a;
// }

// void start_identification() {
//     for (int d = 0; d < 2; d++)
//         for (int i = 0; i < FF_BINS; i++) { id_sum[d][i] = 0.0; id_cnt[d][i] = 0; }
//     w_ref_saved = w_ref_set;
//     id_dir      = 0;
//     w_ref_set   = W_ID;           // まず正転
//     ff_enabled  = false;          // 同定中はFFを足さない
//     id_active   = true;
//     id_start_us = micros();
//     Serial.printf("# ID start: +/-%.3f rad/s, %lu s each\n", W_ID, ID_STAGE_US / 1000000UL);
// }

// // 1方向分の記録から「平均を引いた位相ごとのトルク」を作る。データ不足なら false。
// bool direction_profile(int dir, float out[], float* mean_out, int* empty_out) {
//     bool has[FF_BINS];
//     int n_empty = 0;
//     for (int i = 0; i < FF_BINS; i++) {
//         has[i] = id_cnt[dir][i] > 0;
//         out[i] = has[i] ? (float)(id_sum[dir][i] / id_cnt[dir][i]) : 0.0f;
//         if (!has[i]) n_empty++;
//     }
//     *empty_out = n_empty;
//     if (n_empty > FF_BINS / 4) return false;      // ほとんど回らなかった等
//     // 空の区間は両隣のデータがある区間から線形補間
//     for (int i = 0; i < FF_BINS; i++) {
//         if (has[i]) continue;
//         int l = i, r = i, dl = 0, dr = 0;
//         do { l = (l + FF_BINS - 1) % FF_BINS; dl++; } while (!has[l]);
//         do { r = (r + 1) % FF_BINS; dr++; } while (!has[r]);
//         out[i] = (out[l] * dr + out[r] * dl) / (float)(dl + dr);
//     }
//     // 平均（≒摩擦。正転と逆転で符号が逆）を引いて、角度依存の成分だけを残す
//     float mean = 0.0f;
//     for (int i = 0; i < FF_BINS; i++) mean += out[i];
//     mean /= FF_BINS;
//     for (int i = 0; i < FF_BINS; i++) out[i] -= mean;
//     *mean_out = mean;
//     return true;
// }

// void finish_identification() {
//     id_active = false;
//     w_ref_set = w_ref_saved;

//     float fwd[FF_BINS], rev[FF_BINS], mean_f, mean_r;
//     int empty_f, empty_r;
//     bool ok_f = direction_profile(0, fwd, &mean_f, &empty_f);
//     bool ok_r = direction_profile(1, rev, &mean_r, &empty_r);
//     if (!ok_f || !ok_r) {
//         Serial.printf("# ID failed: empty bins fwd=%d rev=%d (of %d)\n", empty_f, empty_r, FF_BINS);
//         return;
//     }
//     // 正転と逆転の平均 → 角度依存トルクの表
//     float amp = 0.0f;
//     for (int i = 0; i < FF_BINS; i++) {
//         ff_table[i] = 0.5f * (fwd[i] + rev[i]);
//         ff_table2[i] = 0.0f;          // 低速同定は表1だけ
//         amp = fmaxf(amp, fabsf(ff_table[i]));
//     }
//     ff_ready = true;

//     Serial.printf("# ID done: friction fwd=%.4f rev=%.4f Nm, table peak=%.4f Nm\n", mean_f, mean_r, amp);
//     Serial.printf("# ff, bin, phase, table, fwd, rev\n");
//     for (int i = 0; i < FF_BINS; i++) {
//         Serial.printf("# ff, %d, %.3f, %.5f, %.5f, %.5f\n", i, (float)i / FF_BINS, ff_table[i], fwd[i], rev[i]);
//     }
// }

// void abort_identification() {
//     if (!id_active) return;
//     id_active = false;
//     w_ref_set = w_ref_saved;
//     Serial.printf("# ID aborted\n");
// }

// // ============================================================
// // [学習追加] 運転速度での表の学習
// // ============================================================
// void start_learning() {
//     for (int i = 0; i < FF_BINS; i++) { ff_table[i] = 0.0f; ff_table2[i] = 0.0f; }   // 0から学習
//     ff_ready   = true;
//     ff_enabled = true;
//     learning   = true;
//     learn_start_us = micros();
//     Serial.printf("# learn start: w=%.3f rad/s, gain=%.3f -> %.3f after %lu s\n",
//                   w_ref_set, LEARN_GAIN, LEARN_GAIN_FINE, LEARN_FAST_US / 1000000UL);
// }

// void stop_learning(const char* why) {
//     learning = false;
//     float amp1 = 0.0f, amp2 = 0.0f;
//     for (int i = 0; i < FF_BINS; i++) {
//         amp1 = fmaxf(amp1, fabsf(ff_table[i]));
//         amp2 = fmaxf(amp2, fabsf(ff_table2[i]));
//     }
//     Serial.printf("# learn stop (%s): table1 peak=%.4f Nm, table2 peak=%.4f Nm\n", why, amp1, amp2);
//     Serial.printf("# lt, bin, phase, table1(108.5), table2(186)\n");
//     for (int i = 0; i < FF_BINS; i++) {
//         Serial.printf("# lt, %d, %.3f, %.5f, %.5f\n", i, (float)i / FF_BINS, ff_table[i], ff_table2[i]);
//     }
// }

// // 1つの表を更新する。g = 学習の割合 × PDトルク。表の範囲を超えたら true を返す。
// bool learn_table(float* tbl, float phase, float g) {
//     // 表の引き方（線形補間）と同じ重みで隣り合う2区間へ配る
//     float x = phase * FF_BINS;
//     int i0 = (int)floorf(x);
//     float a = x - i0;
//     i0 = ((i0 % FF_BINS) + FF_BINS) % FF_BINS;
//     int i1 = (i0 + 1) % FF_BINS;
//     tbl[i0] += g * (1.0f - a);
//     tbl[i1] += g * a;

//     // 表の平均を0に保つ（一定の摩擦分は位置項に任せ、表には角度で変わる分だけを残す）
//     float mean = 0.0f;
//     for (int i = 0; i < FF_BINS; i++) mean += tbl[i];
//     mean /= FF_BINS;
//     bool over = false;
//     for (int i = 0; i < FF_BINS; i++) {
//         tbl[i] -= mean;
//         if (fabsf(tbl[i]) > FF_TABLE_MAX) over = true;
//     }
//     return over;
// }

// // u: PD が出しているトルク [Nm]。2つの表をそれぞれの周期の位相で更新する。
// void learn_update(double pos, float u) {
//     float gain = (micros() - learn_start_us < LEARN_FAST_US) ? LEARN_GAIN : LEARN_GAIN_FINE;
//     float g = gain * u;
//     bool over1 = learn_table(ff_table,  phase_of(pos, DIST_CYCLES_PER_REV),  g);
//     bool over2 = learn_table(ff_table2, phase_of(pos, DIST2_CYCLES_PER_REV), g);
//     if (over1 || over2) {           // 発散の保護：学習とFFを止める（モータは止めない）
//         stop_learning("table limit");
//         ff_enabled  = false;
//         trip_reason = "FFLIM";
//     }
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
// //   Data: Byte0〜1 目標角度(マニュアルは−4π〜4π、実機は−12.5〜12.5), Byte2〜3 目標速度(−30〜30 rad/s),
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
