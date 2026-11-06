// step1_mit_velocity.cpp
// 学習ステップ1：MITモード（運控モード）による疑似速度制御
//
// ・動かすのは back_motor だけ（front_motor は使わない）
// ・制御則（2 ms ごと）
//     e    += 目標速度 × dt − 実際に進んだ角度    … 位置偏差 e = 目標位置 − 実際の位置
//     t_ff  = KP × e                              … このコードで計算して送る
//     τ     = KD × (目標速度 − 速度) + t_ff        … ドライバの中で計算される
// ・PS4：○ 運転の開始／停止、→ ← 目標速度 ±0.1 rad/s
//
// 読む順番：「5. 受信」→「6. 制御」が中心。1〜4 はその準備、7 は起動とメインループ。

#include <M5Unified.h>
#include <mcp_can.h>
#include <SPI.h>
#include <PS4Controller.h>

// ============================================================
// 1. 設定
// ============================================================
MCP_CAN CAN0(27);                      // CANモジュールの CS ピン
const uint8_t MOTOR_ID  = 0x7F;        // back_motor
const uint8_t MASTER_ID = 0x00;

const float KP = 10.0f;                // 位置ゲイン [Nm/rad]   … 速度の I ゲインに相当
const float KD = 0.2f;                 // 速度ゲイン [Nm·s/rad] … 速度の P ゲインに相当
const unsigned long PERIOD_US = 2000;  // 制御周期 2 ms

const float TAU_MAX    = 3.0f;         // t_ff の上限 [Nm]
const float E_MAX      = 0.5f;         // 位置偏差がこれを超えたら停止 [rad]
const float REZERO_POS = 9.0f;         // 角度がこれを超えたらゼロ点を付け替える [rad]
const float POS_LIMIT  = 12.0f;        // ここまで来たらゼロ点の付け替えが効いていない → 停止 [rad]
const unsigned long FB_TIMEOUT_US = 20000;  // フィードバックが 20 ms 来なければ停止

// 運控モードで送受信する値の範囲（どれも 16bit の 0〜65535 に割り当てられる）
const float P_MAX  = 12.5f;            // 角度   ±12.5 rad（実機で確認済み）
const float V_MAX  = 30.0f;            // 速度   ±30 rad/s
const float T_MAX  = 12.0f;            // トルク ±12 Nm
const float KP_MAX = 500.0f;           // Kp     0〜500
const float KD_MAX = 5.0f;             // Kd     0〜5
const float POS_LSB = 2 * P_MAX / 65535.0f;  // 角度1カウントの大きさ = 0.38 mrad

// ============================================================
// 2. 状態
// ============================================================
bool   running = false;         // 運転中か
float  w_ref   = 0.2f;          // 目標速度 [rad/s]
float  e       = 0.0f;          // 位置偏差 [rad]（目標位置 − 実際の位置）
float  t_ff    = 0.0f;          // 送ったトルク指令 [Nm]

float    pos = 0.0f;            // モータが返した角度 [rad]
float    vel = 0.0f;            // モータが返した速度 [rad/s]
uint16_t raw_prev  = 0;         // 前回の角度（16bitの生の値）
bool     have_prev = false;     // raw_prev が有効か
float    moved     = 0.0f;      // 前回の制御以降に実際に進んだ角度 [rad]
double   pos_total = 0.0;       // 累積角 [rad]（ログ用）
bool     rezero_pending = false;  // ゼロ点の付け替えを送って、まだ反映を確認していない
unsigned long last_fb_us = 0;   // 最後にフィードバックを受け取った時刻
const char* stop_reason = "-";

// ============================================================
// 3. 値の変換（実数 ⇔ 16bit）
// ============================================================
float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

// 0〜65535 → x_min〜x_max
float uint_to_float(uint16_t x, float x_min, float x_max) {
    return x * (x_max - x_min) / 65535.0f + x_min;
}
// x_min〜x_max → 0〜65535（範囲外は端に丸める）
uint16_t float_to_uint(float x, float x_min, float x_max) {
    x = clampf(x, x_min, x_max);
    return (uint16_t)lroundf((x - x_min) * 65535.0f / (x_max - x_min));
}

// ============================================================
// 4. CAN送信
//   拡張ID(29bit) = 通信タイプ(5bit) | 中央16bit | モータID(8bit)
// ============================================================
void can_send(uint8_t type, uint16_t mid, uint8_t* data) {
    uint32_t id = ((uint32_t)type << 24) | ((uint32_t)mid << 8) | MOTOR_ID;
    CAN0.sendMsgBuf(id, 1, 8, data);
}

void motor_enable() {                      // タイプ3：有効化
    uint8_t d[8] = {0};
    can_send(3, MASTER_ID, d);
}
void motor_stop(uint8_t clear_fault) {     // タイプ4：停止（1 なら故障もクリア）
    uint8_t d[8] = {0};
    d[0] = clear_fault;
    can_send(4, MASTER_ID, d);
}
void set_zero() {                          // タイプ6：今の角度を 0 にする（電源を切ると消える）
    uint8_t d[8] = {0};
    d[0] = 1;
    can_send(6, MASTER_ID, d);
}
void set_mit_mode() {                      // タイプ18：パラメータ run_mode(0x7005) に 0（運控モード）を書く
    uint8_t d[8] = {0x05, 0x70, 0, 0, 0, 0, 0, 0};   // インデックスは下位バイトが先
    can_send(18, MASTER_ID, d);
}

// タイプ1：運控モード指令
//   トルクは拡張IDの中央16bit、角度・速度・Kp・Kd はデータに 16bit ずつ（上位バイトが先）
void send_mit(float torque, float p_set, float v_set, float kp, float kd) {
    uint16_t v[4] = {
        float_to_uint(p_set, -P_MAX, P_MAX),
        float_to_uint(v_set, -V_MAX, V_MAX),
        float_to_uint(kp,    0.0f,   KP_MAX),
        float_to_uint(kd,    0.0f,   KD_MAX) };
    uint8_t d[8];
    for (int i = 0; i < 4; i++) {
        d[2 * i]     = v[i] >> 8;
        d[2 * i + 1] = v[i] & 0xFF;
    }
    can_send(1, float_to_uint(torque, -T_MAX, T_MAX), d);
}

// 運転の開始と停止
void start_run() {
    motor_stop(1);            // 前回の故障をクリア
    motor_enable();
    e = 0.0f;                 // 今いる位置から追いかけ始める
    moved = 0.0f;
    pos = 0.0f;
    have_prev = false;        // 止まっている間に回された分を「角度の飛び」と誤判定しない
    rezero_pending = false;
    last_fb_us = micros();
    stop_reason = "-";
    running = true;
}
void stop_run(const char* reason) {
    running = false;
    t_ff = 0.0f;
    stop_reason = reason;
    motor_stop(0);
}

// ============================================================
// 5. 受信（フィードバック＝タイプ2）  … ① 実際に進んだ角度を求める
//   拡張ID：bit28〜24 = 2、bit15〜8 = 送ってきたモータのID
//   データ：角度・速度・トルク・温度（各16bit、上位バイトが先）
// ============================================================
void receive() {
    long unsigned int id;
    unsigned char len;
    unsigned char b[8];
    while (CAN0.checkReceive() == CAN_MSGAVAIL) {
        CAN0.readMsgBuf(&id, &len, b);
        uint8_t type = (id >> 24) & 0x1F;
        uint8_t from = (id >> 8) & 0xFF;
        if (type != 2 || from != MOTOR_ID) continue;

        uint16_t raw = (b[0] << 8) | b[1];
        pos = uint_to_float(raw, -P_MAX, P_MAX);
        vel = uint_to_float((b[2] << 8) | b[3], -V_MAX, V_MAX);

        if (have_prev) {
            // 前回からの移動量。整数のまま差を取るので、丸め誤差が積み重ならない
            float d = ((int32_t)raw - (int32_t)raw_prev) * POS_LSB;
            if (fabsf(d) > 1.0f) {               // 1フレームで 1 rad 以上 = 角度が飛んだ
                if (rezero_pending) {            //   ゼロ点を付け替えた → その間の移動は 速度×時間 で埋める
                    d = vel * PERIOD_US * 1e-6f;
                    rezero_pending = false;
                } else {                         //   想定外 → 安全のため停止
                    d = 0.0f;
                    stop_run("JUMP");
                }
            }
            moved     += d;
            pos_total += d;
        }
        raw_prev   = raw;
        have_prev  = true;
        last_fb_us = micros();
    }
}

// ============================================================
// 6. 制御（2 ms ごと）  … ② 位置偏差の更新 → ③ トルク指令を送る
// ============================================================
void control_step(float dt) {
    if (!running) {
        moved = 0.0f;
        return;
    }

    // 安全確認：フィードバックが来ない、または角度が範囲の端に近い
    //   （どちらも「実際に進んだ角度」が分からなくなり、e が増え続けて急加速するため）
    if (micros() - last_fb_us > FB_TIMEOUT_US) { stop_run("FB");    return; }
    if (fabsf(pos) > POS_LIMIT)                { stop_run("RANGE"); return; }

    // ② 位置偏差 e（目標位置 − 実際の位置）を「この周期の差」だけで更新する
    e += w_ref * dt - moved;   // 目標が進んだ量 − 実際に進んだ量
    moved = 0.0f;
    if (fabsf(e) > E_MAX) {
        stop_run("E_MAX");
        return;
    }

    // ③ トルク指令。ドライバ内の位置項は使わない（Kp = 0、p_set = 0）
    t_ff = clampf(KP * e, -TAU_MAX, TAU_MAX);
    send_mit(t_ff, 0.0f, w_ref, 0.0f, KD);
    //   ④ ドライバの中で τ = KD × (w_ref − 速度) + t_ff が計算される

    // モータが返す角度は ±12.5 rad で止まるので、その手前で今の角度を 0 にする
    if (!rezero_pending && fabsf(pos) > REZERO_POS) {
        set_zero();
        rezero_pending = true;
    }
}

// ============================================================
// 7. 起動とメインループ
// ============================================================
struct Edge {                 // ボタンが「押された瞬間」だけ true
    bool last = false;
    bool pressed(bool now) { bool p = now && !last; last = now; return p; }
};
Edge btnCircle, btnRight, btnLeft;

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    Serial.begin(115200);
    PS4.begin("08:F9:E0:F5:E7:D6");
    if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
        while (1) delay(10);
    }
    CAN0.setMode(MCP_NORMAL);
    M5.Display.setTextSize(3);

    motor_stop(0);            // モードの変更は停止中に行う
    delay(10);
    set_mit_mode();           // 運控モードにする
    delay(50);
}

void loop() {
    M5.update();
    receive();                // ① フィードバックを読む

    // PS4 の操作
    if (PS4.isConnected()) {
        if (btnCircle.pressed(PS4.Circle())) {
            if (running) stop_run("STOP");
            else         start_run();
        }
        if (btnRight.pressed(PS4.Right())) w_ref += 0.1f;
        if (btnLeft.pressed(PS4.Left()))   w_ref -= 0.1f;
        w_ref = clampf(w_ref, -5.0f, 5.0f);
        if (fabsf(w_ref) < 1e-4f) w_ref = 0.0f;
    } else if (running) {
        stop_run("PS4");      // コントローラが切れたら止める
    }

    unsigned long now = micros();

    // ②③ 制御（2 ms ごと）
    static unsigned long last_ctrl = 0;
    if (now - last_ctrl >= PERIOD_US) {
        float dt = (last_ctrl == 0) ? PERIOD_US * 1e-6f : (now - last_ctrl) * 1e-6f;
        last_ctrl = now;
        control_step(dt);
    }

    // ログ（10 ms ごと）：時刻[ms], 目標速度, 速度(累積角から), 速度(モータの値), 位置偏差, トルク指令
    static unsigned long last_log = 0;
    if (now - last_log >= 10000) {
        last_log = now;
        // 累積角の 50 ms 間の変化から速度を求める（モータが返す速度より雑音が小さい）
        static double hist_pos[5];
        static unsigned long hist_t[5];
        static int  k = 0;
        static bool full = false;
        float v_pos = full ? (float)((pos_total - hist_pos[k]) / ((now - hist_t[k]) * 1e-6)) : 0.0f;
        hist_pos[k] = pos_total;
        hist_t[k]   = now;
        k = (k + 1) % 5;
        if (k == 0) full = true;

        Serial.printf("%lu, %.3f, %.4f, %.4f, %.5f, %.4f\n",
                      now / 1000UL, running ? w_ref : 0.0f, v_pos, vel, e, t_ff);
    }

    // 画面（止まっているときだけ更新。運転中の描画は制御周期を乱すため）
    static unsigned long last_disp = 0;
    if (!running && now - last_disp >= 100000) {
        last_disp = now;
        M5.Display.setCursor(0, 10);
        M5.Display.printf("Run : %d\n", running);
        M5.Display.printf("w   : %6.2f\n", w_ref);
        M5.Display.printf("Stop: %-6s\n", stop_reason);
    }
}