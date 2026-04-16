#include <M5Unified.h>
#include <mcp_can.h>
#include <SPI.h>

// --- ピン・ハードウェア設定 ---
#define CAN0_INT 15
const int SPI_CS_PIN = 27; 
MCP_CAN CAN0(SPI_CS_PIN);
uint32_t rxId;
unsigned char len = 0;
unsigned char buf[8];

// --- モーター基本設定 ---
#define MOTOR1_ID  0x7F
#define MOTOR2_ID  0x7F
#define MASTER_ID 0x00

// --- CyberGear 通信モード (拡張ID上位5bit) ---
#define MODE_MOTOR_ENABLE     0x03   
#define MODE_MOTOR_STOP       0x04
#define MODE_SET_ZERO_POS     0x06   
#define MODE_PARAM_WRITE      0x12   
#define MODE_PARAM_READ       0x11

// --- CyberGear 内部レジスタインデックス ---
#define INDEX_RUN_MODE        0x7005 // 1:位置, 2:速度, 3:電流
#define INDEX_TARGET_POS      0x7016 // 目標位置 (float, rad)
#define INDEX_TARGET_SPD      0x700A // 目標速度 (float, rad/s)
#define INDEX_TARGET_CUR      0x7006 // 目標電流 (float, A)
#define INDEX_LIMIT_SPD       0x7017 // 速度制限 (float)
#define INDEX_KP_ANG          0x701E
#define INDEX_KP_VEL          0x701F
#define INDEX_KI_VEL          0x7020

#define INDEX_ENCODER_RAW  0x3004
#define INDEX_MECH_POS     0x3016
#define INDEX_MECH_VEL     0x3017
#define INDEX_IQ           0x3020

// --- モード定義 ---
#define CONTROL_MODE_POS      1
#define CONTROL_MODE_SPD      2
#define CONTROL_MODE_CUR      3

// --- 制御・状態管理 ---
float target_position = 1.57f; // 90度
float target_velocity = 0.0f;  // テスト用: 5 rad/s
float target_current  = 0.0f;  // 0.3 A
int current_mode = CONTROL_MODE_CUR; 
bool is_running = false; // タッチ状態管理用
float pre_error = 0.0;
float pre_prop = 0.0;
float integral = 0.0;
float output = 0.0;
float u_min = -30.0;
float u_max = 30.0;

// ゲイン管理 (初期値を設定しておく)
float kp_vel = 0.00f;
float ki_vel = 0.00f;
float kd_vel = 0.00f;

// 読み取り値格納用
float spd = 0.0;
float trq = 0.0;
unsigned long pre_time_10 = 0;
unsigned long pre_time_100 = 0;

// 関数プロトタイプ
void init_can();
void stop_motor(uint8_t motor_id);
void enable_motor(uint8_t motor_id);
void set_zero_position(uint8_t motor_id);
void send_parameter_read(uint8_t motor_id, uint16_t param_index);
void send_parameter_write(uint8_t motor_id, uint16_t param_index, float value, uint8_t is_byte = 0);
void control_velocity(uint8_t motor_id, float rad_s);
void control_current(uint8_t motor_id, float current);
void change_mode(uint8_t motor_id, uint8_t mode);
void velocity_type_pid_control(float target, float actual, float dt);


// uint16をfloatに変換する関数
float uint_to_float(uint16_t x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (float)x * span / ((1 << bits) - 1) + offset;
}

void setup() {
    M5.begin();
    Serial.begin(115200);
    Serial.println("\n--- CyberGear Unified Control ---");
    Serial.println("Touch screen to Spin, Release to Stop.");
    Serial.println("Keys: 'q'/ 'a' -> kp_vel UP/DOWN, 'w'/ 's' -> ki_vel UP/DOWN");

    init_can();
    delay(500);

    // マニュアルの指示通り「停止 -> モード変更 -> 有効化」の順で行う
    stop_motor(MOTOR2_ID);
    delay(100);
    
    change_mode(MOTOR2_ID, current_mode);
    delay(100);

    set_zero_position(MOTOR2_ID);
    delay(100);

    enable_motor(MOTOR2_ID);
    delay(100);
    
    Serial.println("Setup Completed.");
}

void loop() {
    M5.update();    

    // --- 1. タッチ操作による回転制御 (状態管理) ---
    bool is_touched = (M5.Touch.getCount() > 0);

    if (is_touched && !is_running) {
        is_running = true;
        target_velocity = 5.0;
        Serial.println("Status: RUNNING");
    } 
    else if (!is_touched && is_running) {
        is_running = false;
        target_velocity = 0.0;
        Serial.println("Status: STOPPED");
    }

    // --- 2. シリアル入力によるゲイン変更 ---
    if (Serial.available()) {
        char c = Serial.read();
        bool gain_changed = false;

        switch (c) {
            case 'q': kp_vel += 0.001f; break;
            case 'a': kp_vel -= 0.001f; break;
            case 'w': ki_vel += 0.00001f; break;
            case 's': ki_vel -= 0.00001f; break;
            case 'e': kd_vel += 0.001f; break;
            case 'd': kd_vel -= 0.001f; break;
        }

        // 下限ガード
        if (kp_vel < 0.0f) kp_vel = 0.0f;
        if (ki_vel < 0.0f) ki_vel = 0.0f;
        if (kd_vel < 0.0f) kd_vel = 0.0f;
    }

    // --- 3. CAN受信処理 (※ここを正しく修正しました！) ---
    while (CAN0.checkReceive() == CAN_MSGAVAIL) {
        unsigned long rxId; // ← 型を mcp_can の要求に厳密に合わせる
        
        // エラーが出ていた関数を、正しい3つの引数に戻す
        CAN0.readMsgBuf(&rxId, &len, buf); 

        uint32_t cleanId = rxId & 0x1FFFFFFF;
        uint8_t mode = (cleanId >> 24) & 0x1F;

        if (mode == 0x02) { // フィードバックフレーム
            uint16_t spd_raw = (buf[2] << 8) | buf[3];
            uint16_t trq_raw = (buf[4] << 8) | buf[5];

            spd = uint_to_float(spd_raw, -30.0f, 30.0f, 16); 
            trq = uint_to_float(trq_raw, -12.0f, 12.0f, 16);
        }
    }

    if(millis() - pre_time_10 > 10) {
        float dt = (millis() - pre_time_10) / 1000;
        velocity_type_pid_control(target_velocity, spd, dt);
        pre_time_10 = millis();
        control_current(MOTOR2_ID, -target_current);

        // Serial.print(">vel:");
        // Serial.println(spd);
        // Serial.print(">trq:");
        // Serial.println(trq);
        // Serial.print(">kp:");
        // Serial.println(kp_vel);
        // Serial.print(">ki:");
        // Serial.println(ki_vel);
        // Serial.print(">kd:");
        // Serial.println(kd_vel);
        Serial.printf("%f,%f,%f,%f,%f,%f,%f\n", kp_vel,ki_vel,kd_vel,target_velocity,target_current,spd,trq);
    }

    // --- 4. 100ms周期でのデータ要求とシリアル出力 ---
    // if (millis() - pre_time_100 > 100) {
    // }
}


// --- CyberGear 通信基盤関数群 ---

void init_can() {
    pinMode(CAN0_INT, INPUT);
    if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
        CAN0.setMode(MCP_NORMAL);
        Serial.println("CAN Init OK");
    } else {
        Serial.println("CAN Init Failed!");
        while(1) delay(10);
    }
}

void stop_motor(uint8_t motor_id) {
    uint32_t id = ((uint32_t)MODE_MOTOR_STOP << 24) | ((uint32_t)MASTER_ID << 8) | motor_id;
    uint8_t dummy[8] = {0};
    CAN0.sendMsgBuf(id, 1, 8, dummy);
}

void enable_motor(uint8_t motor_id) {
    uint32_t id = ((uint32_t)MODE_MOTOR_ENABLE << 24) | ((uint32_t)MASTER_ID << 8) | motor_id;
    uint8_t dummy[8] = {0};
    CAN0.sendMsgBuf(id, 1, 8, dummy);
}

void set_zero_position(uint8_t motor_id) {
    uint32_t id = ((uint32_t)MODE_SET_ZERO_POS << 24) | ((uint32_t)MASTER_ID << 8) | motor_id;
    uint8_t dummy[8] = {0};
    dummy[0] = 1;
    CAN0.sendMsgBuf(id, 1, 8, dummy);
}

void change_mode(uint8_t motor_id, uint8_t mode) {
    send_parameter_write(motor_id, INDEX_RUN_MODE, (float)mode, 1);
}

void control_current(uint8_t motor_id, float current) {
    send_parameter_write(motor_id, INDEX_TARGET_CUR, current, 0);
}

void control_velocity(uint8_t motor_id, float rad_s) {
    send_parameter_write(motor_id, INDEX_TARGET_SPD, rad_s, 0);
}

void send_parameter_write(uint8_t motor_id, uint16_t param_index, float value, uint8_t is_byte) {
    uint32_t id = ((uint32_t)MODE_PARAM_WRITE << 24) | ((uint32_t)MASTER_ID << 8) | motor_id;
    uint8_t data[8] = {0};
    
    data[0] = param_index & 0xFF;
    data[1] = (param_index >> 8) & 0xFF;

    if (is_byte) {
        data[4] = (uint8_t)value;
    } else {
        memcpy(&data[4], &value, 4);
    }
    CAN0.sendMsgBuf(id, 1, 8, data);
}

void send_parameter_read(uint8_t motor_id, uint16_t param_index) {
    uint32_t id = ((uint32_t)MODE_PARAM_READ << 24) | ((uint32_t)MASTER_ID << 8) | motor_id;
    uint8_t data[8] = {0};

    data[0] = param_index & 0xFF;
    data[1] = (param_index >> 8) & 0xFF;

    CAN0.sendMsgBuf(id, 1, 8, data);
}

void velocity_type_pid_control(float target, float actual, float dt) {
    float error = target - actual;
    float prop = error - pre_error;
    float deriv = prop - pre_prop;
    integral += error;
    float du = kp_vel * error + ki_vel * integral  + kd_vel * deriv;
    pre_error = error;
    pre_prop = prop;
    output += du;
    target_current = constrain(output, -23.0f, 23.0f);
}