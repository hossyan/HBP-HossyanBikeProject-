#include <M5Unified.h>
#include <mcp_can.h>
#include <SPI.h>
#include <MadgwickAHRS.h>
#include <PS4Controller.h>
#include "policy.h"

// --- ピン・ハードウェア設定 ---
#define CAN0_INT 15
const int SPI_CS_PIN = 27; 
MCP_CAN CAN0(SPI_CS_PIN);
long unsigned int rxId;
unsigned char len = 0;
unsigned char buf[8];

// --- モーター基本設定 ---
#define FRONT_MOTOR_ID  0x7E
#define BACK_MOTOR_ID  0x7F
#define MASTER_ID 0x00

// --- CyberGear 通信モード (拡張ID上位5bit) ---
#define MODE_MOTOR_ENABLE     0x03   
#define MODE_SET_ZERO_POS     0x06   
#define MODE_PARAM_WRITE      0x12   

// --- CyberGear 内部レジスタインデックス ---
#define INDEX_RUN_MODE        0x7005 // 1:位置, 2:速度, 3:電流
#define INDEX_TARGET_POS      0x7016 // 目標位置 (float, rad)
#define INDEX_TARGET_SPD      0x7017 // 目標速度 (float, rad/s)
#define INDEX_TARGET_CUR      0x7006 // 目標電流 (float, A)

// --- モード定義 ---
#define CONTROL_MODE_POS      1
#define CONTROL_MODE_SPD      2
#define CONTROL_MODE_CUR      3

// --- 制御目標値 ---
float front_motor_target = 45 * M_PI / 180.0f; //45degree in radian
float offset_pos = 0.0f;
float back_motor_target = 0.0f;  //A

float pre_error = 0.0f;
float pre_prop = 0.0f;
float integral = 0.0f;
float output = 0.0f;
float u_min = -23.0f;
float u_max = 23.0f;

float kp_vel = 1.05f;
float ki_vel = 2.54f;
float kd_vel = 0.0f;

bool start_flag = false;
bool circle_prev = false;
float stick_left_y = 0.0; 
float stick_right_x = 0.0; 
float target_velocity = 2.0f;

float obs[3] = {0.0, 0.0, 0.0};
float action = 0.0;
float action_scale = 8.0;
unsigned long pre_pid_time = 0.0;

// 角度推定　センサーフィルタリング用
Madgwick filter;
unsigned long microsPerReading, microsPre;
unsigned long microsPre_rl = 0.0;
unsigned long microsPre_pid = 0.0;
float accelScale, gyroScale;

float ax,ay,az;
float gx,gy,gz;
unsigned long microsNow;

float filtered_gx = 0.0;
float filtered_gy = 0.0;
float filtered_gz = 0.0;

float filtered_back_motor_spd = 0.0f;

// --- マルチコア用 ---
volatile float policy_obs[3] = {0.0, 0.0, 0.0};  // タスク間共有obs
volatile float policy_action = 0.0;               // タスク間共有action
portMUX_TYPE policy_mux = portMUX_INITIALIZER_UNLOCKED; // ミューテックス

TaskHandle_t policyTaskHandle = NULL;
#define POLICY_INTERVAL_MS 15

// 関数プロトタイプ
void init_can();
void enable_motor(uint8_t motor_id);
void set_zero_position(uint8_t motor_id);
void send_parameter_write(uint8_t motor_id, uint16_t param_index, float value, uint8_t is_byte = 0);
void velocity_type_pid_control(float target, float actual, float dt);

// cybergear制御関数
void control_position(uint8_t motor_id, float rad);
void control_current(uint8_t motor_id, float ampere);
void change_mode(uint8_t motor_id, uint8_t mode);

// --- 追加：uint16をfloatに変換する関数 ---
float uint_to_float(uint16_t x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (float)x * span / ((1 << bits) - 1) + offset;
}

void policyTask(void *pvParameters) {
    float local_obs[3];
    float local_action = 0.0f;
    TickType_t xLastWakeTime = xTaskGetTickCount(); // 起動時刻を記録
    
    for (;;) {
        portENTER_CRITICAL(&policy_mux);
        local_obs[0] = policy_obs[0];
        local_obs[1] = policy_obs[1];
        local_obs[2] = policy_obs[2];
        policy_action = local_action; 
        portEXIT_CRITICAL(&policy_mux);

        local_action = policy_infer(local_obs) * action_scale;

        vTaskDelay(1); 
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(POLICY_INTERVAL_MS));
    }
}

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Imu.begin();
    Serial.begin(115200);
    filter.begin(25);

    PS4.begin("08:F9:E0:F5:E7:D6");
    
    microsPerReading = 1000000 / 25;
    microsPre = micros();

    init_can();
    delay(1000);

    enable_motor(FRONT_MOTOR_ID);
    enable_motor(BACK_MOTOR_ID);
    delay(100);

    unsigned long timeout = millis();
    while (true) {
        if (millis() - timeout > 3000) break; 
        if (CAN0.checkReceive() == CAN_MSGAVAIL) {        
            CAN0.readMsgBuf(&rxId, &len, buf); 

            uint32_t cleanId = rxId & 0x1FFFFFFF;
            uint8_t source_motor_id = (cleanId >> 8) & 0xFF;
            uint8_t mode = (cleanId >> 24) & 0x1F;

            if (mode == 0x02 && source_motor_id == FRONT_MOTOR_ID) {
                uint16_t pos_raw = (buf[0] << 8) | buf[1];
                offset_pos = uint_to_float(pos_raw, -12.5f, 12.5f, 16);
                break;
            }
        }
    delay(1);
}

    // 初期モード設定
    change_mode(FRONT_MOTOR_ID, CONTROL_MODE_POS);
    change_mode(BACK_MOTOR_ID, CONTROL_MODE_CUR);

    // Core 0 でpolicyタスクを起動
    xTaskCreatePinnedToCore(
        policyTask,        // タスク関数
        "PolicyTask",      // タスク名
        8192,              // スタックサイズ（policy_inferの重さに応じて調整）
        NULL,              // 引数
        1,                 // 優先度
        &policyTaskHandle, // ハンドル
        0                  // Core 0 を指定
    );
}

void loop() {
    M5.update();

    // --- センサデータの取得とフィルタリング ---
    M5.Imu.getAccel(&ax, &ay, &az);
    M5.Imu.getGyro(&gx, &gy, &gz);
    float gx_rad = gx * (M_PI / 180.0f);
    float gy_rad = gy * (M_PI / 180.0f);
    float gz_rad = gz * (M_PI / 180.0f);

    float alpha = 0.8; 
    filtered_gx = gx_rad * alpha + filtered_gx * (1 - alpha); 
    filtered_gy = gy_rad * alpha + filtered_gy * (1 - alpha); 
    filtered_gz = gz_rad * alpha + filtered_gz * (1 - alpha); 

    // Madgwickfilter
    microsNow = micros();
    float dt = (float)(microsNow - microsPre) / 1000000.0f;
    if (dt > 0) {
        filter.begin(1.0f / dt); 
    }    
    filter.updateIMU(gx, gy, gz, ax, ay, az);
    float roll_deg = filter.getRoll();
    float roll_rad = roll_deg * (M_PI / 180.0f);
    microsPre = microsNow;

    // cybergearからのフィードバック受信
    float front_motor_pos, front_motor_spd;
    float back_motor_pos, back_motor_spd;
    while (CAN0.checkReceive() == CAN_MSGAVAIL) {        
        CAN0.readMsgBuf(&rxId, &len, buf); 

        uint32_t cleanId = rxId & 0x1FFFFFFF;
        uint8_t source_motor_id = (cleanId >> 8) & 0xFF;
        uint8_t mode = (cleanId >> 24) & 0x1F;

        if (mode == 0x02) { // フィードバックフレーム
            uint16_t pos_raw = (buf[0] << 8) | buf[1];
            uint16_t spd_raw = (buf[2] << 8) | buf[3];
            uint16_t trq_raw = (buf[4] << 8) | buf[5];

            if (source_motor_id == FRONT_MOTOR_ID) {
                front_motor_pos = uint_to_float(pos_raw, -12.5f, 12.5f, 16);
                front_motor_spd = uint_to_float(spd_raw, -30.0f, 30.0f, 16);
            } else if (source_motor_id == BACK_MOTOR_ID) {
                back_motor_pos = uint_to_float(pos_raw, -12.5f, 12.5f, 16);
                back_motor_spd = uint_to_float(spd_raw, -30.0f, 30.0f, 16);
            }
        }
    }

    // ps4コントローラーからの入力処理
    if (PS4.isConnected()) {
        bool circle_now = PS4.Circle();
        if (circle_now && !circle_prev) { 
            if(!start_flag) {
                target_velocity = -target_velocity; // 目標速度の符号を反転
            }
            start_flag = !start_flag;
        }
        circle_prev = circle_now;

        if (PS4.LStickY()) {
            int stick_value = PS4.LStickY();
            stick_left_y = (float)stick_value / 130;
            if(-0.1 < stick_left_y && stick_left_y < 0.1){
                stick_left_y = 0;
            }
        }
        if (PS4.RStickX()) {
            int stick_value = PS4.RStickX();
            stick_right_x = (float)stick_value / 130;
            if(-0.1 < stick_right_x && stick_right_x < 0.1){
                stick_right_x = 0;
            }
        }
        // PSボタンで切断
        if (PS4.PSButton()) {
            start_flag = false;
            ESP.restart();  // M5Stack自体を再起動、BT接続も切れる
        }
    }

    float alpha_spd = 0.8f;
    filtered_back_motor_spd = back_motor_spd * alpha_spd + filtered_back_motor_spd * (1 - alpha_spd);

    // --- observation ---
    // policy_obsを更新（クリティカルセクション）
    portENTER_CRITICAL(&policy_mux);
    policy_obs[0] = -roll_rad;
    policy_obs[1] = -filtered_gx;
    policy_obs[2] = -filtered_back_motor_spd / 2;
    portEXIT_CRITICAL(&policy_mux);

    // --- policy結果を読み出す ---
    portENTER_CRITICAL(&policy_mux);
    action = policy_action;
    portEXIT_CRITICAL(&policy_mux);

    Serial.printf(">roll_rad:%f\n", -roll_rad);
    Serial.printf(">roll_rad/s:%f\n", -filtered_gx);
    Serial.printf(">spd:%f\n", -filtered_back_motor_spd/2);
    Serial.printf(">action:%f\n", action);

    // cybergearへのコマンド送信
    control_position(FRONT_MOTOR_ID, front_motor_target + offset_pos);
    if(!start_flag){
        action = 0.0f;
        back_motor_target = 0.0f;
    }

    if (micros() - pre_pid_time >= 2000) { // 2ms
        float dt_pid = (micros() - pre_pid_time) / 1000000.0f;
        velocity_type_pid_control(-action, back_motor_spd, dt_pid);
        control_current(BACK_MOTOR_ID, back_motor_target);
        // Serial.printf("obs: %.3f, %.3f, %.3f | action: %.3f | target_current: %.3f\n", policy_obs[0], policy_obs[1], policy_obs[2], action, back_motor_target);
        pre_pid_time = micros();
    }
}

// --- 専用制御関数 ---

void control_position(uint8_t motor_id, float rad) {
    send_parameter_write(motor_id, INDEX_TARGET_POS, rad, 0);
}

void control_current(uint8_t motor_id, float ampere) {
    send_parameter_write(motor_id, INDEX_TARGET_CUR, ampere, 0);
}

void change_mode(uint8_t motor_id, uint8_t mode) {
    send_parameter_write(motor_id, INDEX_RUN_MODE, (float)mode, 1);
    delay(50);
}

// --- 通信基盤関数 ---

void enable_motor(uint8_t motor_id) {
    uint32_t id = ((uint32_t)MODE_MOTOR_ENABLE << 24) | ((uint32_t)MASTER_ID << 8) | motor_id;
    uint8_t dummy[8] = {0};
    CAN0.sendMsgBuf(id, 1, 0, dummy);
}

void set_zero_position(uint8_t motor_id) {
    uint32_t id = ((uint32_t)MODE_SET_ZERO_POS << 24) | ((uint32_t)MASTER_ID << 8) | motor_id;
    uint8_t dummy[8] = {0};
    CAN0.sendMsgBuf(id, 1, 8, dummy);
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

void init_can() {
    if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
        CAN0.setMode(MCP_NORMAL);
    } else {
        while(1) delay(10);
    }
}

// PID
void velocity_type_pid_control(float target, float actual, float dt) {
    float error = target - actual;
    float prop = error - pre_error;
    float deriv = prop - pre_prop;
    integral += error;
    float du = kp_vel * prop + ki_vel * error * dt + kd_vel * deriv;
    pre_error = error;
    pre_prop = prop;
    output += du;
    back_motor_target = constrain(output, u_min, u_max);
}
