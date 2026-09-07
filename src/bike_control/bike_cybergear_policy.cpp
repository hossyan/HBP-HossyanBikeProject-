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
#define INDEX_TARGET_SPD      0x700A // 目標速度 (float, rad/s)
#define INDEX_TARGET_CUR      0x7006 // 目標電流 (float, A)

// --- モード定義 ---
#define CONTROL_MODE_POS      1
#define CONTROL_MODE_SPD      2
#define CONTROL_MODE_CUR      3

// --- 制御目標値 ---
float front_motor_target = 60 * M_PI / 180.0f; //60degree in radian
float offset_pos = 0.0f;
float back_motor_target = 0.0f;  //rad/s
float speed_max = 30.0f; // rad/s

// rlパラメータ
float obs[10] = {0.0f};
float action = 0.0;
float action_scale = 20.0;

// rlパラメータの履歴
float angle_hist[3] = {0.0f};
float gyro_hist[3] = {0.0f};
float tire_hist[3] = {0.0f};

// 角度推定　センサーフィルタリング用
Madgwick filter;
unsigned long microsPerReading, microsPre;
float ax,ay,az;
float gx,gy,gz;
const float sampleRate = 100.0f;
float roll = 0.0f;
float roll_rad = 0.0f;

float gx_rad = 0.0f;
float filtered_gx = 0.0;
float alpha = 0.8f; // フィルタ係数

// --- マルチコア用 ---
volatile float policy_obs[3] = {0.0, 0.0, 0.0};  // タスク間共有obs
volatile float policy_action = 0.0;               // タスク間共有action
portMUX_TYPE policy_mux = portMUX_INITIALIZER_UNLOCKED; // ミューテックス

TaskHandle_t policyTaskHandle = NULL;
#define POLICY_INTERVAL_MS 15

// --- PS4コントローラ用 ---
bool power_on = false;

// 関数プロトタイプ
void init_can();
void enable_motor(uint8_t motor_id);
void set_zero_position(uint8_t motor_id);
void send_parameter_write(uint8_t motor_id, uint16_t param_index, float value, uint8_t is_byte = 0);
void control_position(uint8_t motor_id, float rad);
void control_current(uint8_t motor_id, float ampere);
void control_velocity(uint8_t motor_id, float rad_s);
void change_mode(uint8_t motor_id, uint8_t mode);
float uint_to_float(uint16_t x, float x_min, float x_max, int bits);

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

// rlパラメータがhist付の時
// void policyTask(void *pvParameters) {
//     float local_obs[OBS_DIM];
//     float local_action = 0.0f;
//     TickType_t xLastWakeTime = xTaskGetTickCount(); // 起動時刻を記録
    
//     for (;;) {
//         portENTER_CRITICAL(&policy_mux);
//         for (int i = 0; i < OBS_DIM; i++) {
//             local_obs[i] = policy_obs[i];
//         }
//         policy_action = local_action; 
//         portEXIT_CRITICAL(&policy_mux);

//         local_action = policy_infer(local_obs) * action_scale;

//         vTaskDelay(1); 
//         vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(POLICY_INTERVAL_MS));
//     }
// }

struct TaskTimer {
    unsigned long last_time = 0;
    unsigned long interval;

    TaskTimer(unsigned long iv) : interval(iv) {}

    bool check() {
        unsigned long now = micros();
        if (now - last_time >= interval) {
            last_time = now;
            return true;
        }
        return false;
    }
};
TaskTimer task100ms(100000); // 100ms周期

struct PS4ButtonEdge {
    bool last_state = false;

    bool isPressedOnce(bool current_state) {
        if(!last_state && current_state) {
            last_state = current_state;
            return true;
        }
        last_state = current_state;
        return false;
    }
};
PS4ButtonEdge btnCircle;

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Imu.begin();
    Serial.begin(115200);
    filter.begin(sampleRate);
    PS4.begin("08:F9:E0:F5:E7:D6");
    init_can();
    delay(1000);
    
    microsPerReading = 1000000 / sampleRate;
    microsPre = micros();

    M5.Display.setTextSize(3);

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
    change_mode(BACK_MOTOR_ID, CONTROL_MODE_SPD);

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

    // IMUの更新
    unsigned long microsNow = micros();
    if (microsNow - microsPre >= microsPerReading) {
        M5.Imu.getAccelData(&ax, &ay, &az);
        M5.Imu.getGyroData(&gx, &gy, &gz);
        filter.updateIMU(gx, gy, gz, ax, ay, az);
        roll = filter.getRoll();
        microsPre = microsNow;

        roll_rad = roll * (M_PI / 180.0f);
        gx_rad = gx * (M_PI / 180.0f);

        filtered_gx = alpha * gx_rad + (1 - alpha) * filtered_gx; // 低域フィルタリング
    }

    // cybergearの角度・角速度取得
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

    // --- observation ---
    // 履歴バッファの更新
    // angle_hist[2] = angle_hist[1];
    // angle_hist[1] = angle_hist[0]; 
    // angle_hist[0] = -roll_rad;

    // gyro_hist[2] = gyro_hist[1];
    // gyro_hist[1] = gyro_hist[0];
    // gyro_hist[0] = -filtered_gx;

    // tire_hist[2] = tire_hist[1];
    // tire_hist[1] = tire_hist[0]; 
    // tire_hist[0] = -back_motor_spd / 2;

    // --- policy_obsの更新 ---
    portENTER_CRITICAL(&policy_mux);
    policy_obs[0] = -roll_rad;
    policy_obs[1] = -filtered_gx;
    policy_obs[2] = -back_motor_spd / 2;
    portEXIT_CRITICAL(&policy_mux);

    // portENTER_CRITICAL(&policy_mux);
    // policy_obs[0] = angle_hist[0];
    // policy_obs[1] = angle_hist[1];
    // policy_obs[2] = angle_hist[2];
    // policy_obs[3] = gyro_hist[0];
    // policy_obs[4] = gyro_hist[1]; 
    // policy_obs[5] = gyro_hist[2];
    // policy_obs[6] = tire_hist[0];
    // policy_obs[7] = tire_hist[1];
    // policy_obs[8] = tire_hist[2];
    // portEXIT_CRITICAL(&policy_mux);

    // --- policy結果を読み出す ---
    portENTER_CRITICAL(&policy_mux);
    action = policy_action;
    portEXIT_CRITICAL(&policy_mux);

    back_motor_target = constrain(-action * action_scale, -speed_max, speed_max);

    // PS4コントローラの入力処理
    if (PS4.isConnected()) {
        if (btnCircle.isPressedOnce(PS4.Circle())) {
            power_on = !power_on;
            M5.Display.setCursor(0, 10);
            M5.Display.printf("Power: %d\n", power_on);
        }
    }

    // ディスプレイ表示, 制御はpower_onがtrueのときのみ有効
    if(!power_on) {
        if(task100ms.check()) {
            M5.Display.setCursor(0, 10);
            M5.Display.printf("Power: %d\n", power_on);
            M5.Display.printf("Roll: %6.2f\n", policy_obs[0]);
            M5.Display.printf("Roll_vel: %6.2f\n", policy_obs[1]);
            M5.Display.printf("Back_spd: %6.2f\n", policy_obs[2]);
            M5.Display.printf("Action: %6.2f\n", action);
            M5.Display.printf("output: %6.2f\n", back_motor_target);
        }
        back_motor_target = 0.0f; // power_off時はバックモータを停止
    }

    // cybergearへのコマンド送信
    control_position(FRONT_MOTOR_ID, front_motor_target + offset_pos);
    control_velocity(BACK_MOTOR_ID, back_motor_target);
}

void init_can() {
    if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
        CAN0.setMode(MCP_NORMAL);
    } else {
        while(1) delay(10);
    }
}

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

void control_position(uint8_t motor_id, float rad) {
    send_parameter_write(motor_id, INDEX_TARGET_POS, rad, 0);
}

void control_current(uint8_t motor_id, float ampere) {
    send_parameter_write(motor_id, INDEX_TARGET_CUR, ampere, 0);
}

void control_velocity(uint8_t motor_id, float rad_s) {
    send_parameter_write(motor_id, INDEX_TARGET_SPD, rad_s, 0);
}

void change_mode(uint8_t motor_id, uint8_t mode) {
    send_parameter_write(motor_id, INDEX_RUN_MODE, (float)mode, 1);
    delay(50);
}

float uint_to_float(uint16_t x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (float)x * span / ((1 << bits) - 1) + offset;
}