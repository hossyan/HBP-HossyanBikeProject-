#include <M5Unified.h>
#include <mcp_can.h>
#include <SPI.h>

// --- ピン・ハードウェア設定 ---
#define CAN0_INT 15
const int SPI_CS_PIN = 27; 
MCP_CAN CAN0(SPI_CS_PIN);
long unsigned int rxId;
unsigned char len = 0;
unsigned char buf[8];

// --- モーター基本設定 ---
#define MOTOR1_ID  0x7F
#define MOTOR2_ID  0x7E
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
#define INDEX_LIMIT_SPD       0x7017 // 速度制限 (float)

// --- モード定義 ---
#define CONTROL_MODE_POS      1
#define CONTROL_MODE_SPD      2
#define CONTROL_MODE_CUR      3

// --- 制御目標値 (テスト用) ---
float target_position = 1.57f; // 90度
float target_velocity = 2.0f;   // 2 rad/s
float target_current  = 0.3f;   // 0.3 A

int current_mode = CONTROL_MODE_SPD; // デフォルトモード

// 関数プロトタイプ
void init_can();
void enable_motor(uint8_t motor_id);
void set_zero_position(uint8_t motor_id);
void send_parameter_write(uint8_t motor_id, uint16_t param_index, float value, uint8_t is_byte = 0);

// cybergear制御関数
void control_position(uint8_t motor_id, float rad);
void control_velocity(uint8_t motor_id, float rad_s);
void control_current(uint8_t motor_id, float ampere);
void change_mode(uint8_t motor_id, uint8_t mode);

// --- 追加：uint16をfloatに変換する関数 ---
float uint_to_float(uint16_t x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (float)x * span / ((1 << bits) - 1) + offset;
}

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    Serial.begin(115200);
    
    M5.Lcd.println("CyberGear Unified Control");
    M5.Lcd.println("Touch to switch Mode (1->2->3)");

    init_can();
    delay(1000);

    enable_motor(MOTOR1_ID);
    enable_motor(MOTOR2_ID);
    delay(100);

    // 初期モード設定
    change_mode(MOTOR1_ID, current_mode);
    change_mode(MOTOR2_ID, current_mode);
}

void loop() {
    M5.update();

    // 画面表示
    M5.Lcd.setCursor(0, 100);
    M5.Lcd.printf("Current Mode: %d\n", current_mode);

    // 指が触れている間だけ各モードの目標値を送信
    if (M5.Touch.getCount() > 0) {
        control_velocity(MOTOR1_ID, target_velocity);
        control_velocity(MOTOR2_ID, target_velocity);
        switch (current_mode) {
            // case CONTROL_MODE_POS: control_position(MOTOR1_ID, target_position); control_position(MOTOR2_ID, target_position); break;
            // case CONTROL_MODE_SPD: control_velocity(MOTOR1_ID, target_velocity); control_velocity(MOTOR2_ID, target_velocity); break;
            // case CONTROL_MODE_CUR: control_current(MOTOR1_ID, target_current);   control_current(MOTOR2_ID, target_current);   break;
        }
    } else {
        // 離している時は停止/原点復帰
        control_velocity(MOTOR1_ID, 0.0f);
        control_velocity(MOTOR2_ID, 0.0f);
        switch (current_mode) {
            // case CONTROL_MODE_POS: control_position(MOTOR1_ID, 0.0f); control_position(MOTOR2_ID, 0.0f); break;
            // case CONTROL_MODE_SPD: control_velocity(MOTOR1_ID, 0.0f); control_velocity(MOTOR2_ID, 0.0f); break;
            // case CONTROL_MODE_CUR: control_current(MOTOR1_ID, 0.0f);  control_current(MOTOR2_ID, 0.0f);  break;
        }
    }

    while (CAN0.checkReceive() == CAN_MSGAVAIL) {
        CAN0.readMsgBuf(&rxId, &len, buf);

        uint32_t cleanId = rxId & 0x1FFFFFFF;

        uint8_t source_motor_id = (cleanId >> 8) & 0xFF;
        uint8_t mode = (cleanId >> 24) & 0x1F;

        if (mode == 0x02) {
            uint16_t pos_raw = (buf[0] << 8) | buf[1];
            uint16_t spd_raw = (buf[2] << 8) | buf[3];
            uint16_t trq_raw = (buf[4] << 8) | buf[5];

            float current_pos = uint_to_float(pos_raw, -12.5f, 12.5f, 16);
            float current_spd = uint_to_float(spd_raw, -45.0f, 45.0f, 16);
            float current_trq = uint_to_float(trq_raw, -12.0f, 12.0f, 16);

            Serial.printf("MotorID: 0x%02X | Pos: %6.2f rad | Spd: %6.2f rad/s | Trq: %6.2f Nm\n", 
                          source_motor_id, current_pos, current_spd, current_trq);
        }
    }


    delay(1); 
}

// --- ID指定対応：専用制御関数 ---

void control_position(uint8_t motor_id, float rad) {
    send_parameter_write(motor_id, INDEX_TARGET_POS, rad, 0);
}

void control_velocity(uint8_t motor_id, float rad_s) {
    send_parameter_write(motor_id, INDEX_TARGET_SPD, rad_s, 0);
}

void control_current(uint8_t motor_id, float ampere) {
    send_parameter_write(motor_id, INDEX_TARGET_CUR, ampere, 0);
}

void change_mode(uint8_t motor_id, uint8_t mode) {
    send_parameter_write(motor_id, INDEX_RUN_MODE, (float)mode, 1);
    delay(50);
}

// --- ID指定対応：通信基盤関数 ---

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