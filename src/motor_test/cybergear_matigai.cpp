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

#define MOTOR2_ID  0x7F

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

#define INDEX_KP_ANG          0x701E

#define INDEX_KP_VEL          0x701F

#define INDEX_KI_VEL          0x7020



#define MODE_PARAM_READ 0x11

#define INDEX_ENCODER_RAW  0x3004

#define INDEX_MECH_POS     0x3016

#define INDEX_MECH_VEL     0x3017

#define INDEX_IQ           0x3020



void send_parameter_read(uint8_t motor_id, uint16_t param_index) {

    uint32_t id = ((uint32_t)MODE_PARAM_READ << 24) | ((uint32_t)MASTER_ID << 8) | motor_id;

    uint8_t data[8] = {0};



    data[0] = param_index & 0xFF;

    data[1] = (param_index >> 8) & 0xFF;



    CAN0.sendMsgBuf(id, 1, 8, data);

}



// ゲイン

float kp_ang = 0.0f;

float kp_vel = 0.0;

float ki_vel = 0.0f;



// --- モード定義 ---

#define CONTROL_MODE_POS      1

#define CONTROL_MODE_SPD      2

#define CONTROL_MODE_CUR      3



// --- 制御目標値 (テスト用) ---

float target_position = 1.57f; // 90度

float target_velocity = 5.0f;   // 2 rad/s

float target_current  = 1.0f;   // 0.3 A



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

void set_gains(uint8_t motor_id, float p_ang, float p_vel, float i_vel);



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



    enable_motor(MOTOR2_ID);

    delay(100);

    set_zero_position(MOTOR2_ID);



    // 初期モード設定

    change_mode(MOTOR2_ID, current_mode);

}



float spd = 0.0;

float kp_read = 0.0;

float ki_read = 0.0;





void loop() {

    M5.update();    



    // // 画面表示

    // M5.Lcd.setCursor(0, 100);

    // M5.Lcd.printf("Current Mode: %d\n", current_mode);



    // 指が触れている間だけ各モードの目標値を送信

    if (M5.Touch.getCount() > 0) {

        control_velocity(MOTOR2_ID, target_velocity);

        // control_current(MOTOR2_ID, 0.3f);

    } else {

        // 離している時は停止/原点復帰

        control_velocity(MOTOR2_ID, 0.0f);

        // control_current(MOTOR2_ID, 0.0f);

    }



    // シリアル入力の処理

    if (Serial.available()) {

        char c = Serial.read();



        switch (c) {

            case 'q': kp_vel += 0.001f; break;

            case 'a': kp_vel -= 0.001f; break;

            case 'w': ki_vel += 0.0001f; break;

            case 's': ki_vel -= 0.0001f; break;

        }

    }

   

    set_gains(MOTOR2_ID, kp_ang, kp_vel, ki_vel);



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

            float now = micros();



            // Serial.printf("%f,%f,%f,%.3f,%.3f\n", kp_ang,kp_vel,ki_vel,current_spd, current_trq);

            spd = current_spd;

        }



        if (mode == 0x11) {  // ← パラメータRead応答

            uint16_t index = (buf[1] << 8) | buf[0];



            if (index == 0x701F) {

                float kp;

                memcpy(&kp, &buf[4], 4);

                // Serial.print("spd_kp: ");

                // Serial.println(kp);

                kp_read = kp;

            }



            if (index == 0x7020) {

                float ki;

                memcpy(&ki, &buf[4], 4);

                // Serial.print("spd_ki: ");

                // Serial.println(ki);

                ki_read = ki;

            }



            // if (index == 0x701E) {

            //     float kp_ang;

            //     memcpy(&kp_ang, &buf[4], 4);

            //     Serial.print("pos_kp: ");

                // Serial.println(kp_ang);

            // }

        }

    }



    send_parameter_read(MOTOR2_ID, INDEX_KP_VEL);

    send_parameter_read(MOTOR2_ID, INDEX_KI_VEL);

    Serial.print(">vel:");

    Serial.println(spd);

    Serial.print(">kp:");

    Serial.printf("%f\n",kp_read);

    Serial.print(">ki:");

    Serial.printf("%f\n",ki_read);



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



void set_gains(uint8_t motor_id, float p_ang, float p_vel, float i_vel) {

    send_parameter_write(motor_id, INDEX_KP_ANG, p_ang, 0);

    delay(5); // 連続送信時は少し待機

    send_parameter_write(motor_id, INDEX_KP_VEL, p_vel, 0);

    delay(5);

    send_parameter_write(motor_id, INDEX_KI_VEL, i_vel, 0);

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

    dummy[0] = 1;

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