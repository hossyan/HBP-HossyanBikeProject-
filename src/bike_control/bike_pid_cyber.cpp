#include <M5Unified.h>
#include <mcp_can.h>
#include <SPI.h>
#include <MadgwickAHRS.h>
#include <PS4Controller.h>

Madgwick filter;
unsigned long microsPerReading, microsPre;
float ax, ay, az;
float gx, gy, gz;
const float sampleRate = 100.0f;
float roll = 0.0f;
float roll_rad = 0.0f;

float gx_rad = 0.0f;
float filtered_gx = 0.0f;
float alpha = 0.8f; // フィルタ係数

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
float back_motor_target = 0.0f;  //A

bool power_on = false;

float target_state[3] = {-0.04f, 0.0f, 0.0f};

float kp = 0.00;
float ki = 0.00;
float kd = 0.00;
float kp_inc = 0.2;
float ki_inc = 0.001;
float kd_inc = 0.001;

float output = 0.0f;
float u_min = -23.0f;
float u_max = 23.0f;

float speed_max = 30.0f; // rad/s

//  関数プロトタイプ
void init_can();
void enable_motor(uint8_t motor_id);
void set_zero_position(uint8_t motor_id);
void send_parameter_write(uint8_t motor_id, uint16_t param_index, float value, uint8_t is_byte = 0);
void control_position(uint8_t motor_id, float rad);
void control_current(uint8_t motor_id, float ampere);
void control_velocity(uint8_t motor_id, float rad_s);
void change_mode(uint8_t motor_id, uint8_t mode);
float uint_to_float(uint16_t x, float x_min, float x_max, int bits);

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
PS4ButtonEdge btnUp, btnDown, btnLeft, btnRight, btnTriangle, btnCross, btnCircle;

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Imu.begin();
    Serial.begin(115200);
    filter.begin(sampleRate); // サンプルレートを設定
    PS4.begin("08:F9:E0:F5:E7:D6");
    init_can();
    delay(100);

    microsPerReading = 1000000 / sampleRate;
    microsPre = micros();

    M5.Display.setTextSize(3);

    // モーターの初期化
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

    change_mode(FRONT_MOTOR_ID, CONTROL_MODE_POS);
    change_mode(BACK_MOTOR_ID, CONTROL_MODE_SPD);
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
    while(CAN0.checkReceive() == CAN_MSGAVAIL) {
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

    // PS4コントローラの入力処理
    if(PS4.isConnected()) {
        if(btnUp.isPressedOnce(PS4.Up())) {
            kp += kp_inc;
        }
        if(btnDown.isPressedOnce(PS4.Down())) {
            kp -= kp_inc;
        }
        if(btnLeft.isPressedOnce(PS4.Left())) {
            ki += ki_inc;
        }
        if(btnRight.isPressedOnce(PS4.Right())) {
            ki -= ki_inc;
        }
        if(btnTriangle.isPressedOnce(PS4.Triangle())) {
            kd += kd_inc;
        }
        if(btnCross.isPressedOnce(PS4.Cross())) {
            kd -= kd_inc;
        }
        if(btnCircle.isPressedOnce(PS4.Circle())) {
            power_on = !power_on;
            M5.Display.setCursor(0, 10);
            M5.Display.printf("Power: %d\n", power_on);
        }
    }

    // ディスプレイ表示
    if(!power_on) {
        if(task100ms.check()) {
            if(abs(kp) < 0.0001) kp = 0.0;
            if(abs(ki) < 0.0001) ki = 0.0;
            if(abs(kd) < 0.0001) kd = 0.0;

            M5.Display.setCursor(0, 10);
            M5.Display.printf("Power: %d\n", power_on);
            M5.Display.printf("Roll: %6.2f\n", roll_rad);
            M5.Display.printf("Kp: %6.3f\n", kp);
            M5.Display.printf("Ki: %6.3f\n", ki);
            M5.Display.printf("Kd: %6.3f\n", kd);
            M5.Display.printf("Output: %6.3f\n", output); 
            M5.Display.printf("motor: %6.3f\n", output * speed_max);
        }
    }

    // フィードバック制御
    float roll_error = roll_rad - target_state[0];
    float roll_velocity_error = filtered_gx - target_state[1];
    float wheel_velocity_error = back_motor_spd - target_state[2];
    output = -kp * roll_error - ki * roll_velocity_error - kd * wheel_velocity_error;
    if (output > 1) output = 1;
    if (output < -1) output = -1;
    back_motor_target = output * speed_max; // 目標速度を設定（最大30rad/s）
    back_motor_target = kp;
    if (!power_on) {
        back_motor_target = 0.0f;
    }

    // cybergearの制御
    control_position(FRONT_MOTOR_ID, front_motor_target + offset_pos);
    control_velocity(BACK_MOTOR_ID, back_motor_target);

    Serial.printf(">SPD:");
    Serial.println(back_motor_spd);
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