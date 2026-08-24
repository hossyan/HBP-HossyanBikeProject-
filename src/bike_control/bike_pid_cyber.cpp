#include <M5Unified.h>
#include <mcp_can.h>
#include <MadgwickAHRS.h>
#include <PS4Controller.h>

Madgwick filter;
unsigned long microsPerReading, microsPre;
float ax, ay, az;
float gx, gy, gz;
const float sampleRate = 100.0f;
float roll = 0.0f;

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

bool power_on = false;

float kp = 0.00;
float ki = 0.00;
float kd = 0.00;
float kp_inc = 0.001;
float ki_inc = 0.001;
float kd_inc = 0.001;
float output = 0.0f;

//  関数群

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

    microsPerReading = 1000000 / sampleRate;
    microsPre = micros();

    M5.Display.setTextSize(3);
}

float pre_now = 0.0f;
void loop() {
    M5.update();

    unsigned long microsNow = micros();
    if (microsNow - microsPre >= microsPerReading) {
        M5.Imu.getAccelData(&ax, &ay, &az);
        M5.Imu.getGyroData(&gx, &gy, &gz);
        filter.updateIMU(gx, gy, gz, ax, ay, az);
        roll = filter.getRoll();
        microsPre = microsNow;
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
            M5.Display.printf("Roll: %6.2f\n", roll);
            M5.Display.printf("Kp: %6.3f\n", kp);
            M5.Display.printf("Ki: %6.3f\n", ki);
            M5.Display.printf("Kd: %6.3f\n", kd);
            M5.Display.printf("Output: %6.2f\n", output); 
        }
    }
    
 
}