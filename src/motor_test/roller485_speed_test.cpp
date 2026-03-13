// #include <Wire.h>
// #include <M5Unified.h>
// #include "parameters.h"
// #include <math.h>

// #define M1_id 0x64
// #define M2_id 0x65

// // レジスタアドレス
// #define REG_ENABLE         0x00      // モーター有効化レジスタ
// #define REG_MODE           0x01      // 動作モード設定レジスタ
// #define REG_SPEED        0x40      // 速度設定レジスタ
// #define REG_POSITION        0x80      // 位置設定レジスタ
// #define REG_CURRENT        0xB0      // 電流設定レジスタ
// #define Speed_Readback     0x60      // エンコーダ(rpm)読み取りレジスタ
// #define Position_Readback  0x90      // エンコーダ(位置)読み取りレジスタ


// #define speed_mode   0x01      //動作モード：速度制御
// #define position_mode   0x02      //動作モード：位置制御
// #define current_mode    0x03      //動作モード：電流制御

// // roller485のセットアップ
// void set_motor_enable(uint8_t address, bool enable) {
//     Wire.beginTransmission(address);
//     Wire.write(REG_ENABLE);
//     Wire.write(enable ? 0x01 : 0x00);
//     Wire.endTransmission();
// }

// // roller485の動作モード設定
// void set_control_mode(uint8_t address, uint8_t mode) {
//     Wire.beginTransmission(address);
//     Wire.write(REG_MODE);
//     Wire.write(mode);
//     Wire.endTransmission();
// }

// // roller485の速度制御
// void set_speed(int8_t address, int32_t speed) {
//     int32_t speed_value = speed * 100;

//     Wire.beginTransmission(address);
//     Wire.write(REG_SPEED);

//     // 32ビットの値をリトルエンディアンで4バイトに分割して送信
//     Wire.write(speed_value & 0xFF);
//     Wire.write((speed_value >> 8) & 0xFF);
//     Wire.write((speed_value >> 16) & 0xFF);
//     Wire.write((speed_value >> 24) & 0xFF);

//     Wire.endTransmission();
// }

// float speed_read(uint8_t address){
//     Wire.beginTransmission(address);
//     Wire.write(Speed_Readback);
//     Wire.endTransmission();

//     Wire.requestFrom(address, 4);

//     if(Wire.available() >= 4){
//         byte byte0 = Wire.read();
//         byte byte1 = Wire.read();
//         byte byte2 = Wire.read();
//         byte byte3 = Wire.read();

//         long rpm_value = (long)byte3 << 24 | (long)byte2 << 16 | (long)byte1 << 8 | (long)byte0;
//         rpm_value = rpm_value / 100; // データシートより
//         // float speed_value = rpm_value * (2.0f * M_PI / 60.0f);
//         return rpm_value;
//     }

//     return 0;
// }

// void setup() {
//     auto cfg = M5.config();

//     M5.begin(cfg);
//     M5.Imu.begin();
//     Serial.begin(115200);
//     Wire.begin(); 

//     M5.Display.setTextSize(2);
//     M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
//     M5.Display.fillScreen(TFT_BLACK);
//     // button_draw();

//     Serial.println("--- Roller Motor Control Start ---");

//     set_motor_enable(M1_id, true);

//     set_control_mode(M1_id, speed_mode);
//     }

// void loop() {
//     M5.update();

//     set_speed(M1_id, 200);
//     float M1_speed = speed_read(M1_id);
//     Serial.println(M1_speed);
// }