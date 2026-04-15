// #include <Wire.h>
// #include <M5Unified.h>
// #include <math.h>

// #define M1_id 0x65
// #define M2_id 0x64

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

// #define BUTTON_X 15
// #define BUTTON_Y 15
// #define BUTTON_W 70
// #define BUTTON_H 50
// #define BUTTON_INC_X 110
// #define BUTTON_INC_Y 90

// #define EMERGENCY_BUTTON_X 50
// #define EMERGENCY_BUTTON_Y 200
// #define EMERGENCY_CIRCLE 35

// #define OUTPUT_X 130
// #define OUTPUT_Y 190

// bool buttonPressed = false;
// bool emergency_button = false;

// float inc_kp = 10;
// float inc_kd = 10;
// float inc_ki = 10;
// float kp = 50.0;
// float kd = 0.0;
// float ki = 0.0;

// // ボタン描画
// void button_draw() {
//     for(int i = 0; i < 3; i++) {
//         for(int j = 0; j < 2; j++){
//             int x = BUTTON_X + BUTTON_INC_X * i;
//             int y = BUTTON_Y + BUTTON_INC_Y * j;
//             M5.Display.fillRoundRect(x, y, BUTTON_W, BUTTON_H, 10, TFT_DARKGRAY);

//             int triY1 = y + (j==0 ? 15 : BUTTON_H - 15);
//             int triY2 = y + (j==0 ? BUTTON_H - 18 : 18);
//             M5.Display.fillTriangle(x + BUTTON_W / 2 , triY1, x + 20, triY2, x + BUTTON_W - 20, triY2, TFT_BLACK);
//         }
//     }
// }

// // PID値表示
// void pid_draw() {
//     float pid_vals[3] = {kp, ki, kd};
//     const char* pid_names[3] = {"kp", "ki", "kd"};

//     for(int i = 0; i < 3; i++) {
//         int x_val = BUTTON_X + BUTTON_INC_X * i - (i==1 ? 10 : 0);
//         int y_val = BUTTON_Y + BUTTON_H + (BUTTON_INC_Y - BUTTON_H) / 2;
//         int x_name = BUTTON_X + BUTTON_INC_X * i + BUTTON_W / 2 - 10;
//         int y_name = y_val -15;

//         M5.Display.setCursor(x_val, y_val);
//         M5.Display.printf(i==0 ? "%.3f" : i==1 ? "%.7f" : "%.5f", pid_vals[i]);

//         M5.Display.setCursor(x_name, y_name);
//         M5.Display.print(pid_names[i]);
//     }
// }

// // 緊急停止
// void emergency_button_draw(){
//     static bool old_emergency_button = !emergency_button;  
//     if (emergency_button != old_emergency_button) {
//         if (emergency_button == true) {
//             M5.Display.setTextColor(TFT_WHITE, TFT_GREEN);
//             M5.Display.fillCircle(EMERGENCY_BUTTON_X, EMERGENCY_BUTTON_Y, EMERGENCY_CIRCLE, TFT_GREEN);
//             M5.Display.drawCenterString("ON", EMERGENCY_BUTTON_X,EMERGENCY_BUTTON_Y - 5);
//         } else {
//             M5.Display.setTextColor(TFT_WHITE, TFT_RED);
//             M5.Display.fillCircle(EMERGENCY_BUTTON_X, EMERGENCY_BUTTON_Y,EMERGENCY_CIRCLE, TFT_RED);
//             M5.Display.drawCenterString("OFF", EMERGENCY_BUTTON_X,EMERGENCY_BUTTON_Y - 5);
//         }
//         old_emergency_button = emergency_button;  
//         M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
//     }

//     M5.Display.setCursor(OUTPUT_X, OUTPUT_Y);
// }

// // 画面タッチ認識
// void display_touch() {
//     if (M5.Touch.getCount() > 0) {
//         auto t = M5.Touch.getDetail();
//         if(!buttonPressed) {
//             buttonPressed = true;
//             for(int i = 0; i < 3; i++){
//                 for(int j = 0; j < 2; j++){
//                     int x = BUTTON_X + BUTTON_INC_X * i;
//                     int y = BUTTON_Y + BUTTON_INC_Y * j;
//                     if(t.x > x && t.x < x + BUTTON_W && t.y > y && t.y < y + BUTTON_H){
//                         float inc = (i==0 ? inc_kp : i==1 ? inc_ki : inc_kd);
//                         if(j == 0) { if(i==0) kp += inc; else if(i==1) ki += inc; else if(i==2) kd += inc;}
//                         else { if(i==0) kp -= inc; else if(i==1) ki -= inc; else if(i==2) kd -= inc; }
//                     }
//                 }
//             }
//             if(t.x > EMERGENCY_BUTTON_X - EMERGENCY_CIRCLE && t.x < EMERGENCY_BUTTON_X + EMERGENCY_CIRCLE &&
//                t.y > EMERGENCY_BUTTON_Y - EMERGENCY_CIRCLE && t.y < EMERGENCY_BUTTON_Y + EMERGENCY_CIRCLE){
//                 emergency_button = !emergency_button;
//             }
//         }
//     } else {
//         buttonPressed = false;
//     }
// }

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

// // roller485の電流制御
// void set_current(int8_t address, int32_t current) {
//     int32_t current_value = current * 100;

//     Wire.beginTransmission(address);
//     Wire.write(REG_CURRENT);

//     // 32ビットの値をリトルエンディアンで4バイトに分割して送信
//     Wire.write(current_value & 0xFF);
//     Wire.write((current_value >> 8) & 0xFF);
//     Wire.write((current_value >> 16) & 0xFF);
//     Wire.write((current_value >> 24) & 0xFF);

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
//     Serial.begin(115200);
//     Wire.begin(); 

//     M5.Display.setTextSize(2);
//     M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
//     M5.Display.fillScreen(TFT_BLACK);
//     // button_draw();

//     Serial.println("--- Roller Motor Control Start ---");

//     set_motor_enable(M1_id, true);

//     set_control_mode(M1_id, current_mode);
// }
// float pre_now = 0.0;


// void loop() {
//     M5.update();

//     button_draw();
//     emergency_button_draw();
//     pid_draw();
//     display_touch();    

//     int current_cmd = (int)(kp);

//     if (emergency_button == false){
//         current_cmd = 0.0;
//     }
//     float now = micros();
//     float delta = now - pre_now;
//     pre_now = now;

//     set_current(M1_id, current_cmd);
//     float M1_speed = speed_read(M1_id);
//     Serial.printf("%f,%f\n",delta,M1_speed);
// }