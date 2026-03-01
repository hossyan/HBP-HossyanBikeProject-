// #include <Wire.h>
// #include <M5Unified.h>
// #include <MadgwickAHRS.h>
// #include "parameters.h"
// #include <PS4Controller.h>

// Madgwick filter;
// unsigned long microsPerReading, microsPre;
// float accelScale, gyroScale;

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

// float ax,ay,az;
// float gx,gy,gz;
// float roll,pitch,yaw;
// unsigned long microsNow;

// #define left_motor_id 0x65
// #define right_motor_id 0x64

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

// float inc_kp = 0.001;
// float inc_ki = 0.0000001; 
// float inc_kd = 0.00001;

// float kp = 0.173;
// float ki = 0.0000002;
// float kd = 0.00247;

// float pre_time = 0.0;
// int current_min = 41;
// int current_max = 460;
// float target_angle = 0.0;
// float integral = 0.0;
// float pre_error = 0.0;

// float angle = 0.0;
// float angle_acc = 0.0;

// float pre_output[2] = {0.0, 0.0};

// float gx_rad = 0.0;
// float pre_roll = 0.0;
// float filtered_output = 0.0;

// float filtered_gx = 0.0;
// float filtered_gz = 0.0;

// float pre_ax=0.0;
// float pre_ay=0.0;
// float pre_az=0.0;
// float pre_gx=0.0;
// float pre_gy=0.0;
// float pre_gz=0.0;

// float stick_left_y = 0.0; 
// float stick_right_x = 0.0; 

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
//     M5.Display.printf("angle : %.3f", roll);
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
//         float speed_value = rpm_value * (2.0f * M_PI / 60.0f);
//         return speed_value;
//     }

//     return 0;
// }

// // AIへの入力用バッファ (5つの観測値)
// float input[5];   
// // 中間層のバッファ
// float layer1[64];
// float layer2[64];
// // AIからの出力 (右トルク, 左トルク)
// float output[1];  

// // AIの計算実行 (推論)
// void run_inference() {
//     // --- 第1層: input(5) -> layer1(64) ---
//     for (int i = 0; i < 64; i++) {
//         float sum = b1[i];
//         for (int j = 0; j < 5; j++) {
//             sum += W1[i * 5 + j] * input[j];
//         }
//         layer1[i] = tanhf(sum); 
//     }

//     // --- 第2層: layer1(64) -> layer2(64) ---
//     for (int i = 0; i < 64; i++) {
//         float sum = b2[i];
//         for (int j = 0; j < 64; j++) {
//             sum += W2[i * 64 + j] * layer1[j];
//         }
//         layer2[i] = tanhf(sum);
//     }

//     // --- 第3層: layer2(64) -> output(2) ---
//     for (int i = 0; i < 1; i++) {
//         float sum = b3[i];
//         for (int j = 0; j < 64; j++) {
//             sum += W3[i * 64 + j] * layer2[j];
//         }
//         // output[i] = tanhf(sum);
//         output[i] = constrain(sum, -1.0f, 1.0f);
//     }
// }

// void setup() {
//     auto cfg = M5.config();

//     M5.begin(cfg);
//     M5.Imu.begin();
//     Serial.begin(115200);
//     Wire.begin(); 
//     filter.begin(25);

//     M5.Display.setTextSize(2);
//     M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
//     M5.Display.fillScreen(TFT_BLACK);
//     PS4.begin("08:F9:E0:F5:E7:D6");
//     // button_draw();

//     Serial.println("--- Roller Motor Control Start ---");

//     set_motor_enable(left_motor_id, true);
//     set_motor_enable(right_motor_id, true);

//     set_control_mode(left_motor_id, current_mode);
//     set_control_mode(right_motor_id, current_mode);

//     microsPerReading = 1000000 / 25;
//     microsPre = micros();
//     }

// void loop() {
//     M5.update();

//     // pid_draw();
//     emergency_button_draw();
//     display_touch();

//     M5.Imu.getAccel(&ax, &ay, &az);
//     M5.Imu.getGyro(&gx, &gy, &gz);
//     float gx_rad = gx * (M_PI / 180.0f);
//     float gy_rad = gy * (M_PI / 180.0f);
//     float gz_rad = gz * (M_PI / 180.0f);

//     float alpha = 0.8;
//     filtered_gx = gx_rad * alpha + filtered_gx * (1-alpha); 
//     filtered_gz = gz_rad * alpha + filtered_gz * (1-alpha); 

//     // microsNow = micros();
//     // float dt = (microsNow - microsPre) / 1000000.0; 
//     // angle_acc = atan2(ay, az);
//     // roll = 0.98 * (roll + gx_rad * dt) + 0.02 * angle_acc;
//     // gx_rad = (roll - pre_roll) / dt;
//     // pre_roll = roll;
//     // microsPre = microsNow;

//     // Madgwickfilter
//     microsNow = micros();
//     float dt = (float)(microsNow - microsPre) / 1000000.0f;
//     if (dt > 0) {
//         filter.begin(1.0f / dt); 
//     }    

//     // float alpha = 0.2f;
//     // ax = ax*alpha+pre_ax*(1-alpha);
//     // ay = ay*alpha+pre_ay*(1-alpha);
//     // az = az*alpha+pre_az*(1-alpha);
//     // gx = gx*0.85+pre_gx*0.15;
//     // gy = gy*0.85+pre_gy*0.15;
//     // gz = gz*0.85+pre_gz*0.15;
//     // pre_ax, pre_ay, pre_az = ax,ay,az;

//     filter.updateIMU(gx, gy, gz, ax, ay, az);
//     roll = filter.getRollRadians();
//     // float alpha_roll = 0.9;
//     // roll = roll*alpha_roll + pre_roll*(1-alpha_roll);
//     microsPre = microsNow;

//     // 車輪回転速度(rad/s)取得
//     float left_wheel_speed = speed_read(left_motor_id);
//     float right_wheel_speed = speed_read(right_motor_id);

//     // コントローラ値取得
//     if (PS4.isConnected()) {
//         if (PS4.LStickY()) {
//             int stick_value = PS4.LStickY();
//             stick_left_y = (float)stick_value / 130; //正規化 
//             if(-0.1 < stick_left_y && stick_left_y < 0.1){
//                 stick_left_y = 0;
//             }
//         }
//         if (PS4.RStickX()) {
//             stick_right_x = PS4.RStickX(); 
//         }
//     }

//     // ニューラルネットワークの入力に代入
//     input[0] = roll;
//     input[1] = filtered_gx;
//     input[2] = left_wheel_speed;
//     input[3] = -right_wheel_speed;
//     input[4] = stick_left_y * 3;
//     // input[0] = 0.0024496778;
//     // input[1] = 0.22504774;
//     // input[2] = -2.7366974;
//     // input[3] = -2.7368207;
//     // input[4] = 0.0;


//     // for (int i = 0; i < 5; i++){
//     //     input[i] = 0;
//     // }

//     // auto now = millis();
//     // static auto pre = now;
//     // if (now - pre >= 10){
        
//     // }
//     run_inference();
//         // pre = now;
    
//     // int current_cmd = 0;
//     // if(output[0] > 0){
//     //     current_cmd = (int)((current_max - current_min) * abs(output[0]) + current_min);
//     // }else if(output[0] < 0){
//     //     current_cmd = (int)((current_max - current_min) * abs(output[0]) + current_min);
//     //     current_cmd = - current_cmd;
//     // }
//     int current_cmd = (int)(current_max * output[0]);
//     int out_L = current_cmd;
//     int out_R = -current_cmd;

//     // int out_L = -output[0] * current_max*0.5;
//     // int out_R = output[0] * current_max*0.5;

//     if (emergency_button == false){
//         out_L = 0.0;
//         out_R = 0.0;
//     }
//     set_current(left_motor_id, out_L);
//     set_current(right_motor_id, out_R);

//     // Serial.printf("%f, %f, %f\n",output[0], output[1], roll);
    
//     // Serial.printf("%f\n", dt);

//     Serial.print(">roll:");
//     Serial.println(input[0]);
//     Serial.print(">gx_roll:");
//     Serial.println(input[1]);
//     Serial.print(">wheel_vel:");
//     Serial.println(input[2]);
//     Serial.print(">contoroller:");
//     Serial.println(input[4]);
//     Serial.print(">output:");
//     Serial.println(output[0]);
//     Serial.print(">gz:");
//     Serial.println(filtered_gz);

//     // Serial.print(">speed:");
//     // Serial.printf("%f,%f\n", input[4], output[0]);
//     delay(1.5);
// }