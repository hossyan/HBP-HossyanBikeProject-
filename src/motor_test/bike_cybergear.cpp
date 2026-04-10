// #include <M5Unified.h>
// #include <mcp_can.h>
// #include <SPI.h>
// #include <MadgwickAHRS.h>

// // --- ピン・ハードウェア設定 ---
// #define CAN0_INT 15
// const int SPI_CS_PIN = 27; 
// MCP_CAN CAN0(SPI_CS_PIN);
// long unsigned int rxId;
// unsigned char len = 0;
// unsigned char buf[8];

// // --- モーター基本設定 ---
// #define MOTOR1_ID  0x7F
// #define MOTOR2_ID  0x7E
// #define MASTER_ID 0x00

// // --- CyberGear 通信モード (拡張ID上位5bit) ---
// #define MODE_MOTOR_ENABLE     0x03   
// #define MODE_SET_ZERO_POS     0x06   
// #define MODE_PARAM_WRITE      0x12   

// // --- CyberGear 内部レジスタインデックス ---
// #define INDEX_RUN_MODE        0x7005 // 1:位置, 2:速度, 3:電流
// #define INDEX_TARGET_POS      0x7016 // 目標位置 (float, rad)
// #define INDEX_TARGET_SPD      0x700A // 目標速度 (float, rad/s)
// #define INDEX_TARGET_CUR      0x7006 // 目標電流 (float, A)
// #define INDEX_LIMIT_SPD       0x7017 // 速度制限 (float)

// // --- モード定義 ---
// #define CONTROL_MODE_POS      1
// #define CONTROL_MODE_SPD      2
// #define CONTROL_MODE_CUR      3

// // --- 制御目標値 (テスト用) ---
// float target_position = 1.57f; // 90度
// float target_velocity = 2.0f;   // 2 rad/s
// float target_current  = 0.3f;   // 0.3 A

// int current_mode = CONTROL_MODE_SPD; // デフォルトモード

// // 関数プロトタイプ
// void init_can();
// void enable_motor(uint8_t motor_id);
// void set_zero_position(uint8_t motor_id);
// void send_parameter_write(uint8_t motor_id, uint16_t param_index, float value, uint8_t is_byte = 0);

// // cybergear制御関数
// void control_position(uint8_t motor_id, float rad);
// void control_velocity(uint8_t motor_id, float rad_s);
// void control_current(uint8_t motor_id, float ampere);
// void change_mode(uint8_t motor_id, uint8_t mode);

// // --- 追加：uint16をfloatに変換する関数 ---
// float uint_to_float(uint16_t x, float x_min, float x_max, int bits) {
//     float span = x_max - x_min;
//     float offset = x_min;
//     return (float)x * span / ((1 << bits) - 1) + offset;
// }

// Madgwick filter;
// unsigned long microsPerReading, microsPre;
// unsigned long microsPre_rl = 0.0;
// unsigned long microsPre_pid = 0.0;
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

// float inc_kp = 0.001;
// float inc_ki = 0.0000001; 
// float inc_kd = 0.00001;

// float kp = 0.130;
// float ki = 0.0000002;
// float kd = 0.00180;

// float pre_time = 0.0;
// int speed_max = 500;
// float target_angle[1] = {0.0};
// float integral = 0.0;
// float pre_error = 0.0;

// float angle = 0.0;
// float angle_acc = 0.0;

// float pre_output[2] = {0.0, 0.0};

// float gx_rad = 0.0;
// float pre_roll = 0.0;
// float filtered_output = 0.0;

// float filtered_gx = 0.0;
// float filtered_gy = 0.0;
// float filtered_gz = 0.0;

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
//     M5.Display.printf("angle : %.3f", pitch);
// }

// // 画面タッチ認識（長押し対応版）
// void display_touch() {
//     if (M5.Touch.getCount() > 0) {
//         auto t = M5.Touch.getDetail();
        
//         // --- PID値の変更セクション（長押しで連続反応） ---
//         for(int i = 0; i < 3; i++){
//             for(int j = 0; j < 2; j++){
//                 int x = BUTTON_X + BUTTON_INC_X * i;
//                 int y = BUTTON_Y + BUTTON_INC_Y * j;
                
//                 if(t.x > x && t.x < x + BUTTON_W && t.y > y && t.y < y + BUTTON_H){
//                     float inc = (i==0 ? inc_kp : i==1 ? inc_ki : inc_kd);
                    
//                     // 必要に応じて変化速度を調整（例：inc * 0.1）
//                     if(j == 0) { 
//                         if(i==0) kp += inc; else if(i==1) ki += inc; else if(i==2) kd += inc;
//                     } else { 
//                         if(i==0) kp -= inc; else if(i==1) ki -= inc; else if(i==2) kd -= inc; 
//                     }
//                 }
//             }
//         }

//         // --- 緊急停止ボタン（誤作動防止のため、ここだけは「1回押し」を維持） ---
//         if(!buttonPressed) { // 前のフレームで押されていなかった場合のみ実行
//             if(t.x > EMERGENCY_BUTTON_X - EMERGENCY_CIRCLE && t.x < EMERGENCY_BUTTON_X + EMERGENCY_CIRCLE &&
//                t.y > EMERGENCY_BUTTON_Y - EMERGENCY_CIRCLE && t.y < EMERGENCY_BUTTON_Y + EMERGENCY_CIRCLE){
//                 emergency_button = !emergency_button;
//             }
//         }
//         buttonPressed = true; // 押されている状態を記録
//     } else {
//         buttonPressed = false; // 指が離れたらリセット
//     }
// }

// void setup() {
//     auto cfg = M5.config();
//     M5.begin(cfg);
//     M5.Imu.begin();
//     Serial.begin(115200);
//     filter.begin(25);

//     M5.Display.setTextSize(2);
//     M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
//     M5.Display.fillScreen(TFT_BLACK);
    
//     microsPerReading = 1000000 / 25;
//     microsPre = micros();

//     init_can();
//     delay(1000);

//     enable_motor(MOTOR1_ID);
//     enable_motor(MOTOR2_ID);
//     delay(100);

//     // 初期モード設定
//     change_mode(MOTOR1_ID, CONTROL_MODE_POS);
//     change_mode(MOTOR2_ID, CONTROL_MODE_SPD);
// }

// void loop() {
//     M5.update();

//     button_draw();
//     pid_draw();
//     emergency_button_draw();
//     display_touch();

//     M5.Imu.getAccel(&ax, &ay, &az);
//     M5.Imu.getGyro(&gx, &gy, &gz);
//     float gx_rad = gx * (M_PI / 180.0f);
//     float gy_rad = gy * (M_PI / 180.0f);
//     float gz_rad = gz * (M_PI / 180.0f);

//     filtered_gx = gx_rad * 0.8 + filtered_gx * 0.2; 
//     filtered_gy = gy_rad * 0.8 + filtered_gy * 0.2; 
//     filtered_gz = gz_rad * 0.8 + filtered_gz * 0.2; 

//     // Madgwickfilter
//     microsNow = micros();
//     float dt = (float)(microsNow - microsPre) / 1000000.0f;
//     if (dt > 0) {
//         filter.begin(1.0f / dt); 
//     }    

//     filter.updateIMU(gx, gy, gz, ax, ay, az);
//     pitch = filter.getPitch();
//     microsPre = microsNow;
    
//     float error = target_angle[0] - pitch;
//     integral += error * dt;
//     float diriv = (error - pre_error) / dt;
//     float output = kp * error + ki * integral + kd * diriv;
//     pre_error = error;

//     int current_cmd = (int)(output * speed_max);

//     if (emergency_button == false){
//         current_cmd = 0.0;
//     }

//     control_velocity(MOTOR2_ID, current_cmd);

//     delay(1); 
// }

// // --- ID指定対応：専用制御関数 ---

// void control_position(uint8_t motor_id, float rad) {
//     send_parameter_write(motor_id, INDEX_TARGET_POS, rad, 0);
// }

// void control_velocity(uint8_t motor_id, float rad_s) {
//     send_parameter_write(motor_id, INDEX_TARGET_SPD, rad_s, 0);
// }

// void control_current(uint8_t motor_id, float ampere) {
//     send_parameter_write(motor_id, INDEX_TARGET_CUR, ampere, 0);
// }

// void change_mode(uint8_t motor_id, uint8_t mode) {
//     send_parameter_write(motor_id, INDEX_RUN_MODE, (float)mode, 1);
//     delay(50);
// }

// // --- ID指定対応：通信基盤関数 ---

// void enable_motor(uint8_t motor_id) {
//     uint32_t id = ((uint32_t)MODE_MOTOR_ENABLE << 24) | ((uint32_t)MASTER_ID << 8) | motor_id;
//     uint8_t dummy[8] = {0};
//     CAN0.sendMsgBuf(id, 1, 0, dummy);
// }

// void set_zero_position(uint8_t motor_id) {
//     uint32_t id = ((uint32_t)MODE_SET_ZERO_POS << 24) | ((uint32_t)MASTER_ID << 8) | motor_id;
//     uint8_t dummy[8] = {0};
//     CAN0.sendMsgBuf(id, 1, 8, dummy);
// }

// void send_parameter_write(uint8_t motor_id, uint16_t param_index, float value, uint8_t is_byte) {
//     uint32_t id = ((uint32_t)MODE_PARAM_WRITE << 24) | ((uint32_t)MASTER_ID << 8) | motor_id;
//     uint8_t data[8] = {0};
    
//     data[0] = param_index & 0xFF;
//     data[1] = (param_index >> 8) & 0xFF;

//     if (is_byte) {
//         data[4] = (uint8_t)value;
//     } else {
//         memcpy(&data[4], &value, 4);
//     }
//     CAN0.sendMsgBuf(id, 1, 8, data);
// }

// void init_can() {
//     if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
//         CAN0.setMode(MCP_NORMAL);
//     } else {
//         while(1) delay(10);
//     }
// }