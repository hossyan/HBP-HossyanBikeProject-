// #include <M5Unified.h>
// #include <mcp_can.h>
// #include <SPI.h>

// // --- ピン・ハードウェア設定 ---
// #define CAN0_INT 15
// const int SPI_CS_PIN = 27; 
// MCP_CAN CAN0(SPI_CS_PIN);
// uint32_t rxId;
// unsigned char len = 0;
// unsigned char buf[8];

// // --- モーター基本設定 ---
// #define MOTOR1_ID  0x7F
// #define MOTOR2_ID  0x7F
// #define MASTER_ID 0x00

// // --- CyberGear 通信モード (拡張ID上位5bit) ---
// #define MODE_MOTOR_ENABLE     0x03   
// #define MODE_MOTOR_STOP       0x04
// #define MODE_SET_ZERO_POS     0x06   
// #define MODE_PARAM_WRITE      0x12   
// #define MODE_PARAM_READ       0x11

// // --- CyberGear 内部レジスタインデックス ---
// #define INDEX_RUN_MODE        0x7005 // 1:位置, 2:速度, 3:電流
// #define INDEX_TARGET_POS      0x7016 // 目標位置 (float, rad)
// #define INDEX_TARGET_SPD      0x700A // 目標速度 (float, rad/s)
// #define INDEX_TARGET_CUR      0x7006 // 目標電流 (float, A)
// #define INDEX_LIMIT_SPD       0x7017 // 速度制限 (float)
// #define INDEX_KP_ANG          0x701E
// #define INDEX_KP_VEL          0x701F
// #define INDEX_KI_VEL          0x7020

// #define INDEX_ENCODER_RAW  0x3004
// #define INDEX_MECH_POS     0x3016
// #define INDEX_MECH_VEL     0x3017
// #define INDEX_IQ           0x3020

// // --- モード定義 ---
// #define CONTROL_MODE_POS      1
// #define CONTROL_MODE_SPD      2
// #define CONTROL_MODE_CUR      3

// // --- 制御・状態管理 ---
// float target_position = 1.57f; // 90度
// float target_velocity = 5.0f;  // テスト用: 5 rad/s
// float target_current  = 1.0f;  // 0.3 A
// int current_mode = CONTROL_MODE_SPD; 
// bool is_running = false; // タッチ状態管理用

// // ゲイン管理 (初期値を設定しておく)
// float kp_vel = 0.5f;
// float ki_vel = 0.001f;

// // 読み取り値格納用
// float spd = 0.0;
// float kp_read = 0.0;
// float ki_read = 0.0;
// unsigned long last_update_time = 0;

// // 関数プロトタイプ
// void init_can();
// void stop_motor(uint8_t motor_id);
// void enable_motor(uint8_t motor_id);
// void set_zero_position(uint8_t motor_id);
// void send_parameter_read(uint8_t motor_id, uint16_t param_index);
// void send_parameter_write(uint8_t motor_id, uint16_t param_index, float value, uint8_t is_byte = 0);
// void control_velocity(uint8_t motor_id, float rad_s);
// void change_mode(uint8_t motor_id, uint8_t mode);

// // uint16をfloatに変換する関数
// float uint_to_float(uint16_t x, float x_min, float x_max, int bits) {
//     float span = x_max - x_min;
//     float offset = x_min;
//     return (float)x * span / ((1 << bits) - 1) + offset;
// }

// void setup() {
//     M5.begin();
//     Serial.begin(115200);
//     Serial.println("\n--- CyberGear Unified Control ---");
//     Serial.println("Touch screen to Spin, Release to Stop.");
//     Serial.println("Keys: 'q'/ 'a' -> kp_vel UP/DOWN, 'w'/ 's' -> ki_vel UP/DOWN");

//     init_can();
//     delay(500);

//     // マニュアルの指示通り「停止 -> モード変更 -> 有効化」の順で行う
//     stop_motor(MOTOR2_ID);
//     delay(100);
    
//     change_mode(MOTOR2_ID, current_mode);
//     delay(100);

//     // 初期のゲインを書き込む
//     send_parameter_write(MOTOR2_ID, INDEX_KP_VEL, kp_vel, 0);
//     delay(10);
//     send_parameter_write(MOTOR2_ID, INDEX_KI_VEL, ki_vel, 0);
//     delay(10);

//     set_zero_position(MOTOR2_ID);
//     delay(100);

//     enable_motor(MOTOR2_ID);
//     delay(100);
    
//     Serial.println("Setup Completed.");
// }

// void loop() {
//     M5.update();    

//     // --- 1. タッチ操作による回転制御 (状態管理) ---
//     bool is_touched = (M5.Touch.getCount() > 0);

//     if (is_touched && !is_running) {
//         is_running = true;
//         control_velocity(MOTOR2_ID, target_velocity);
//         Serial.println("Status: RUNNING");
//     } 
//     else if (!is_touched && is_running) {
//         is_running = false;
//         control_velocity(MOTOR2_ID, 0.0f);
//         Serial.println("Status: STOPPED");
//     }

//     // --- 2. シリアル入力によるゲイン変更 ---
//     if (Serial.available()) {
//         char c = Serial.read();
//         bool gain_changed = false;

//         switch (c) {
//             case 'q': kp_vel += 0.1f; gain_changed = true; break;
//             case 'a': kp_vel -= 0.1f; gain_changed = true; break;
//             case 'w': ki_vel += 0.001f; gain_changed = true; break;
//             case 's': ki_vel -= 0.001f; gain_changed = true; break;
//         }

//         // 下限ガード
//         if (kp_vel < 0.0f) kp_vel = 0.0f;
//         if (ki_vel < 0.0f) ki_vel = 0.0f;

//         // 変更があった時だけCANで送信
//         if (gain_changed) {
//             send_parameter_write(MOTOR2_ID, INDEX_KP_VEL, kp_vel, 0);
//             delay(5);
//             send_parameter_write(MOTOR2_ID, INDEX_KI_VEL, ki_vel, 0);
//             Serial.printf("[Gain Set] kp_vel: %.3f, ki_vel: %.4f\n", kp_vel, ki_vel);
//         }
//     }

//     // --- 3. CAN受信処理 (※ここを正しく修正しました！) ---
//     while (CAN0.checkReceive() == CAN_MSGAVAIL) {
//         unsigned long rxId; // ← 型を mcp_can の要求に厳密に合わせる
        
//         // エラーが出ていた関数を、正しい3つの引数に戻す
//         CAN0.readMsgBuf(&rxId, &len, buf); 

//         uint32_t cleanId = rxId & 0x1FFFFFFF;
//         uint8_t mode = (cleanId >> 24) & 0x1F;

//         if (mode == 0x02) { // フィードバックフレーム
//             uint16_t spd_raw = (buf[2] << 8) | buf[3];
//             spd = uint_to_float(spd_raw, -30.0f, 30.0f, 16); 
//         }
//         else if (mode == 0x11) { // パラメータRead応答
//             uint16_t index = (buf[1] << 8) | buf[0];
//             if (index == 0x701F) {
//                 memcpy(&kp_read, &buf[4], 4);
//             }
//             if (index == 0x7020) {
//                 memcpy(&ki_read, &buf[4], 4);
//             }
//         }
//     }

//     // --- 4. 100ms周期でのデータ要求とシリアル出力 ---
//     if (millis() - last_update_time > 100) {
//         last_update_time = millis();

//         // ★追加：現在の目標速度を定期的に再送信する（これが最新速度フィードバックのトリガーになる）
//         control_velocity(MOTOR2_ID, is_running ? target_velocity : 0.0f);

//         // ゲインの読み取り要求
//         send_parameter_read(MOTOR2_ID, INDEX_KP_VEL);
//         send_parameter_read(MOTOR2_ID, INDEX_KI_VEL);

//         // シリアル出力
//         Serial.print(">vel:");
//         Serial.println(spd);
//         Serial.print(">kp:");
//         Serial.println(kp_read, 3);
//         Serial.print(">ki:");
//         Serial.println(ki_read, 4);
//     }
// }


// // --- CyberGear 通信基盤関数群 ---

// void init_can() {
//     pinMode(CAN0_INT, INPUT);
//     if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
//         CAN0.setMode(MCP_NORMAL);
//         Serial.println("CAN Init OK");
//     } else {
//         Serial.println("CAN Init Failed!");
//         while(1) delay(10);
//     }
// }

// void stop_motor(uint8_t motor_id) {
//     uint32_t id = ((uint32_t)MODE_MOTOR_STOP << 24) | ((uint32_t)MASTER_ID << 8) | motor_id;
//     uint8_t dummy[8] = {0};
//     CAN0.sendMsgBuf(id, 1, 8, dummy);
// }

// void enable_motor(uint8_t motor_id) {
//     uint32_t id = ((uint32_t)MODE_MOTOR_ENABLE << 24) | ((uint32_t)MASTER_ID << 8) | motor_id;
//     uint8_t dummy[8] = {0};
//     CAN0.sendMsgBuf(id, 1, 8, dummy);
// }

// void set_zero_position(uint8_t motor_id) {
//     uint32_t id = ((uint32_t)MODE_SET_ZERO_POS << 24) | ((uint32_t)MASTER_ID << 8) | motor_id;
//     uint8_t dummy[8] = {0};
//     dummy[0] = 1;
//     CAN0.sendMsgBuf(id, 1, 8, dummy);
// }

// void change_mode(uint8_t motor_id, uint8_t mode) {
//     send_parameter_write(motor_id, INDEX_RUN_MODE, (float)mode, 1);
// }

// void control_velocity(uint8_t motor_id, float rad_s) {
//     send_parameter_write(motor_id, INDEX_TARGET_SPD, rad_s, 0);
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

// void send_parameter_read(uint8_t motor_id, uint16_t param_index) {
//     uint32_t id = ((uint32_t)MODE_PARAM_READ << 24) | ((uint32_t)MASTER_ID << 8) | motor_id;
//     uint8_t data[8] = {0};

//     data[0] = param_index & 0xFF;
//     data[1] = (param_index >> 8) & 0xFF;

//     CAN0.sendMsgBuf(id, 1, 8, data);
// }