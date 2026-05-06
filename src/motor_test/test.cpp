// #include <M5Unified.h>
// #include <mcp_can.h>
// #include <SPI.h>
// #include <vector>

// // --- ピン・ハードウェア設定 ---
// #define CAN0_INT 15
// const int SPI_CS_PIN = 27; 
// MCP_CAN CAN0(SPI_CS_PIN);
// uint8_t buf[8];

// // --- モーターID等 ---
// #define MOTOR2_ID  0x7F
// #define MASTER_ID 0x00

// // --- CyberGear レジスタ・モード ---
// #define MODE_MOTOR_ENABLE     0x03   
// #define MODE_MOTOR_STOP       0x04
// #define MODE_SET_ZERO_POS     0x06   
// #define MODE_PARAM_WRITE      0x12   
// #define INDEX_RUN_MODE         0x7005 
// #define INDEX_TARGET_CUR       0x7006 
// #define CONTROL_MODE_CUR       3

// // --- データ記録用設定 (PSRAM活用) ---
// struct DataLog {
//     float time;
//     float target_vel;
//     float actual_vel;
//     float current;
// };
// std::vector<DataLog> log_buffer; // PSRAMが有効な環境では自動的にヒープ(PSRAM)を使います
// const int MAX_LOG_SIZE = 12000;  // 12秒分 (10ms周期)

// // --- 制御・状態管理 ---
// float target_velocity = 0.0f;
// float target_current  = 0.0f;
// bool is_measuring = false;
// unsigned long start_time = 0;
// unsigned long last_ctrl_time = 0;

// float pre_error = 0.0, pre_prop = 0.0, output = 0.0;
// float kp_vel = 0.48f;  // Simで調整した値に合わせる
// float ki_vel = 0.00086f;
// float kd_vel = 0.0f;

// float spd = 0.0;

// // プロトタイプ宣言
// void init_can();
// void stop_motor(uint8_t motor_id);
// void enable_motor(uint8_t motor_id);
// void change_mode(uint8_t motor_id, uint8_t mode);
// void send_parameter_write(uint8_t motor_id, uint16_t param_index, float value, uint8_t is_byte = 0);
// void velocity_type_pid_control(float target, float actual, float dt);
// float uint_to_float(uint16_t x, float x_min, float x_max, int bits);

// void setup() {
//     auto cfg = M5.config();
//     M5.begin(cfg);
//     Serial.begin(115200);

//     // PSRAMの確認と予約
//     if (psramFound()) {
//         log_buffer.reserve(MAX_LOG_SIZE);
//         Serial.println("PSRAM Found. Buffer reserved.");
//     }

//     init_can();
//     delay(500);

//     stop_motor(MOTOR2_ID);
//     delay(100);
//     change_mode(MOTOR2_ID, CONTROL_MODE_CUR);
//     delay(100);
//     enable_motor(MOTOR2_ID);
    
//     Serial.println("Ready. Touch Screen to Start Sequence.");
// }

// void loop() {
//     M5.update();

//     // 1. シーケンス開始判定
//     if (M5.Touch.getCount() > 0 && !is_measuring) {
//         is_measuring = true;
//         start_time = millis();
//         log_buffer.clear();
//     }

//     // 2. CAN受信 (常に最新の速度を取得)
//     while (CAN0.checkReceive() == CAN_MSGAVAIL) {
//         unsigned long rxId;
//         unsigned char len = 0;
//         CAN0.readMsgBuf(&rxId, &len, buf); 
//         if (((rxId & 0x1FFFFFFF) >> 24 & 0x1F) == 0x02) {
//             uint16_t spd_raw = (buf[2] << 8) | buf[3];
//             spd = uint_to_float(spd_raw, -30.0f, 30.0f, 16); 
//         }
//     }

//     // 3. 制御・記録ループ (10ms周期)
//     if (is_measuring) {
//         unsigned long now = millis();
//         float elapsed = (now - start_time) / 1000.0f;

//         if (now - last_ctrl_time >= 10) {
//             float dt = (now - last_ctrl_time) / 1000.0f;
//             last_ctrl_time = now;

//             // --- ターゲット速度生成 (Simと同じロジック) ---
//             if (elapsed < 5.0f)      target_velocity = floor(elapsed) * 2.0f;
//             else if (elapsed < 6.0f) target_velocity = 10.0f;
//             else if (elapsed < 10.0f) target_velocity = 10.0f - floor(elapsed - 5.0f) * 2.0f;
//             else                     target_velocity = 0.0f;

//             // PID制御
//             velocity_type_pid_control(target_velocity, spd, dt);
//             send_parameter_write(MOTOR2_ID, INDEX_TARGET_CUR, target_current, 0);

//             // バッファに格納
//             if (log_buffer.size() < MAX_LOG_SIZE) {
//                 log_buffer.push_back({elapsed, target_velocity, spd, target_current});
//             }

//             // 12秒経過で終了
//             if (elapsed >= 12.0f) {
//                 is_measuring = false;
//                 send_parameter_write(MOTOR2_ID, INDEX_TARGET_CUR, 0.0f, 0);
//                 Serial.println("DATA_START");
//                 Serial.println("time_sec,target_vel,speed,torque");
//                 for (const auto& log : log_buffer) {
//                     Serial.printf("%.3f,%.3f,%.3f\n", log.time, log.target_vel, log.actual_vel);
//                 }
//                 Serial.println("DATA_END");
//             }
//         }
//     }
// }

// // --- 制御ロジック・通信関数 ---
// void velocity_type_pid_control(float target, float actual, float dt) {
//     float error = target - actual;
//     float prop = error - pre_error;
//     float deriv = prop - pre_prop;
//     float du = kp_vel * prop + ki_vel * error + kd_vel * deriv;
//     pre_error = error;
//     pre_prop = prop;
//     output += du;
//     target_current = constrain(output, -23.0f, 23.0f);
// }

// float uint_to_float(uint16_t x, float x_min, float x_max, int bits) {
//     float span = x_max - x_min;
//     return (float)x * span / ((1 << bits) - 1) + x_min;
// }

// void init_can() {
//     if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
//         CAN0.setMode(MCP_NORMAL);
//         Serial.println("CAN OK");
//     } else {
//         while(1) { Serial.println("CAN Fail"); delay(1000); }
//     }
// }

// void stop_motor(uint8_t motor_id) {
//     uint32_t id = ((uint32_t)MODE_MOTOR_STOP << 24) | motor_id;
//     uint8_t d[8] = {0};
//     CAN0.sendMsgBuf(id, 1, 8, d);
// }

// void enable_motor(uint8_t motor_id) {
//     uint32_t id = ((uint32_t)MODE_MOTOR_ENABLE << 24) | motor_id;
//     uint8_t d[8] = {0};
//     CAN0.sendMsgBuf(id, 1, 8, d);
// }

// void change_mode(uint8_t motor_id, uint8_t mode) {
//     send_parameter_write(motor_id, INDEX_RUN_MODE, (float)mode, 1);
// }

// void send_parameter_write(uint8_t motor_id, uint16_t param_index, float value, uint8_t is_byte) {
//     uint32_t id = ((uint32_t)MODE_PARAM_WRITE << 24) | motor_id;
//     uint8_t data[8] = {0};
//     data[0] = param_index & 0xFF;
//     data[1] = (param_index >> 8) & 0xFF;
//     if (is_byte) data[4] = (uint8_t)value;
//     else memcpy(&data[4], &value, 4);
//     CAN0.sendMsgBuf(id, 1, 8, data);
// }