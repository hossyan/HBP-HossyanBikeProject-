// #include <M5Unified.h>
// #include <mcp_can.h>
// #include <SPI.h>

// // 20,000サンプル (約240KB)
// const uint32_t MAX_SAMPLES = 20000;

// // センサーデータの構造体
// struct SensorData {
//   uint32_t time_us; 
//   float spd;
//   float torque;
// };

// SensorData* dataLog = nullptr;
// uint32_t sampleCount = 0;
// bool isMeasuring = false;
// bool sending = true;

// // --- ピン・ハードウェア設定 ---
// #define CAN0_INT 15
// const int SPI_CS_PIN = 27; 
// MCP_CAN CAN0(SPI_CS_PIN);
// uint32_t rxId;
// unsigned char len = 0;
// unsigned char buf[8];

// // --- モーター基本設定 ---
// #define MOTOR_ID  0x7F
// #define MASTER_ID 0x00

// // --- CyberGear 通信モード (拡張ID上位5bit) ---
// #define MODE_MOTOR_ENABLE     0x03   
// #define MODE_MOTOR_STOP       0x04
// #define MODE_SET_ZERO_POS     0x06   
// #define MODE_PARAM_WRITE      0x12   

// // --- CyberGear 内部レジスタインデックス ---
// #define INDEX_RUN_MODE        0x7005 // 1:位置, 2:速度, 3:電流
// #define INDEX_TARGET_CUR      0x7006 // 目標電流 (float, A)

// // --- モード定義 ---
// #define CONTROL_MODE_CUR      3

// // --- 制御・状態管理 ---
// float target_current  = 0.0f; 
// int current_mode = CONTROL_MODE_CUR; 
// bool is_running = false; // タッチ状態管理用

// // 読み取り値格納用
// float spd = 0.0;
// float trq = 0.0;
// unsigned long pre_time_10 = 0;
// unsigned long pre_time_100 = 0;
// uint32_t start_time = 0;

// // 関数プロトタイプ
// void init_can();
// void stop_motor(uint8_t motor_id);
// void enable_motor(uint8_t motor_id);
// void set_zero_position(uint8_t motor_id);
// void send_parameter_write(uint8_t motor_id, uint16_t param_index, float value, uint8_t is_byte = 0);
// void control_current(uint8_t motor_id, float current);
// void change_mode(uint8_t motor_id, uint8_t mode);
// void sendData();

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

//     // PSRAMからメモリ確保
//     dataLog = (SensorData*)ps_malloc(MAX_SAMPLES * sizeof(SensorData));

//     if (dataLog == nullptr) {
//         M5.Display.fillScreen(RED);
//         M5.Display.setCursor(0, 0);
//         M5.Display.println("Memory Error");
//     } else {
//         M5.Display.println("Memory OK");
//         M5.Display.println("Touch Screen to Start");
//     }

//     init_can();
//     delay(500);

//     stop_motor(MOTOR_ID);
//     delay(100);
    
//     change_mode(MOTOR_ID, current_mode);
//     delay(100);

//     set_zero_position(MOTOR_ID);
//     delay(100);

//     enable_motor(MOTOR_ID);
//     delay(100);
    
//     Serial.println("Setup Completed.");
// }

// float pre_time = 0.0;

// void loop() {
//     M5.update();    

//     // --- タッチ操作による回転制御 (状態管理) ---
//     bool is_touched = (M5.Touch.getCount() > 0);

//     if (is_touched && !isMeasuring && dataLog != nullptr) {
//         M5.Display.fillScreen(BLACK);
//         sampleCount = 0;
//         is_running = true;
//         isMeasuring = true;
//         start_time = micros();
//     } 

//     // --- 計測とモータ出力値の計算---
//     if(isMeasuring == true){
//         // モータ制御
//         uint32_t now = micros();
//         if (now - pre_time > 1000){
//             float t = (now - start_time) / 1000000.0;
//             float f = 4.0;
//             float A = 2.0;
//             float torque = A * (sin(2*M_PI*f*t) + 0.6*sin(2*M_PI*3.4*f*t) + 0.3*sin(2*M_PI*7.4*f*t));
//             // float torque = A * sin(2*M_PI*f*t);
//             target_current = torque / 0.615;

//             // dataLogにセンサーデータを格納
//             dataLog[sampleCount].time_us = micros();
//             dataLog[sampleCount].spd = spd;
//             dataLog[sampleCount].torque = torque;
//             sampleCount++;
//             pre_time = now;
//         }

//         // 規定数に達したら計測終了
//         if (sampleCount >= MAX_SAMPLES && sending == true) {
//             isMeasuring = false;
//             target_current = 0.0;
//             control_current(MOTOR_ID, target_current);
        
//             // 計測が終わってから初めて画面を使用する
//             M5.Display.fillScreen(BLUE);
//             M5.Display.setCursor(0, 0);
//             M5.Display.println("Sampling Finished!");
//             M5.Display.println("Sending to PC...");
        
//             sendData();
//             sending = false;
//         }
//     }

//     // --- CAN受信処理  ---
//     while (CAN0.checkReceive() == CAN_MSGAVAIL) {
//         unsigned long rxId;
        
//         CAN0.readMsgBuf(&rxId, &len, buf); 

//         uint32_t cleanId = rxId & 0x1FFFFFFF;
//         uint8_t mode = (cleanId >> 24) & 0x1F;

//         if (mode == 0x02) { 
//             uint16_t spd_raw = (buf[2] << 8) | buf[3];
//             uint16_t trq_raw = (buf[4] << 8) | buf[5];

//             spd = uint_to_float(spd_raw, -30.0f, 30.0f, 16); 
//             trq = uint_to_float(trq_raw, -12.0f, 12.0f, 16);
//         }
//     }

//     control_current(MOTOR_ID, target_current);

// }


// // 関数群 ---
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

// void control_current(uint8_t motor_id, float current) {
//     send_parameter_write(motor_id, INDEX_TARGET_CUR, current, 0);
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

// void sendData() {
//   Serial.println("DATA_START");
//   Serial.println("time_us,speed,torque");

//   for (uint32_t i = 0; i < MAX_SAMPLES; i++) {
//     float time_sec = (float)(dataLog[i].time_us - start_time) / 1000000.0f;
//     Serial.print(time_sec, 6);
//     Serial.print(",");
//     Serial.print(dataLog[i].spd, 4);
//     Serial.print(",");
//     Serial.println(dataLog[i].torque, 4);
//   }
//   Serial.println("DATA_END");
  
//   M5.Display.fillScreen(GREEN);
//   M5.Display.setCursor(0, 0);
//   M5.Display.println("All Done!");
//   M5.Display.println("Data sent to PC");
// }