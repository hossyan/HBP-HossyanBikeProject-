// #include <M5Unified.h>

// // 20,000サンプル (約240KB)
// const uint32_t MAX_SAMPLES = 20000;

// struct SensorData {
//   uint32_t time_us; 
//   float gyro;
//   float torque;
// };

// void sendData();

// SensorData* dataLog = nullptr;
// uint32_t sampleCount = 0;
// bool isMeasuring = false;
// uint32_t startTime = 0;
// bool sending = true;

// void setup() {
//   auto cfg = M5.config();
//   M5.begin(cfg);
  
//   M5.Display.setTextSize(2);
//   M5.Display.println("High-Speed Logger");

//   // PSRAMからメモリ確保
//   dataLog = (SensorData*)ps_malloc(MAX_SAMPLES * sizeof(SensorData));

//   if (dataLog == nullptr) {
//     M5.Display.fillScreen(RED);
//     M5.Display.setCursor(0, 0);
//     M5.Display.println("Memory Error");
//   } else {
//     M5.Display.println("Memory OK");
//     M5.Display.println("Touch Screen to Start");
//   }

//   Serial.begin(115200);
// }

// void loop() {
//   M5.update();

//   // 計測開始：タッチされた瞬間にフラグを立てる
//   if (M5.Touch.getCount() > 0 && !isMeasuring && dataLog != nullptr) {
//     // 計測直前に画面をクリア（計測中の負荷を減らすため）
//     M5.Display.fillScreen(BLACK);
    
//     sampleCount = 0;
//     startTime = micros();
//     isMeasuring = true;
//   }

//   // サンプリングループ
//   if (isMeasuring) {
//     // 経過時間、角速度、トルクを最速で取得
//     dataLog[sampleCount].time_us = micros();
    
//     // センサーデータの取得
//     // ※ M5.Imu.update() は内部でI2C通信を行うため、ここが実質的な速度上限になります
//     M5.Imu.update(); 
//     float gx, gy, gz;
//     M5.Imu.getGyro(&gx, &gy, &gz);
    
//     dataLog[sampleCount].gyro = gz;
//     dataLog[sampleCount].torque = 0.0; // ここにトルク取得のコード

//     sampleCount++;

//     // 規定数に達したら即座に計測終了
//     if (sampleCount >= MAX_SAMPLES && sending == true) {
//       isMeasuring = false;
      
//       // 計測が終わってから初めて画面を使用する
//       M5.Display.fillScreen(BLUE);
//       M5.Display.setCursor(0, 0);
//       M5.Display.println("Sampling Finished!");
//       M5.Display.println("Sending to PC...");
      
//       sendData();
//       sending = false;
//     }
//   }
// }

// void sendData() {
//   Serial.println("DATA_START");
//   Serial.println("time_us,gyro,torque");

//   for (uint32_t i = 0; i < MAX_SAMPLES; i++) {
//     float time_sec = (float)(dataLog[i].time_us - startTime) / 1000000.0f;
//     Serial.print(time_sec, 6);
//     Serial.print(",");
//     Serial.print(dataLog[i].gyro, 4);
//     Serial.print(",");
//     Serial.println(dataLog[i].torque, 4);
//   }
//   Serial.println("DATA_END");
  
//   M5.Display.fillScreen(GREEN);
//   M5.Display.setCursor(0, 0);
//   M5.Display.println("All Done!");
//   M5.Display.println("Data sent to PC");
// }