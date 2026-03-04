// #include <Wire.h>
// #include <M5Unified.h>

// // ★ 設定対象のI2Cアドレス（デフォルトは0x64）
// #define TARGET_I2C_ADDR 0x65

// // レジスタアドレス定義
// #define REG_P 0xA0  // Speed Max Current Setting (章3.2)
// #define REG_I 0xA4  // Speed Max Current Setting (章3.2)
// #define REG_D 0xA8  // Speed Max Current Setting (章3.2)
// #define REG_SAVE_FLASH        0xF0  // Save Flash (章6.4)

// // 設定値
// const int32_t GAIN_VALUE = 6500000;

// void setup() {
//   // M5UnifiedとWireの初期化
//   auto cfg = M5.config();
//   M5.begin(cfg);
//   Wire.begin();
//   Serial.begin(115200);

//   Serial.println("--- [開始] Max Current設定変更と保存 ---");
//   delay(1000);

//   // ----------------------------------------------------
//   // 1. Speed Max Current (最大電流) を書き込む
//   // ----------------------------------------------------
//   Serial.print("ステップ1: Speed Max Current を ");
//   Serial.print(GAIN_VALUE);
//   Serial.println(" に設定します...");

//   Wire.beginTransmission(TARGET_I2C_ADDR);
//   Wire.write(REG_P); // レジスタ 0x50

//   // 4バイトのデータをリトルエンディアン順（下位バイトから）で送信
//   // Byte0 -> Byte1 -> Byte2 -> Byte3
//   Wire.write((uint8_t)(GAIN_VALUE & 0xFF));         // Byte 0
//   Wire.write((uint8_t)((GAIN_VALUE >> 8) & 0xFF));  // Byte 1
//   Wire.write((uint8_t)((GAIN_VALUE >> 16) & 0xFF)); // Byte 2
//   Wire.write((uint8_t)((GAIN_VALUE >> 24) & 0xFF)); // Byte 3

//   uint8_t error = Wire.endTransmission();

//   if (error == 0) {
//     Serial.println(">> 設定値を送信しました。");
//   } else {
//     Serial.print(">> 送信エラー: ");
//     Serial.println(error);
//     while (1); // エラーなら停止
//   }

//   delay(200); // 内部処理のために少し待機

//   // ----------------------------------------------------
//   // 2. 設定をフラッシュメモリに保存する
//   // ----------------------------------------------------
//   Serial.println("ステップ2: 設定をフラッシュメモリに保存します...");

//   Wire.beginTransmission(TARGET_I2C_ADDR);
//   Wire.write(REG_SAVE_FLASH); // レジスタ 0xF0
//   Wire.write(0x01);           // 1: Saveコマンド
//   Wire.endTransmission();

//   Serial.println(">> 保存命令を送信しました。");
  
//   // フラッシュへの書き込みは時間がかかるため、長めに待機
//   delay(1000); 

//   Serial.println("--- 設定変更と保存処理 完了 ---");
//   Serial.println("設定を反映させるため、ローラーユニットの電源を再投入してください。");
// }

// void loop() {
//   // この処理は一度だけで良いのでloopは空です
// }