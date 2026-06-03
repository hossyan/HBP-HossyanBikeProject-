// /**
//  * CyberGear CAN ID 変更プログラム
//  * Board  : M5Stack Core2
//  * Library: M5Unified, MCP_CAN (coryjfowler/MCP_CAN_lib)
//  *
//  * 接続 (動作確認済みコードに準拠):
//  *   MCP2515 CS  -> GPIO 27
//  *   MCP2515 INT -> GPIO 15
//  *   MCP2515 SCK -> GPIO 18
//  *   MCP2515 SI  -> GPIO 23
//  *   MCP2515 SO  -> GPIO 38
//  *
//  * 使い方:
//  *   1. CURRENT_CAN_ID に現在のモーターのCAN IDを設定
//  *   2. NEW_CAN_ID    に変更後のCAN IDを設定
//  *   3. 書き込んで起動し、画面の指示に従いBtnAを押す
//  *
//  * CyberGear 通信タイプ7 (CAN ID変更) フレーム構成:
//  *   拡張29bit CAN ID:
//  *     [28:24] 通信タイプ = 7
//  *     [23:16] 新しいCAN ID
//  *     [15: 8] マスターID
//  *     [ 7: 0] 現在のモーターID
//  *   データ: 8バイト全て 0x00
//  */

// #include <M5Unified.h>
// #include <mcp_can.h>
// #include <SPI.h>

// // ==============================
// //  設定 (必要に応じて変更)
// // ==============================
// static const uint8_t CURRENT_CAN_ID = 0x7F;  // 現在のモーターCAN ID (デフォルト: 0x7F)
// static const uint8_t NEW_CAN_ID     = 0x7E;  // 変更後のCAN ID (0x01 ~ 0x7F)
// static const uint8_t MASTER_ID      = 0x00;  // マスター(M5Stack)のCAN ID

// // MCP2515 ピン設定 (動作確認済みコードに準拠)
// #define CAN0_INT    15
// const int SPI_CS_PIN = 27;

// // ==============================
// //  CyberGear 通信タイプ定数
// // ==============================
// #define CMD_SET_CAN_ID  0x07  // 通信タイプ7: CAN ID変更

// MCP_CAN CAN0(SPI_CS_PIN);

// // -----------------------------------------------
// // CAN ID 変更コマンドを送信する
// // 動作確認済みコードのビット配置に準拠:
// //   ((commType << 24) | (newId << 16) | (MASTER_ID << 8) | currentId)
// // -----------------------------------------------
// bool sendChangeCanId(uint8_t currentId, uint8_t newId)
// {
//   uint32_t extId = ((uint32_t)CMD_SET_CAN_ID << 24) |
//                    ((uint32_t)newId           << 16) |
//                    ((uint32_t)MASTER_ID       <<  8) |
//                    ((uint32_t)currentId);

//   uint8_t data[8] = {0};
//   byte result = CAN0.sendMsgBuf(extId, 1, 8, data);
//   return (result == CAN_OK);
// }

// // -----------------------------------------------
// // 画面描画ヘルパー
// // -----------------------------------------------
// void drawScreen(const char* title, const char* msg, uint32_t color = TFT_WHITE)
// {
//   M5.Lcd.fillScreen(TFT_BLACK);
//   M5.Lcd.setTextColor(TFT_CYAN);
//   M5.Lcd.setTextSize(2);
//   M5.Lcd.setCursor(8, 10);
//   M5.Lcd.print(title);
//   M5.Lcd.drawLine(0, 35, 320, 35, TFT_DARKGREY);
//   M5.Lcd.setTextColor(color);
//   M5.Lcd.setTextSize(1);
//   M5.Lcd.setCursor(8, 50);
//   M5.Lcd.print(msg);
// }

// void drawStatus(bool canReady)
// {
//   char buf[256];
//   snprintf(buf, sizeof(buf),
//     "Current CAN ID : 0x%02X\n"
//     "New     CAN ID : 0x%02X\n"
//     "Master  CAN ID : 0x%02X\n\n"
//     "MCP2515 (CS=27): %s\n\n"
//     "----------------------------\n"
//     "[BtnA] Send change command\n"
//     "[BtnC] Restart",
//     CURRENT_CAN_ID, NEW_CAN_ID, MASTER_ID,
//     canReady ? "OK" : "INIT FAILED");

//   drawScreen("CyberGear ID Changer", buf, canReady ? TFT_GREEN : TFT_RED);

//   M5.Lcd.setTextColor(TFT_YELLOW);
//   M5.Lcd.setTextSize(1);
//   M5.Lcd.setCursor(20,  220); M5.Lcd.print("  [SEND]");
//   M5.Lcd.setCursor(220, 220); M5.Lcd.print("[RESTART]");
// }

// // ==============================
// //  setup
// // ==============================
// void setup()
// {
//   auto cfg = M5.config();
//   M5.begin(cfg);
//   Serial.begin(115200);
//   Serial.println("CyberGear ID Changer starting...");

//   bool canOk = false;
//   if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
//     CAN0.setMode(MCP_NORMAL);
//     canOk = true;
//     Serial.println("MCP2515 initialized OK");
//   } else {
//     Serial.println("MCP2515 init FAILED");
//   }

//   drawStatus(canOk);
// }

// // ==============================
// //  loop
// // ==============================
// void loop()
// {
//   M5.update();

//   // BtnA: CAN ID変更コマンド送信
//   if (M5.BtnA.wasPressed()) {
//     Serial.printf("Sending CAN ID change: 0x%02X -> 0x%02X\n", CURRENT_CAN_ID, NEW_CAN_ID);
//     drawScreen("Sending...", "Sending CAN ID change command...", TFT_YELLOW);
//     delay(200);

//     bool ok = sendChangeCanId(CURRENT_CAN_ID, NEW_CAN_ID);

//     if (ok) {
//       char msg[128];
//       snprintf(msg, sizeof(msg),
//         "SUCCESS!\n\n"
//         "0x%02X  -->  0x%02X\n\n"
//         "Motor ID changed.\n"
//         "Please power cycle\nthe motor to confirm.",
//         CURRENT_CAN_ID, NEW_CAN_ID);
//       drawScreen("Done!", msg, TFT_GREEN);
//       Serial.println("Command sent successfully.");
//     } else {
//       drawScreen("Error", "Failed to send CAN frame.\nCheck wiring / MCP2515.", TFT_RED);
//       Serial.println("CAN send failed.");
//     }

//     delay(3000);
//     drawStatus(true);
//   }

//   // BtnC: 再起動
//   if (M5.BtnC.wasPressed()) {
//     ESP.restart();
//   }

//   delay(20);
// }