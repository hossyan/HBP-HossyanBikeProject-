#include <Wire.h>
#include <M5Unified.h>

#define BUTTON_X 15
#define BUTTON_Y 15
#define BUTTON_W 70
#define BUTTON_H 50
#define BUTTON_INC_X 110
#define BUTTON_INC_Y 90

#define EMERGENCY_BUTTON_X 50
#define EMERGENCY_BUTTON_Y 200
#define EMERGENCY_CIRCLE 35

#define OUTPUT_X 130
#define OUTPUT_Y 190

bool buttonPressed = false;
bool emergency_button = false;

float ax,ay,az;
float gx,gy,gz;
float roll,pitch,yaw;

#define left_motor_id 0x64
#define right_motor_id 0x65

// レジスタアドレス
#define REG_ENABLE         0x00      // モーター有効化レジスタ
#define REG_MODE           0x01      // 動作モード設定レジスタ
#define REG_CURRENT        0xB0      // 電流設定レジスタ
#define REG_SPEED        0x40      // 速度設定レジスタ
#define Speed_Readback     0x60      // エンコーダ(rpm)読み取りレジスタ
#define Position_Readback  0x90      // エンコーダ(位置)読み取りレジスタ
#define Speed_Readback     0x60      // エンコーダ(rpm)読み取りレジスタ
#define Current_Readback  0xC0      // 電流値読み取りレジスタ

#define current_mode    0x03      //動作モード：電流制御
#define speed_mode    0x01      //動作モード：電流制御

float inc_kp = 0.001;
float inc_ki = 0.0000001; 
float inc_kd = 0.00001;

float kp = 0.097;
float ki = 0.0000003;
float kd = 0.00201;

float pid_time = 0.0;
float pid_time_pre = 0.0;
int current_max = 460;
int speed_max = 21000000;
float target_angle = 0.0;
float integral = 0.0;
float pre_error = 0.0;
float pre_time;

float angle = 0.0;
float angle_acc = 0.0;

float output = 0;

// ボタン描画
void button_draw() {
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 2; j++){
            int x = BUTTON_X + BUTTON_INC_X * i;
            int y = BUTTON_Y + BUTTON_INC_Y * j;
            M5.Display.fillRoundRect(x, y, BUTTON_W, BUTTON_H, 10, TFT_DARKGRAY);

            int triY1 = y + (j==0 ? 15 : BUTTON_H - 15);
            int triY2 = y + (j==0 ? BUTTON_H - 18 : 18);
            M5.Display.fillTriangle(x + BUTTON_W / 2 , triY1, x + 20, triY2, x + BUTTON_W - 20, triY2, TFT_BLACK);
        }
    }
}

// PID値表示
void pid_draw() {
    float pid_vals[3] = {kp, ki, kd};
    const char* pid_names[3] = {"kp", "ki", "kd"};

    for(int i = 0; i < 3; i++) {
        int x_val = BUTTON_X + BUTTON_INC_X * i - (i==1 ? 10 : 0);
        int y_val = BUTTON_Y + BUTTON_H + (BUTTON_INC_Y - BUTTON_H) / 2;
        int x_name = BUTTON_X + BUTTON_INC_X * i + BUTTON_W / 2 - 10;
        int y_name = y_val -15;

        M5.Display.setCursor(x_val, y_val);
        M5.Display.printf(i==0 ? "%.3f" : i==1 ? "%.7f" : "%.5f", pid_vals[i]);

        M5.Display.setCursor(x_name, y_name);
        M5.Display.print(pid_names[i]);
    }

    M5.Display.setCursor(OUTPUT_X, OUTPUT_Y);
    M5.Display.printf("Output : %.0f", output * current_max);
}

// 緊急停止
void emergency_button_draw(){
    static bool old_emergency_button = !emergency_button;  
    if (emergency_button != old_emergency_button) {
        if (emergency_button == true) {
            M5.Display.setTextColor(TFT_WHITE, TFT_GREEN);
            M5.Display.fillCircle(EMERGENCY_BUTTON_X, EMERGENCY_BUTTON_Y, EMERGENCY_CIRCLE, TFT_GREEN);
            M5.Display.drawCenterString("ON", EMERGENCY_BUTTON_X,EMERGENCY_BUTTON_Y - 5);
        } else {
            M5.Display.setTextColor(TFT_WHITE, TFT_RED);
            M5.Display.fillCircle(EMERGENCY_BUTTON_X, EMERGENCY_BUTTON_Y,EMERGENCY_CIRCLE, TFT_RED);
            M5.Display.drawCenterString("OFF", EMERGENCY_BUTTON_X,EMERGENCY_BUTTON_Y - 5);
        }
        old_emergency_button = emergency_button;  
        M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    }
}

// 画面タッチ認識
void display_touch() {
    if (M5.Touch.getCount() > 0) {
        auto t = M5.Touch.getDetail();
        if(!buttonPressed) {
            buttonPressed = true;
            for(int i = 0; i < 3; i++){
                for(int j = 0; j < 2; j++){
                    int x = BUTTON_X + BUTTON_INC_X * i;
                    int y = BUTTON_Y + BUTTON_INC_Y * j;
                    if(t.x > x && t.x < x + BUTTON_W && t.y > y && t.y < y + BUTTON_H){
                        float inc = (i==0 ? inc_kp : i==1 ? inc_ki : inc_kd);
                        if(j == 0) { if(i==0) kp += inc; else if(i==1) ki += inc; else if(i==2) kd += inc;}
                        else { if(i==0) kp -= inc; else if(i==1) ki -= inc; else if(i==2) kd -= inc; }
                    }
                }
            }
            if(t.x > EMERGENCY_BUTTON_X - EMERGENCY_CIRCLE && t.x < EMERGENCY_BUTTON_X + EMERGENCY_CIRCLE &&
               t.y > EMERGENCY_BUTTON_Y - EMERGENCY_CIRCLE && t.y < EMERGENCY_BUTTON_Y + EMERGENCY_CIRCLE){
                emergency_button = !emergency_button;
            }
        }
    } else {
        buttonPressed = false;
    }
}

// roller485のセットアップ
void set_motor_enable(uint8_t address, bool enable) {
    Wire.beginTransmission(address);
    Wire.write(REG_ENABLE);
    Wire.write(enable ? 0x01 : 0x00);
    Wire.endTransmission();
}

// roller485の動作モード設定
void set_control_mode(uint8_t address, uint8_t mode) {
    Wire.beginTransmission(address);
    Wire.write(REG_MODE);
    Wire.write(mode);
    Wire.endTransmission();
}

// 電流制御
void set_current(int8_t address, int32_t current) {
    int32_t current_value = current * 100;

    Wire.beginTransmission(address);
    Wire.write(REG_CURRENT);

    // 32ビットの値をリトルエンディアンで4バイトに分割して送信
    Wire.write(current_value & 0xFF);
    Wire.write((current_value >> 8) & 0xFF);
    Wire.write((current_value >> 16) & 0xFF);
    Wire.write((current_value >> 24) & 0xFF);

    Wire.endTransmission();
}

// 速度制御
void set_speed(int8_t address, int32_t speed) {
    int32_t speed_value = speed * 100;

    Wire.beginTransmission(address);
    Wire.write(REG_SPEED);

    // 32ビットの値をリトルエンディアンで4バイトに分割して送信
    Wire.write(speed_value & 0xFF);
    Wire.write((speed_value >> 8) & 0xFF);
    Wire.write((speed_value >> 16) & 0xFF);
    Wire.write((speed_value >> 24) & 0xFF);

    Wire.endTransmission();
}

// エンコーダ読み取り(rpm)
long read_speed(uint8_t address){
    Wire.beginTransmission(address);
    Wire.write(Speed_Readback);
    Wire.endTransmission();

    Wire.requestFrom(address, 4);

    if(Wire.available() >= 4){
        byte byte0 = Wire.read();
        byte byte1 = Wire.read();
        byte byte2 = Wire.read();
        byte byte3 = Wire.read();

        long value = (long)byte3 << 24 | (long)byte2 << 16 | (long)byte1 << 8 | (long)byte0;
        return value;
    }

    return 0;
}

// 電流値読み取り
long read_current(uint8_t address){
    Wire.beginTransmission(address);
    Wire.write(Current_Readback);
    Wire.endTransmission();

    Wire.requestFrom(address, 4);

    if(Wire.available() >= 4){
        byte byte0 = Wire.read();
        byte byte1 = Wire.read();
        byte byte2 = Wire.read();
        byte byte3 = Wire.read();

        long value = (long)byte3 << 24 | (long)byte2 << 16 | (long)byte1 << 8 | (long)byte0;
        return value;
    }

    return 0;
}

void setup() {
    auto cfg = M5.config();

    M5.begin(cfg);
    M5.Imu.begin();
    Serial.begin(115200);
    Wire.begin(); 

    M5.Display.setTextSize(2);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.fillScreen(TFT_BLACK);
    button_draw();

    Serial.println("--- Roller Motor Control Start ---");
    randomSeed(analogRead(0));

    set_motor_enable(left_motor_id, true);
    set_motor_enable(right_motor_id, true);

    set_control_mode(left_motor_id, current_mode);
    set_control_mode(right_motor_id, current_mode);

    pid_time_pre = micros();
    }

void loop() {
    M5.update();

    pid_draw();
    emergency_button_draw();
    display_touch();

    pid_time = micros();
    float dt = (pid_time - pid_time_pre) / 1000000.0;
    pid_time_pre = pid_time;

    M5.Imu.getAccel(&ax, &ay, &az);
    M5.Imu.getGyro(&gx, &gy, &gz);

    angle_acc = atan2(ay, az) * 180.0 / PI;
    angle = 0.98 * (angle + gx * dt) +0.02 * angle_acc;

    float error = target_angle - angle;
    integral += error * dt;
    float diriv = (error - pre_error) / dt;
    float randomValue = (random(-1000, 1001)) / 100000.0;
    // output = kp * error + ki * integral + kd * diriv + randomValue;
    output = kp * error + ki * integral + kd * diriv;
    pre_error = error;

    if(error > 45 || error < -45 || emergency_button == false){
        output = 0;
    }

    if(output >= 1.0){
        output = 1.0;
    }else if(output <= -1.0){
        output = -1.0;
    }
    set_current(left_motor_id, output * current_max);
    set_current(right_motor_id, -output * current_max);
    long left_current = read_current(left_motor_id);
    long left_speed = read_speed(left_motor_id);

    // Serial.print(">output:");
    // Serial.println(output);
    // Serial.print(">current:");
    // Serial.println(left_current);
    // Serial.print(">rad/s:");
    // Serial.println(left_speed);
    // Serial.print(">ax:");
    // Serial.println(ax);
    // Serial.print(">ay:");
    // Serial.println(ay);
    // Serial.print(">az:");
    // Serial.println(az);
    // Serial.print(">gx:");
    // Serial.println(gx);
    // Serial.print(">gy:");
    // Serial.println(gy);
    // Serial.print(">gz:");
    // Serial.println(gz);
    Serial.printf("%f,%f,%f,%f,%f,%f\n",ax,ay,az,gx,gy,gz);

}