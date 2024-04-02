#include <Arduino.h>
#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <PS4Controller.h>
#include <esp_bt_defs.h>
#include <esp_bt_main.h>
#include "qei.hpp"

// 制御周期
const int control_period = 1000;                         // us
const float control_freq = 1.0f / float(control_period); // Hz
int control_count = 0;
int interval = 0;
int preinterval = 0;

// モーター
const int top_motor_In1 = 25;
const int top_motor_In2 = 26;
const int bottom_motor_In1 = 27;
const int bottom_motor_In2 = 32;
const int ch_top1 = 0;
const int ch_top2 = 1;
const int ch_bottom1 = 2;
const int ch_bottom2 = 3;

// 磁気エンコーダ
const int steer_encoder_SCL = 22;
const int steer_encoder_SDA = 21;

// ローターリーエンコーダ
const int top_encoder_A = 16;
const int top_encoder_B = 17;
const int bottom_encoder_A = 18;
const int bottom_encoder_B = 19;

// ゲイン調整用ポテンショピン
const int pot1 = 34;
const int pot2 = 35;
const int pot3 = 36;

// 状態表示LED
const int statusLED1 = 13;
const int statusLED2 = 14;
const int ch_led1 = 5;
const int ch_led2 = 6;

// スイッチ
const int sw_pin=13;

// ps4コントローラーの指令値
float command_X = 0.f;
float command_Y = 0.f;
float command_steer = 0.0; // rad
float command_speed = 0.0;

// ステア角フィードバック
float steer_angle = 0.0;     // deg 角度
float steer_pre_angle = 0.0; // deg前周期のステア角
float steer_angle_ref = 0.0; // deg 目標値
float steer_gainP = 0.0;
float steer_gainI = 0.0;
float steer_gainD = 0.0;
float err_steer = 0.0;
float pre_err_steer = 0.0;
float steer_calcP = 0.0;
float steer_calcI = 0.0;
float steer_calcD = 0.0;
float steer_calcPID = 0.0;
// スピードフィードバック

// モーターデューティ
float top_motor_duty_out = 0.0;
float bottom_motor_duty_out = 0.0;

void setup()
{
  Serial.begin(57600);
  PS4.begin("0C:B8:15:C1:3C:66");
  Wire.begin();
  qei_setup_x4(PCNT_UNIT_0, top_encoder_A, top_encoder_B);
  qei_setup_x4(PCNT_UNIT_1, bottom_encoder_A, bottom_encoder_B);
  int ledcfreq = 20000;   // Hz
  int ledcResolution = 8; // bit
  ledcAttachPin(top_motor_In1, ch_top1);
  ledcAttachPin(top_motor_In2, ch_top2);
  ledcAttachPin(bottom_motor_In1, ch_bottom1);
  ledcAttachPin(bottom_motor_In1, ch_bottom2);
  ledcSetup(ch_top1, ledcfreq, ledcResolution);
  ledcSetup(ch_top2, ledcfreq, ledcResolution);
  ledcSetup(ch_bottom1, ledcfreq, ledcResolution);
  ledcSetup(ch_bottom1, ledcfreq, ledcResolution);
  // ゲイン調整用ポテンショ
  pinMode(pot1, INPUT);
  pinMode(pot2, INPUT);
  pinMode(pot3, INPUT);
  // 状態表示LED
  ledcAttachPin(statusLED1, ch_led1);
  ledcSetup(ch_led1, ledcfreq, ledcResolution);
  // pinMode(statusLED1, OUTPUT);
  // pinMode(statusLED2, OUTPUT);
}

void loop()
{
  /*PS4コントローラー*/
  if (PS4.isConnected())
  {
    // スティックの値を-1~1に変換
    command_X = (float)PS4.LStickX() / 128.0f;
    command_Y = (float)PS4.LStickY() / 128.0f;
    // スティックの値を-1~1に変換、その後速度指令値の上限1になるようルート2でわる
    // command_X = (float)PS4.LStickX() * 0.005524271728f; //(1/128)*(1/sqrt(2))
    // command_Y = (float)PS4.LStickY() * 0.005524271728f;
    // ステア角指令値
    command_steer = (float)atan2(command_Y, command_X);
    // 速度指令値
    command_speed = sqrt(command_Y * command_Y + command_X * command_X);
    // LED消灯
    ledcWrite(statusLED1, 0);
  }
  else
  {
    // LED点灯
    ledcWrite(statusLED1, 255);
    // 指令値を0に
    command_steer = 0.0f;
    command_speed = 0.0f;
  }

  if (control_count % 50)
  {
    // 指令値ベクトル
    Serial.printf("%f,%f/r/n", command_X, command_Y);
    // ステア指令値と速度指令値
    Serial.printf("%f,%f/r/n", command_steer, command_speed);
  }

  /*ステアエンコーダー*/
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  Wire.endTransmission(false);

  Wire.requestFrom(0x36, 2);

  while (Wire.available())
  {
    byte angle_h = Wire.read();
    byte angle_l = Wire.read();
    unsigned int angle = (0x0F & angle_h) << 8 | angle_l;
    steer_angle = (float)angle * 2 * M_PI / 4096.f;
    Serial.println(angle);
    // Serial.print(angle,HEX);
    // Serial.print(" ");
  }
  // delay(1);

  // Wire.beginTransmission(0x36);
  // Wire.write(0x0B);
  // Wire.endTransmission(false);

  // Wire.requestFrom(0x36, 1);

  // while (Wire.available())
  // {
  //   byte state = Wire.read();

  //   Serial.println(state, BIN);
  // }

  /* ステア角フィードバック*/
  // ゲイン調整
  steer_gainP = 0.0f + (float)(analogRead(pot1) - 2048) * 0.00024414f * 0.0001f;
  steer_gainI = 0.0f + (float)(analogRead(pot2) - 2048) * 0.00024414f * 0.0f;
  steer_gainD = 0.0f + (float)(analogRead(pot3) - 2048) * 0.00024414f * 0.0f;
  // ステア角度変化目標値
  // steer_angle_ref = command_steer - steer_angle;
  err_steer = command_steer - steer_angle;
  // err_steer = steer_angle - steer_angle_ref;
  // steer_vel=err_steer/(control_period*10.);
  // フィードバック項の計算
  steer_calcP = err_steer;
  steer_calcI = steer_calcI + (err_steer + pre_err_steer) * ((float)control_period * 1.0E-6f) * 0.5;
  steer_calcD = (err_steer - pre_err_steer) * control_freq;
  steer_calcPID = steer_gainP * steer_calcP + steer_gainI * steer_calcI + steer_gainD * steer_calcD;

  pre_err_steer = err_steer;
  steer_pre_angle = steer_angle;

  if (steer_calcPID > 1.0)
  {
    steer_calcPID = 1.0;
  }
  if (steer_calcPID < -1.0)
  {
    steer_calcPID = -1.0;
  }

  // モーターの出力に変換
  if (err_steer >= 0.0628)
  {
    command_speed = 0.0;
  }
  top_motor_duty_out = command_speed * 0.25 + steer_calcPID;
  bottom_motor_duty_out = command_speed * 0.25 - steer_calcPID;

  if (control_count % 50)
  {
    Serial.printf("%f,%f,%f/r/n", steer_gainP, steer_gainD, steer_gainD);
    Serial.printf("%f/r/n", top_motor_duty_out, bottom_motor_duty_out);
    Serial.printf("%f/r/n", err_steer);
    Serial.printf("%f,%f/r/n", top_motor_duty_out, bottom_motor_duty_out);
  }

  if (top_motor_duty_out > 255)
  {
    top_motor_duty_out = 255;
  }
  if (top_motor_duty_out < -255)
  {
    top_motor_duty_out = -255;
  }
  if (bottom_motor_duty_out > 255)
  {
    bottom_motor_duty_out = 255;
  }
  if (bottom_motor_duty_out < -255)
  {
    bottom_motor_duty_out = -255;
  }

  if(digitalRead(sw_pin)==0)
  {
    top_motor_duty_out=0.0f;
    bottom_motor_duty_out=0.0f;
  }

  if (top_motor_duty_out >= 0.0)
  {
    ledcWrite(ch_top1, abs((int)top_motor_duty_out));
    ledcWrite(ch_top2, 0);
  }
  else
  {
    ledcWrite(ch_top1, 0);
    ledcWrite(ch_top2, abs((int)top_motor_duty_out));
  }
  if (bottom_motor_duty_out >= 0.0)
  {
    ledcWrite(ch_bottom1, abs((int)bottom_motor_duty_out));
    ledcWrite(ch_bottom2, 0);
  }
  else
  {
    ledcWrite(ch_bottom1, 0);
    ledcWrite(ch_bottom2, abs((int)bottom_motor_duty_out));
  }

  steer_pre_angle = steer_angle;

  /*制御周期安定化*/
  control_count++;
  interval = micros() - preinterval;
  while (interval < control_period)
  {
    interval = micros() - preinterval;
  }
  preinterval = micros();
}