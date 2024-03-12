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

// ps4コントローラーの指令値
float command_X = 0.f;
float command_Y = 0.f;
float command_steer = 0.0;
float command_speed = 0.0;

// ステア角フィードバック
float steer_angle = 0.0;
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
int top_motor_duty_out = 0.0;
int bottom_motor_duty_out = 0.0;

void setup()
{
  Serial.begin(57600);
  Wire.begin();
  qei_setup_x4(PCNT_UNIT_0, top_encoder_A, top_encoder_B);
  qei_setup_x4(PCNT_UNIT_1, bottom_encoder_A, bottom_encoder_B);
  int ledcfreq = 20000;   // Hz
  int ledcResolution = 8; // bit
  ledcSetup(ch_top1, ledcfreq, ledcResolution);
  ledcSetup(ch_top2, ledcfreq, ledcResolution);
  ledcSetup(ch_bottom1, ledcfreq, ledcResolution);
  ledcSetup(ch_bottom1, ledcfreq, ledcResolution);
}

void loop()
{
  if (PS4.isConnected()){
    command_X = (float)PS4.LStickX() * 1.f;
    command_Y = (float)PS4.LStickY() * 1.f;
    // ステア角指令値
    command_steer = atan2(command_Y, command_X);
    // スピード指令値
    command_speed = sqrt(command_Y * command_Y + command_X * command_X);
  }

  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  Wire.endTransmission(false);

  Wire.requestFrom(0x36, 2);

  while (Wire.available()){
    byte angle_h = Wire.read();
    byte angle_l = Wire.read();
    unsigned int angle = (0x0F & angle_h) << 8 | angle_l;
    steer_angle = (float)angle * 360.f / 4096.f;
    Serial.print(angle);
    // Serial.print(angle,HEX);
    Serial.print(" ");
  }
  delay(1);

  Wire.beginTransmission(0x36);
  Wire.write(0x0B);
  Wire.endTransmission(false);

  Wire.requestFrom(0x36, 1);

  while (Wire.available()){
    byte state = Wire.read();

    Serial.println(state, BIN);
  }

  /* ステア角フィードバック*/
  err_steer = steer_angle - command_steer;
  steer_calcP = err_steer;
  steer_calcI = steer_calcI + (err_steer + pre_err_steer) * ((float)control_period * 1.0E-6f) * 0.5;
  steer_calcD = (err_steer - pre_err_steer) * control_freq;
  steer_calcPID = steer_gainP * steer_calcP + steer_gainI * steer_calcI + steer_gainD * steer_calcD;
  pre_err_steer = err_steer;
  //モーターの出力に変換
  if(err_steer>=5.0){
    command_speed=0.0;
  }
  top_motor_duty_out=command_speed*0.25+command_steer;
  bottom_motor_duty_out=command_speed*0.25-command_steer;

  if(top_motor_duty_out>255){top_motor_duty_out=200;}
  if(top_motor_duty_out<-255){top_motor_duty_out=-200;}
  if(bottom_motor_duty_out>255){bottom_motor_duty_out=200;}
  if(bottom_motor_duty_out<-255){bottom_motor_duty_out=-200;}
  if(top_motor_duty_out>=0.0){
    ledcWrite(ch_top1,abs(top_motor_duty_out));
    ledcWrite(ch_top2,0);
  }else{
    ledcWrite(ch_top1,0.0);
    ledcWrite(ch_top2,abs(top_motor_duty_out));
  }
  if(bottom_motor_duty_out>=0.0){
    ledcWrite(ch_bottom1,abs(bottom_motor_duty_out));
    ledcWrite(ch_bottom2,0.0);
  }else{
    ledcWrite(ch_bottom1,0);
    ledcWrite(ch_bottom2,abs(bottom_motor_duty_out));
  }
  /*制御周期安定化*/
  control_count++;
  interval = micros() - preinterval;
  while (interval < control_period)
  {
    interval = micros() - preinterval;
  }
  preinterval = micros();
}