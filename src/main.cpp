#include <Arduino.h>
#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <PS4Controller.h>
#include <esp_bt_defs.h>
#include <esp_bt_main.h>

const int control_period = 1000; // us
const float control_freq = 1.0f / float(control_period); //Hz
int control_count=0;
int interval=0;
int preinterval=0;


void setup()
{
}

void loop()
{
  control_count++;
  interval=micros()-preinterval;
  while(interval<control_period){
    interval=micros()-preinterval;
  }
  preinterval=micros();
}