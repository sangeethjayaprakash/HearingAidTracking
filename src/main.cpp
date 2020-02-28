
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <SoftwareSerial.h>

#define TOUCH 4
#define LED 10
#define NORMAL_BODY_TEMP 38
#define BT_TX 2
#define BT_RX 3

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
SoftwareSerial bt(BT_TX, BT_RX);

boolean touchState = LOW;
float temperature = 0;
char c = '\0';
String alarm_status = "\0";

void setup() {
  Serial.begin(115200);
  bt.begin(9600);
  mlx.begin();
  pinMode(TOUCH,INPUT);
}

// Tracking function
void trackUsage(boolean touchState, float temperature) {
  if (touchState){
    if (temperature > NORMAL_BODY_TEMP){
      Serial.println("Device worn");
      bt.println("ON");
    }
  }
  else{
    Serial.println("Device taken off");
    bt.println("OFF");
  }
}

// Blink alarm function
void blinkAlarm() {
  digitalWrite(LED, HIGH);
}

void loop() {
  while (bt.available()){
    c = bt.read();
    if (c == '#') {
      break;
    }
    alarm_status += c;
  }
  alarm_status.trim();
  if (alarm_status == "TRUE") {
    blinkAlarm();
  }
  touchState = digitalRead(TOUCH);
  temperature = (float)mlx.readObjectTempC();
  trackUsage(touchState, temperature);
}
