#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#define TOUCH 4
#define LED 10
#define NORMAL_BODY_TEMP 38
#define BT_TX 2
#define BT_RX 3
#define ADDR 0x68
#define TH_ACCEL 200

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
SoftwareSerial bt(BT_TX, BT_RX);

boolean touchState = LOW;
float temperature = 0;
char c = '\0';
String alarm_status = "\0";
unsigned int data[6];

void setup() {
  Wire.begin();
  
  // Start I2C transmission
  Wire.beginTransmission(ADDR);
  // Select gyroscope configuration register
  Wire.write(0x1B);
  // Full scale range = 2000 dps
  Wire.write(0x18);
  // Stop I2C transmission
  Wire.endTransmission();

  // Start I2C transmission
  Wire.beginTransmission(ADDR);
  // Select accelerometer configuration register
  Wire.write(0x1C);
  // Full scale range = +/-16g
  Wire.write(0x18);
  // Stop I2C transmission
  Wire.endTransmission();

  // Start I2C transmission
  Wire.beginTransmission(ADDR);
  // Select power management register
  Wire.write(0x6B);
  // PLL with xGyro reference
  Wire.write(0x01);
  // Stop I2C transmission
  Wire.endTransmission();
  
  Serial.begin(115200);
  bt.begin(9600);
  mlx.begin();
  pinMode(TOUCH,INPUT);
}

// Tracking function
void trackUsage(boolean touchState, float temperature, int acceleration) {
  if (touchState == HIGH && temperature > NORMAL_BODY_TEMP && acceleration > TH_ACCEL) {
    Serial.println("Device worn");
    bt.println("ON");
  }
  else {
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
  
  // Start I2C transmission
  Wire.beginTransmission(ADDR);
  // Select data register
  Wire.write(0x43);
  // Stop I2C transmission
  Wire.endTransmission();

  // Request 6 bytes of data
  Wire.requestFrom(ADDR, 6);

  // Read 6 byte of data
  if (Wire.available() == 6)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
    data[4] = Wire.read();
    data[5] = Wire.read();
  }

  // Convert the data
  int xAccl = data[0] * 256 + data[1];
  int yAccl = data[2] * 256 + data[3];
  int zAccl = data[4] * 256 + data[5];
  int acceleration = sqrt(pow(xAccl, 2) + pow(yAccl, 2) + pow(zAccl, 2));
  
  touchState = digitalRead(TOUCH);
  temperature = (float)mlx.readObjectTempC();
  trackUsage(touchState, temperature, acceleration);
}
