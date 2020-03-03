#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <SoftwareSerial.h>


#define NORMAL_BODY_TEMP 25
#define TH_ACCEL 200

#define TOUCH 8

#define R_LED1 A0
#define G_LED1 A1
#define R_LED2 A2
#define G_LED2 A3

#define BT_RX 2
#define BT_TX 3

#define ADDR 0x68

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
SoftwareSerial bt(BT_RX, BT_TX);

boolean touchState = LOW;
float temperature = 0;
char c = '\0';
String alarm_status = "\0";
unsigned int data[6];

void setup() {
  pinMode(TOUCH, INPUT);
  pinMode(R_LED1, OUTPUT);
  pinMode(G_LED1, OUTPUT);
  pinMode(R_LED2, OUTPUT);
  pinMode(G_LED2, OUTPUT);
  Serial.begin(115200);
  bt.begin(9600);
  mlx.begin();
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
}

// Tracking function
// Add RGB Led codes here
void trackUsage(boolean touchState, float temperature, int acceleration) {
  if (touchState == HIGH && temperature >NORMAL_BODY_TEMP &&acceleration > TH_ACCEL) {
    Serial.println("Device worn");
    analogWrite(R_LED1, 0);
    analogWrite(R_LED2, 0);
    bt.println("ONN");
  }
  else {
    Serial.println("Device taken off");
    analogWrite(R_LED1, 1024);
    analogWrite(R_LED2, 1024);
    bt.println("OFF");
  }
}

// Blink alarm function
void blinkAlarm(boolean state) {
  if (state) {
     analogWrite(G_LED1, HIGH);
   analogWrite(G_LED2, HIGH);
    Serial.println("TRU");
  }
  else {
     analogWrite(G_LED1, LOW);
     analogWrite(G_LED2, LOW);
   Serial.println("FLS");
  }
}

void loop() {
  if (bt.available()) {
    alarm_status = "\0";
    while (bt.available()){
      c = bt.read();
      if (c == '#') {
        break;
      }
      alarm_status += c;
    }
    alarm_status.trim();
    Serial.println(alarm_status);
    
    if (alarm_status == "TRU") {
      blinkAlarm(true);
    }
    
//    if (alarm_status == "FLS") {
//      blinkAlarm(false);
//    }
  }
   touchState = digitalRead(TOUCH);
  temperature = (float)mlx.readObjectTempC();
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
  
 Serial.println(acceleration);   
 Serial.println(touchState);
 Serial.println(temperature); 
  trackUsage(touchState, temperature, acceleration);
  delay(2000);
}
