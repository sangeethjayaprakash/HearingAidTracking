
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>


Adafruit_MLX90614 mlx = Adafruit_MLX90614();
#define TOUCH 4
#define LED 10
#define NORMAL_BODY_TEMP 38

boolean touchState;

void setup() {
  Serial.begin(9600);
  mlx.begin();
    pinMode(TOUCH,INPUT);
}

void loop() {

  Serial.println("Temperature from MLX90614:");
  Serial.print("Ambient:      ");
  Serial.print(mlx.readAmbientTempC());
  Serial.println(" °C");
  Serial.print("Contactless: ");
  Serial.print(mlx.readObjectTempC());
  Serial.println(" °C");
  Serial.println();
   
  touchState = digitalRead(TOUCH);

  if(touchState == HIGH) {
    digitalWrite(LED, HIGH);//write statements to execute when the sw is high
  }
  else {
    digitalWrite(LED,LOW);//write statements to execute when the sw is low
  }
}
