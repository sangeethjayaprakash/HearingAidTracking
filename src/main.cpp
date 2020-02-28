#include <Wire.h>
#include <Adafruit_MLX90614.h>
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
#define touchSw 4
#define led 10

void setup() {
  Serial.begin(9600);
  mlx.begin();
    pinMode(touchSw,INPUT);
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
//  delay(500);
   boolean touchState;
  touchState = digitalRead(touchSw);

  if(touchState == HIGH)
  {
    digitalWrite(led, HIGH);//write statements to execute when the sw is high
  }
  else
  {
    digitalWrite(led,LOW);//write statements to execute when the sw is low
  }
 //   delay(500);
//  Serial.println(touchState);
}
