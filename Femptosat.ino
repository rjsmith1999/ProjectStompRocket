
#include <SPI.h>
#include <RFM69.h>

#include "quaternionFilters.h"
#include "MPU9250.h"

#define LED_GREEN 3
#define LED_ORANGE 4

MPU9250 myIMU;
RFM69 radio;

#define SerialDebug true
#define ARHS true

void setup()
{
  Serial.begin(115200);
  
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_ORANGE, OUTPUT);

  digitalWrite(LED_ORANGE, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  
  setup_IMU();
  digitalWrite(LED_GREEN, LOW);
  
  setup_radio();
  digitalWrite(LED_ORANGE, LOW);  
}

void loop() {
  read_IMU();

  if (!ARHS) {
    radio_send();
  } else if (myIMU.delt_t > 500) {
    digitalWrite(LED_ORANGE, HIGH);  
    radio_send();        
    digitalWrite(LED_ORANGE, LOW);    
  }
  
}

