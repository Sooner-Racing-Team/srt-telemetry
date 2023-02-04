#include "max6675.h" 

int SO = 3;
int CS = 5;
int sck = 6;
MAX6675 module(sck, CS, SO);

void setup() {   
  Serial.begin(9600);
}

void loop() {
  float temperature = module.readCelsius(); 
  float temperature2 = module.readFahrenheit();
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(F("°C "));   
  delay(1000);
}
