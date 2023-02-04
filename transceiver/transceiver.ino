#include <SoftwareSerial.h>
#include <math.h>

SoftwareSerial Serial1(10,11); // HC-12 TX Pin, HC-12 RX Pin

void setup() {
  Serial.begin(9600);             // Serial port to computer
  Serial1.begin(9600);               // Serial port to HC12

}
float counter = 0;
void loop() {
 
  counter = counter + .1;
  float x;
  x = sin(counter);

  Serial1.println(x);
  
  delay(100);
  while (Serial1.available()) {        // If HC-12 has data
    Serial.write(Serial1.read());      // Send the data to Serial monitor
  }
  while (Serial.available()) {      // If Serial monitor has data
    Serial1.write(Serial.read());      // Send that data to HC-12
  }
}
