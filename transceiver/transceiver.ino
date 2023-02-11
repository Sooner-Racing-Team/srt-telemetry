#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "max6675.h" 
#include <SoftwareSerial.h>
#include <math.h>

/*****************************************************************************************/
/**********************************- GLOBAL VARIABLES - **********************************/

// Accelerometer Global Variables

Adafruit_MPU6050 mpu; // Accelerometer library function needed for variables used in accel code
double a_avg_x = 0;
double a_movingAverage = 0;
double time_total = 0;

double a_treadmill_x[10] = {0};

// Thermocouple Global Variables

int SO = 2; // has to be digital pin (changed from the original thermocouple code where the pins were digital PWM, troubleshoot this if thermocouple doesn't work
int CS = 3; // digital pin
int sck = 4; // digital pin
MAX6675 module(sck, CS, SO);


// Linear Potentiometer Global Variables

const int potPin = A3; // pin A0 to read analog input
int value; // save analog value


// Transceiver Global Variables
SoftwareSerial HC12(3,2); // HC-12 TX Pin, HC-12 RX Pin
float counter = 0;

/*****************************************************************************************/

void setup() {
 Serial.begin(9600);
Serial.print("Void setup has been entered");
double a_avg_x, milli,temperature_F;
  
 /* ***********************/
 // ACCELEROMETER SETUP CODE
 /* if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip"); // debugging failsafe in case the accelerometer cannot be communicated with
    while (1) {
      delay(10);
    }
   */ 
    
  //}
  Serial.println("MPU6050 Found!");

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // range of g's that can be read, +- 2 to +- 16 is available. The smaller the range, the more accurate the reported values. 

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); // gyroscopic set range (no need to touch ever)

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); //Default bandwidth used by accelerometer

  
  
 /* ***********************/
 // THERMOCOUPLE SETUP CODE


 /* ***********************/ 
 // LINEAR POTENTIOMETER SETUP CODE

  pinMode(potPin, INPUT); 

 /* ***********************/ 
 // TRANSCEIVER SETUP CODE
 
 HC12.begin(9600); // Serial port to HC12

while (true) {
  Serial.begin(9600);
  delay(100);
  // Serial.print("in da perma loop");
a_avg_x = accelerometer(); // run code to collect acceleration data

temperature_F = temperature(); // run code to collect temperature data

milli = potentiometer(); // run code to collect potentiometer data

transmit(a_avg_x, milli, temperature_F); // run code to send the collected sensor data to the receiving transceiver

  }
}

void loop() {
Serial.print("stuck in void loop");
}

 double accelerometer() {
   /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

//Setting bounds on X-acceleration

  if (a.acceleration.x > 2){
    a.acceleration.x = 2;
  }

  if (a.acceleration.x < -2){
    a.acceleration.x = -2;
  }
  
  /* Print out the values */
  //Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  // Serial.print('\t'); 
  /*Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print('\t');
  //Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.print('\t');
  Serial.println(" m/s^2");
*/
  
  a_treadmill_x[0] = a_treadmill_x[1];
  a_treadmill_x[1] = a_treadmill_x[2];
  a_treadmill_x[2] = a_treadmill_x[3];
  a_treadmill_x[3] = a_treadmill_x[4];
  a_treadmill_x[4] = a_treadmill_x[5];
  a_treadmill_x[5] = a_treadmill_x[6];
  a_treadmill_x[6] = a_treadmill_x[7];
  a_treadmill_x[7] = a_treadmill_x[8];
  a_treadmill_x[8] = a_treadmill_x[9];
  a_treadmill_x[9] = a.acceleration.x;

  a_avg_x = (a_treadmill_x[0] + a_treadmill_x[1] + a_treadmill_x[2] + a_treadmill_x[3] + a_treadmill_x[4] + a_treadmill_x[5] + a_treadmill_x[6] + a_treadmill_x[7] + a_treadmill_x[8] + a_treadmill_x[9]) / 10;
  
  /*a_total_x = a_total_x + a.acceleration.x; //increments a_total_x by the newly read a.acceleration.x
  time_total = time_total + 1;
  a_movingAverage = a_total_x / time_total;  
  */
  
  //Serial.print("Moving average: ");
  Serial.print(a_avg_x);
  Serial.println(" Gs");
//  Serial.print('\t'); //needed for plotting multiple variables; must be placed after every variable


  //reset total for weighing purposes
  /*if (time_total > 5){
     a_avg_x = 0;
     time_total = 0;
  }
    */

 // Serial.print("Rotation X: ");
  //Serial.print(g.gyro.x);
  //Serial.print(", Y: ");
 /* Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");
*/
 // Serial.println("");


  return (a_avg_x);
  
}

double temperature() {
  float temperature = module.readCelsius(); 
  float temperature_F = module.readFahrenheit(); // library command for reading the fahrenheit module
 // Serial.print("Temperature: ");
 // Serial.print(temperature);
//  Serial.println(F("°C "));   
//  delay(1000);
  return temperature_F;
}

double potentiometer() {

   value = analogRead(potPin);          //Read and save analog value from potentiometer
  float calibration_metric = 52.0/1023;
  float calibration_imperial = 2.047/1023;

  float inches = calibration_imperial * value;
  float milli = calibration_metric * value;
//  Serial.print(inches);
//  Serial.println(" inches");
  //Serial.print("Raw value");
 // Serial.println(value);
  Serial.print(milli);
  Serial.println(" millimeters");
  // Serial.println(value);
 // delay(100);                          //Small delay
  
  return (milli);
}

double transmit(double accel, double millimeters, double temp) {

HC12.print(accel);
HC12.println(" Gs");
//HC12.println(temp);
// HC12.print(millimeters);
// HC12.println(" mm");

  while (HC12.available()) {        // If HC-12 has data
    Serial.write(HC12.read());      // Send the data to Serial monitor
  }
  while (Serial.available()) {      // If Serial monitor has data
    HC12.write(Serial.read());      // Send that data to HC-12
  }
  Serial.println("Yum 2");
}
