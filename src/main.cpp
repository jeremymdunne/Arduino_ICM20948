#include <Arduino.h>
#include <Arduino_ICM20948.h>

Arduino_ICM20948 imu; 
Arduino_ICM20948::ICM20948_Raw_Data data; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 
  delay(2000); 
  Serial.println("Hello, World!"); 
  int status = imu.begin(); 
  Serial.println("Status Code: " + String(status));
}

void loop() {
  if(imu.update() > 0){
    imu.get_data(&data); 
    Serial.println("Accel: " + String(data.accel[0]) + "\t" + String(data.accel[1]) + "\t" + String(data.accel[2]));
    Serial.println("Gyro: " + String(data.gyro[0]) + "\t" + String(data.gyro[1]) + "\t" + String(data.gyro[2]));
    Serial.println("Mag: " + String(data.mag[0]) + "\t" + String(data.mag[1]) + "\t" + String(data.mag[2]));
    Serial.println();
  }
  delay(100);
}