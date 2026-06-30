#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "secrets.h"   // WIFI_SSID, WIFI_PASSWORD, PC_IP -- copy secrets.example.h to secrets.h
WiFiUDP udp;

unsigned long previousTime = 0;
const long interval = 100;

const char* SSID     = WIFI_SSID;
const char* PASSWORD = WIFI_PASSWORD;
const char* PC_IP_ADDR = PC_IP;         // receiving PC's IP, from secrets.h
const int   UDP_PORT = 5005;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire.beginTransmission(0x68); // this is the MPU6050's I2C address
  Wire.write(0x6B); // now pointing to PWR_MGMT_1 register
  Wire.write(0x00); // writing 0
  Wire.endTransmission(true);
  Serial.begin(115200);

  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
}
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(UDP_PORT); // allocate and bind the WiFiUDP socket to a known local port
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentTime = millis();
  if(delayPassed(currentTime)){
    readFromSensor();
  }
}



bool delayPassed(unsigned long currentTime){
  if(currentTime - previousTime >= interval){
    previousTime = currentTime;
    return(true);
  }
  return false;
}


void readFromSensor(){
  Wire.beginTransmission(0x68);
  Wire.write(0x3b);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14);

  int16_t accel_x = (Wire.read() << 8) | Wire.read();
  int16_t accel_y = (Wire.read() << 8) | Wire.read();
  int16_t accel_z = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();      // skip temp
  int16_t gyro_x  = (Wire.read() << 8) | Wire.read();
  int16_t gyro_y  = (Wire.read() << 8) | Wire.read();
  int16_t gyro_z  = (Wire.read() << 8) | Wire.read();

  printToSerial(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
  sendUDP(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);

}

void printToSerial(int16_t ax, int16_t ay, int16_t az,
                   int16_t gx, int16_t gy, int16_t gz) {
    Serial.println("=============================");
    Serial.print("ACCEL (raw)  X: "); Serial.print(ax);
    Serial.print("  Y: ");            Serial.print(ay);
    Serial.print("  Z: ");            Serial.println(az);
    Serial.print("GYRO  (raw)  X: "); Serial.print(gx);
    Serial.print("  Y: ");            Serial.print(gy);
    Serial.print("  Z: ");            Serial.println(gz);
    Serial.println("=============================");
    Serial.println();
}

void sendUDP(int16_t ax, int16_t ay, int16_t az,
             int16_t gx, int16_t gy, int16_t gz) {
    char buffer[64];
    sprintf(buffer, "ax:%d,ay:%d,az:%d,gx:%d,gy:%d,gz:%d",
            ax, ay, az, gx, gy, gz);

    udp.beginPacket(PC_IP_ADDR, UDP_PORT);
    udp.write((uint8_t*)buffer, strlen(buffer));
    udp.endPacket();
}