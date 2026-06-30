// lifeOs.ino  --  ESP32 + MPU6050 (DMP) -> UDP telemetry
//
// The MPU6050's on-chip DMP produces a fused orientation quaternion; we also
// forward the raw accel/gyro counts so the PC-side Monitor tab keeps working.
//
// Concurrency (ESP-WROOM-32, dual core):
//   * IMU task pinned to core 0 drains the DMP FIFO promptly (interrupt-driven
//     on GPIO4) and publishes the latest sample into a spinlock-guarded struct.
//   * The Arduino loop() (core 1) reads that struct and transmits over UDP.
// This keeps the IMU FIFO read path off the TX/WiFi path, so a future on-board
// camera (JPEG encode) on core 1 can never starve the FIFO and force resyncs.
//
// Requires the "MPU6050" library by Electronic Cats (bundles I2Cdev +
// MPU6050_6Axis_MotionApps20). WiFi creds + PC IP live in secrets.h.

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "secrets.h"   // WIFI_SSID, WIFI_PASSWORD, PC_IP

#define INTERRUPT_PIN 4              // MPU6050 INT -> ESP32 GPIO4
const int   UDP_PORT = 5005;
const long  interval = 20;           // UDP TX period, ms (50 Hz)

MPU6050 mpu;
WiFiUDP  udp;

// --- DMP state (touched only by setup + the IMU task) ---
bool     dmpReady   = false;
uint8_t  devStatus;                  // dmpInitialize() return code (0 = success)
uint16_t packetSize;                 // expected DMP FIFO packet size
uint8_t  fifoBuffer[64];

// --- INT pin -> data-ready flag ---
volatile bool mpuInterrupt = false;
void IRAM_ATTR dmpDataReady() { mpuInterrupt = true; }

// --- Shared latest sample, written by the IMU task, read by loop() ---
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
volatile float    sQ[4]     = {1, 0, 0, 0};   // w, x, y, z
volatile int16_t  sAccel[3] = {0, 0, 0};
volatile int16_t  sGyro[3]  = {0, 0, 0};
volatile uint32_t sSeq      = 0;               // bumped on every fresh sample

TaskHandle_t imuTaskHandle = NULL;

unsigned long previousTime = 0;
uint32_t      lastSentSeq  = 0;


// ---------------------------------------------------------------------------
// IMU task: drain the DMP FIFO and publish the newest quaternion + raw counts.
// Pinned to core 0 so it never waits behind WiFi/UDP (or a future camera) work.
// ---------------------------------------------------------------------------
void imuTask(void *param) {
  Quaternion   q;
  VectorInt16  accel;
  int16_t      gyro[3];

  for (;;) {
    if (!dmpReady) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

    if (!mpuInterrupt && mpu.getFIFOCount() < packetSize) {
      vTaskDelay(1);                 // nothing ready yet -> yield the core
      continue;
    }
    mpuInterrupt = false;

    uint8_t  intStatus = mpu.getIntStatus();
    uint16_t fifoCount = mpu.getFIFOCount();

    // Overflow: the documented stale/garbage-quaternion failure mode. Resync.
    if ((intStatus & 0x10) || fifoCount >= 1024) {
      mpu.resetFIFO();
      continue;
    }

    if (intStatus & 0x02) {          // DMP data ready
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&accel, fifoBuffer);
      mpu.dmpGetGyro(gyro, fifoBuffer);

      portENTER_CRITICAL(&stateMux);
      sQ[0] = q.w; sQ[1] = q.x; sQ[2] = q.y; sQ[3] = q.z;
      sAccel[0] = accel.x; sAccel[1] = accel.y; sAccel[2] = accel.z;
      sGyro[0] = gyro[0]; sGyro[1] = gyro[1]; sGyro[2] = gyro[2];
      sSeq++;
      portEXIT_CRITICAL(&stateMux);
    }
  }
}


void sendUDP(const float q[4], const int16_t a[3], const int16_t g[3]) {
  char buffer[160];
  snprintf(buffer, sizeof(buffer),
           "q0:%.4f,q1:%.4f,q2:%.4f,q3:%.4f,ax:%d,ay:%d,az:%d,gx:%d,gy:%d,gz:%d",
           q[0], q[1], q[2], q[3], a[0], a[1], a[2], g[0], g[1], g[2]);
  udp.beginPacket(PC_IP, UDP_PORT);
  udp.write((uint8_t*)buffer, strlen(buffer));
  udp.endPacket();
}


void setup() {
  Wire.begin();
  Wire.setClock(400000);             // 400 kHz I2C for prompt FIFO reads
  Serial.begin(115200);

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(mpu.testConnection() ? "MPU6050 OK" : "MPU6050 connection FAILED");

  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    // Calibrate at rest: keep the sensor still and level for a couple seconds.
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println("DMP ready");
  } else {
    // 1 = initial memory load failed, 2 = DMP config updates failed
    Serial.print("DMP init failed, code "); Serial.println(devStatus);
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
  udp.begin(UDP_PORT);

  // IMU on core 0; loop()/WiFi run on core 1.
  xTaskCreatePinnedToCore(imuTask, "imuTask", 4096, NULL, 3, &imuTaskHandle, 0);
}


void loop() {
  unsigned long currentTime = millis();
  if (currentTime - previousTime < interval) return;
  previousTime = currentTime;

  float    q[4];
  int16_t  a[3], g[3];
  uint32_t seq;

  portENTER_CRITICAL(&stateMux);
  q[0] = sQ[0]; q[1] = sQ[1]; q[2] = sQ[2]; q[3] = sQ[3];
  a[0] = sAccel[0]; a[1] = sAccel[1]; a[2] = sAccel[2];
  g[0] = sGyro[0]; g[1] = sGyro[1]; g[2] = sGyro[2];
  seq  = sSeq;
  portEXIT_CRITICAL(&stateMux);

  if (seq == lastSentSeq) return;    // no fresh sample since last TX
  lastSentSeq = seq;

  sendUDP(q, a, g);
}
