// lifeOs.ino  --  ESP32 + MPU6050 (DMP): Bluetooth sensor/servo link + Wi-Fi video
//
// Two concurrent channels, each playing to its strength:
//   * BLUETOOTH (Classic SPP, always on) -- sensor telemetry out (quaternion +
//     raw counts + servo echo), servo commands in, and Wi-Fi PROVISIONING in.
//     Reliable, network-independent, keeps the laptop's Wi-Fi free.
//   * WI-FI (station, on demand) -- high-bandwidth VIDEO out over UDP. Brought
//     up only after the laptop sends Wi-Fi credentials + its own IP over
//     Bluetooth, so a changing laptop DHCP address is handled every boot and
//     nothing is hardcoded. (Camera is future work; for now a synthetic frame
//     stream lets us test the Wi-Fi path.)
//
// Provisioning line (over Bluetooth):  wifi:<ssid>|<password>|<laptop_ip>
// Identity query (over Bluetooth):     id?  ->  id:lifeos,proto:1,servos:N
//
// Concurrency: an IMU task on core 0 drains the DMP FIFO (interrupt on GPIO4);
// loop() on core 1 runs Bluetooth + the Wi-Fi video sender.
//
// Libraries: "MPU6050" by Electronic Cats, "ESP32Servo". WiFi/WiFiUdp/
// BluetoothSerial ship with the ESP32 core. BT + Wi-Fi + the DMP blob need a
// large partition scheme (Tools > Partition Scheme > "Huge APP").

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <ESP32Servo.h>
#include "BluetoothSerial.h"
#include <WiFi.h>
#include <WiFiUdp.h>

#define INTERRUPT_PIN 4              // MPU6050 INT -> ESP32 GPIO4
const char*  BT_NAME       = "lifeos";
const long   interval      = 20;     // sensor TX period, ms (50 Hz)
const int    VIDEO_PORT    = 5010;   // laptop receives video UDP here
const size_t VIDEO_PKT     = 1024;   // synthetic video packet size, bytes
const long   videoInterval = 20;     // video frame period, ms

// --- Servos: GPIOs clear of the MPU (I2C 21/22, INT 4), flash, strapping and
// input-only pins. Signal wire only; power servos from a separate 5-6V supply
// sharing ground with the ESP32. ---
#define NUM_SERVOS 2
const int SERVO_PINS[NUM_SERVOS] = {13, 25};
Servo servos[NUM_SERVOS];
int   servoPos[NUM_SERVOS] = {90, 90};   // last commanded angle; echoed in telemetry

MPU6050 mpu;
BluetoothSerial SerialBT;

// --- Wi-Fi video state (provisioned over Bluetooth) ---
WiFiUDP   videoUdp;
String    wifiSsid, wifiPass;
IPAddress laptopIp;
enum VideoState { VID_IDLE, VID_CONNECTING, VID_STREAMING };
VideoState    videoState    = VID_IDLE;
unsigned long lastVideoTime = 0;
uint32_t      videoSeq      = 0;

// --- DMP state (touched only by setup + the IMU task) ---
bool     dmpReady   = false;
uint8_t  devStatus;
uint16_t packetSize;
uint8_t  fifoBuffer[64];

volatile bool mpuInterrupt = false;
void IRAM_ATTR dmpDataReady() { mpuInterrupt = true; }

// --- Shared latest sample, written by the IMU task, read by loop() ---
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
volatile float    sQ[4]     = {1, 0, 0, 0};
volatile int16_t  sAccel[3] = {0, 0, 0};
volatile int16_t  sGyro[3]  = {0, 0, 0};
volatile uint32_t sSeq      = 0;

TaskHandle_t imuTaskHandle = NULL;
unsigned long previousTime = 0;
uint32_t      lastSentSeq  = 0;


// ---------------------------------------------------------------------------
// IMU task: drain the DMP FIFO and publish the newest quaternion + raw counts.
// ---------------------------------------------------------------------------
void imuTask(void *param) {
  Quaternion   q;
  VectorInt16  accel;
  int16_t      gyro[3];

  for (;;) {
    if (!dmpReady) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

    if (!mpuInterrupt && mpu.getFIFOCount() < packetSize) {
      vTaskDelay(1);
      continue;
    }
    mpuInterrupt = false;

    uint8_t  intStatus = mpu.getIntStatus();
    uint16_t fifoCount = mpu.getFIFOCount();

    if ((intStatus & 0x10) || fifoCount >= 1024) {   // FIFO overflow -> resync
      mpu.resetFIFO();
      continue;
    }

    if (intStatus & 0x02) {
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


// ---------------------------------------------------------------------------
// Bluetooth: sensor telemetry out; commands + provisioning in.
// ---------------------------------------------------------------------------
void buildTelemetry(char *buffer, size_t n,
                    const float q[4], const int16_t a[3], const int16_t g[3]) {
  // dmp = MPU DMP producing orientation; wf = Wi-Fi video streaming. These let
  // the GUI status bar light the MPU and Wi-Fi indicators truthfully.
  snprintf(buffer, n,
           "q0:%.4f,q1:%.4f,q2:%.4f,q3:%.4f,ax:%d,ay:%d,az:%d,gx:%d,gy:%d,gz:%d,"
           "s0:%d,s1:%d,dmp:%d,wf:%d",
           q[0], q[1], q[2], q[3], a[0], a[1], a[2], g[0], g[1], g[2],
           servoPos[0], servoPos[1],
           dmpReady ? 1 : 0, (videoState == VID_STREAMING) ? 1 : 0);
}

void applyServoCommand(char *buf) {
  for (char *tok = strtok(buf, ","); tok; tok = strtok(NULL, ",")) {
    int idx, ang;
    if (sscanf(tok, "s%d:%d", &idx, &ang) == 2 && idx >= 0 && idx < NUM_SERVOS) {
      if (ang < 0)   ang = 0;
      if (ang > 180) ang = 180;
      servoPos[idx] = ang;
      servos[idx].write(ang);
    }
  }
}

// Start (or restart) the Wi-Fi station link for video. args = "ssid|pass|ip".
void provisionWifi(char *args) {
  char *ssid = strtok(args, "|");
  char *pass = strtok(NULL, "|");
  char *ips  = strtok(NULL, "|");
  if (!(ssid && pass && ips)) {
    SerialBT.println("wifi:error,bad-args");
    return;
  }
  if (!laptopIp.fromString(ips)) {
    SerialBT.println("wifi:error,bad-ip");
    return;
  }
  wifiSsid = ssid;
  wifiPass = pass;
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  videoState = VID_CONNECTING;
  SerialBT.printf("wifi:connecting,%s\n", wifiSsid.c_str());
  Serial.printf("Provisioned Wi-Fi '%s', video -> %s:%d\n", wifiSsid.c_str(), ips, VIDEO_PORT);
}

void handleBtLine(char *line) {
  if (strcmp(line, "id?") == 0) {
    SerialBT.printf("id:lifeos,proto:1,servos:%d\n", NUM_SERVOS);
  } else if (strncmp(line, "wifi:", 5) == 0) {
    provisionWifi(line + 5);
  } else {
    applyServoCommand(line);
  }
}

// Accumulate incoming Bluetooth bytes into lines and dispatch each.
void btPoll() {
  static char line[200];
  static size_t idx = 0;
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c == '\n' || c == '\r') {
      if (idx > 0) { line[idx] = '\0'; handleBtLine(line); idx = 0; }
    } else if (idx < sizeof(line) - 1) {
      line[idx++] = c;
    }
  }
}

void btSendSensor(const float q[4], const int16_t a[3], const int16_t g[3]) {
  if (!SerialBT.hasClient()) return;
  char buffer[200];
  buildTelemetry(buffer, sizeof(buffer), q, a, g);
  SerialBT.println(buffer);
}


// ---------------------------------------------------------------------------
// Wi-Fi video: synthetic frame stream to the provisioned laptop IP.
// Each packet: "vid:<seq>:" header padded with filler to VIDEO_PKT bytes, so
// the PC-side test can measure throughput and detect loss via the sequence.
// ---------------------------------------------------------------------------
void sendVideoFrame() {
  static uint8_t buf[VIDEO_PKT];
  int n = snprintf((char *)buf, VIDEO_PKT, "vid:%u:", videoSeq);
  if (n < 0) n = 0;
  if ((size_t)n < VIDEO_PKT) memset(buf + n, 'X', VIDEO_PKT - n);
  videoUdp.beginPacket(laptopIp, VIDEO_PORT);
  videoUdp.write(buf, VIDEO_PKT);
  videoUdp.endPacket();
  videoSeq++;
}

void updateWifiVideo() {
  if (videoState == VID_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED) {
      videoState = VID_STREAMING;
      SerialBT.printf("wifi:connected,%s\n", WiFi.localIP().toString().c_str());
      Serial.print("Wi-Fi connected as "); Serial.print(WiFi.localIP());
      Serial.print(", streaming video to "); Serial.println(laptopIp);
    }
    return;
  }
  if (videoState == VID_STREAMING) {
    unsigned long now = millis();
    if (now - lastVideoTime >= videoInterval) {
      lastVideoTime = now;
      sendVideoFrame();
    }
  }
}


void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], 500, 2400);
    servos[i].write(servoPos[i]);
  }

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(mpu.testConnection() ? "MPU6050 OK" : "MPU6050 connection FAILED");

  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println("DMP ready");
  } else {
    Serial.print("DMP init failed, code "); Serial.println(devStatus);
  }

  SerialBT.begin(BT_NAME);
  Serial.printf("Bluetooth SPP up as \"%s\"; provision Wi-Fi/video over it.\n", BT_NAME);

  xTaskCreatePinnedToCore(imuTask, "imuTask", 4096, NULL, 3, &imuTaskHandle, 0);
}


void loop() {
  btPoll();              // sensor commands + Wi-Fi provisioning
  updateWifiVideo();     // Wi-Fi connect state machine + video frames

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

  if (seq == lastSentSeq) return;
  lastSentSeq = seq;

  btSendSensor(q, a, g);
}
