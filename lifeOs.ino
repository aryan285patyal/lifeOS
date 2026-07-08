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
// Stop video / Wi-Fi off (Bluetooth):  wifi:off  ->  wifi:off,ok
// Identity query (over Bluetooth):     id?  ->  id:lifeos,proto:1,servos:N
// IMU recalibration (over Bluetooth):  cal  ->  cal:start ... cal:done
//   (re-runs the GYRO bias calibration; keep the sensor still ~2 s in any
//    orientation; telemetry pauses while it runs. Accel offsets are never
//    touched here -- see recalibrateImu() for why the library's accel cal
//    is broken; use the GUI's 6-position wizard instead)
// 6-position accel cal (Bluetooth):    acal:set,<bx>,<by>,<bz>  ->  acal:ok,<ox>,<oy>,<oz>
//   (per-axis accel bias in raw +/-2g counts, solved by the GUI's six-face
//    wizard; converted to hardware offset-register units, written to the MPU,
//    and persisted in NVS so they survive power cycles)
//   acal:clear  ->  acal:cleared  (drop stored offsets; next boot auto-cals)
// Servo enable/disable (Bluetooth):    e0:1,e1:0  (0 = detach, servo goes limp)
// Debug echo (USB serial monitor):     debug  ->  toggles echoing every BT
//   telemetry line to USB Serial (plus a 1 Hz Wi-Fi/video status line and the
//   stored acal offsets on enable), whether or not a BT client is connected.
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
#include <Preferences.h>

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
bool  servoEnabled[NUM_SERVOS] = {true, true};  // detached (limp) when false; echoed as e0/e1

MPU6050 mpu;
BluetoothSerial SerialBT;

// --- 6-position accel calibration: offset-register values persisted in NVS
// (the chip loses them on power cycle; dmpInitialize() also resets them) ---
Preferences prefs;
bool acalStored = false;             // NVS holds valid 6-position offsets

bool debugMode = false;              // "debug" over USB: echo telemetry to Serial

// --- Wi-Fi video state (provisioned over Bluetooth) ---
WiFiUDP   videoUdp;
String    wifiSsid, wifiPass;
IPAddress laptopIp;
enum VideoState { VID_IDLE, VID_CONNECTING, VID_STREAMING };
VideoState    videoState    = VID_IDLE;
unsigned long lastVideoTime = 0;
uint32_t      videoSeq      = 0;

// --- DMP state (touched only by setup + the IMU task) ---
volatile bool dmpReady = false;      // also cleared by recalibrateImu() on core 1
uint8_t  devStatus;
uint16_t packetSize;
uint8_t  fifoBuffer[64];

volatile bool mpuInterrupt = false;
void IRAM_ATTR dmpDataReady() { mpuInterrupt = true; }

// Recalibration handshake: loop() (core 1) must not touch the I2C bus while the
// IMU task (core 0) is mid-read, so it raises imuPause and waits for the task
// to acknowledge idling via imuPaused before calibrating.
volatile bool imuPause  = false;
volatile bool imuPaused = false;

// --- Shared latest sample, written by the IMU task, read by loop() ---
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
volatile float    sQ[4]     = {1, 0, 0, 0};
volatile int16_t  sAccel[3] = {0, 0, 0};
volatile int16_t  sGyro[3]  = {0, 0, 0};
volatile int16_t  sTemp     = 0;         // die temperature, raw counts
volatile uint32_t sSeq      = 0;

TaskHandle_t imuTaskHandle = NULL;
unsigned long previousTime = 0;
uint32_t      lastSentSeq  = 0;


// ---------------------------------------------------------------------------
// IMU task: drain the DMP FIFO and publish the newest quaternion + raw counts.
// ---------------------------------------------------------------------------
void imuTask(void *param) {
  Quaternion   q;
  int16_t      rawA[3], rawG[3];

  for (;;) {
    if (!dmpReady || imuPause) {
      imuPaused = true;                // signal loop() the I2C bus is free
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    imuPaused = false;

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
      // Accel/gyro come from the sensor's data registers, NOT the DMP FIFO:
      // the FIFO "accel" is a DMP-internal filtered quantity that only equals
      // gravity when flat and collapses toward zero when rotated (measured
      // ~0.03g total while inverted) -- useless as raw counts. The registers
      // hold the true raw measurements at the DMP's FSRs (+/-2g, +/-2000dps).
      mpu.getMotion6(&rawA[0], &rawA[1], &rawA[2], &rawG[0], &rawG[1], &rawG[2]);
      int16_t temp = mpu.getTemperature();   // not in the FIFO; separate register

      portENTER_CRITICAL(&stateMux);
      sQ[0] = q.w; sQ[1] = q.x; sQ[2] = q.y; sQ[3] = q.z;
      sAccel[0] = rawA[0]; sAccel[1] = rawA[1]; sAccel[2] = rawA[2];
      sGyro[0] = rawG[0]; sGyro[1] = rawG[1]; sGyro[2] = rawG[2];
      sTemp = temp;
      sSeq++;
      portEXIT_CRITICAL(&stateMux);
    }
  }
}


// ---------------------------------------------------------------------------
// Bluetooth: sensor telemetry out; commands + provisioning in.
// ---------------------------------------------------------------------------
void buildTelemetry(char *buffer, size_t n, const float q[4],
                    const int16_t a[3], const int16_t g[3], int16_t t) {
  // dmp = MPU DMP producing orientation; wf = Wi-Fi video streaming. These let
  // the GUI status bar light the MPU and Wi-Fi indicators truthfully.
  // tp = die temperature, raw counts (degC = raw/340 + 36.53, done on the PC).
  snprintf(buffer, n,
           "q0:%.4f,q1:%.4f,q2:%.4f,q3:%.4f,ax:%d,ay:%d,az:%d,gx:%d,gy:%d,gz:%d,"
           "tp:%d,s0:%d,s1:%d,e0:%d,e1:%d,dmp:%d,wf:%d",
           q[0], q[1], q[2], q[3], a[0], a[1], a[2], g[0], g[1], g[2],
           t, servoPos[0], servoPos[1],
           servoEnabled[0] ? 1 : 0, servoEnabled[1] ? 1 : 0,
           dmpReady ? 1 : 0, (videoState == VID_STREAMING) ? 1 : 0);
}

void setServoEnabled(int idx, bool en) {
  if (en == servoEnabled[idx]) return;
  servoEnabled[idx] = en;
  if (en) {
    servos[idx].setPeriodHertz(50);
    servos[idx].attach(SERVO_PINS[idx], 500, 2400);
    servos[idx].write(servoPos[idx]);  // resume at the last commanded angle
  } else {
    servos[idx].detach();              // stop pulses; the servo goes limp
  }
}

void applyServoCommand(char *buf) {
  for (char *tok = strtok(buf, ","); tok; tok = strtok(NULL, ",")) {
    int idx, val;
    if (sscanf(tok, "s%d:%d", &idx, &val) == 2 && idx >= 0 && idx < NUM_SERVOS) {
      if (val < 0)   val = 0;
      if (val > 180) val = 180;
      servoPos[idx] = val;             // remembered even while disabled
      if (servoEnabled[idx]) servos[idx].write(val);
    } else if (sscanf(tok, "e%d:%d", &idx, &val) == 2 && idx >= 0 && idx < NUM_SERVOS) {
      setServoEnabled(idx, val != 0);
    }
  }
}

// Re-run the MPU6050 GYRO bias calibration on demand (BT line "cal"). Blocks
// loop() for ~1-2 s, so telemetry pauses; the sensor must be held still (any
// orientation). Accel is deliberately NOT touched: the library's
// CalibrateAccel() drives flat Z to 2g instead of 1g (measured on v1.4.4),
// planting a +1g Z bias in the offset registers -- the cause of the DMP
// roll/pitch decaying toward wrong angles. Accel offsets come only from the
// GUI's 6-position calibration ("acal:set") or factory trim.
void recalibrateImu() {
  if (!dmpReady) {
    SerialBT.println("cal:error,dmp-not-ready");
    return;
  }
  SerialBT.println("cal:start");
  imuPause = true;
  while (!imuPaused) vTaskDelay(1);    // wait for the IMU task to free the bus
  mpu.setDMPEnabled(false);
  mpu.CalibrateGyro(6);
  mpu.resetFIFO();
  mpu.setDMPEnabled(true);
  imuPause = false;
  SerialBT.println("cal:done");
  Serial.println("Gyro biases recalibrated (BT request)");
}

// Apply + persist 6-position accel offsets (BT line "acal:set,<bx>,<by>,<bz>",
// bias in raw +/-2g counts from the GUI wizard) or drop them ("acal:clear").
// Offset registers count 2048 LSB/g vs 16384 LSB/g raw -> divide by 8; bit 0
// of each register is a reserved factory temperature-compensation bit and must
// be preserved.
void setAccelCal(char *args) {
  if (strcmp(args, "clear") == 0) {
    prefs.begin("lifeos", false);
    prefs.remove("aov"); prefs.remove("aox"); prefs.remove("aoy"); prefs.remove("aoz");
    prefs.end();
    acalStored = false;
    SerialBT.println("acal:cleared");
    Serial.println("6-position accel offsets cleared (BT request)");
    return;
  }
  long b[3];
  if (sscanf(args, "set,%ld,%ld,%ld", &b[0], &b[1], &b[2]) != 3) {
    SerialBT.println("acal:error,bad-args");
    return;
  }
  if (abs(b[0]) > 8000 || abs(b[1]) > 8000 || abs(b[2]) > 8000) {
    SerialBT.println("acal:error,bias-too-large");
    return;
  }
  if (!dmpReady) {
    SerialBT.println("acal:error,dmp-not-ready");
    return;
  }
  imuPause = true;
  while (!imuPaused) vTaskDelay(1);
  mpu.setDMPEnabled(false);
  int16_t cur[3] = { mpu.getXAccelOffset(), mpu.getYAccelOffset(), mpu.getZAccelOffset() };
  int16_t reg[3];
  for (int i = 0; i < 3; i++) {
    int16_t nv = cur[i] - (int16_t)lroundf(b[i] / 8.0f);
    reg[i] = (nv & ~1) | (cur[i] & 1);   // keep the factory temp-comp bit
  }
  mpu.setXAccelOffset(reg[0]);
  mpu.setYAccelOffset(reg[1]);
  mpu.setZAccelOffset(reg[2]);
  mpu.resetFIFO();
  mpu.setDMPEnabled(true);
  imuPause = false;
  prefs.begin("lifeos", false);
  prefs.putShort("aox", reg[0]);
  prefs.putShort("aoy", reg[1]);
  prefs.putShort("aoz", reg[2]);
  prefs.putBool("aov", true);
  prefs.end();
  acalStored = true;
  SerialBT.printf("acal:ok,%d,%d,%d\n", reg[0], reg[1], reg[2]);
  Serial.printf("6-position accel offsets set: %d %d %d (persisted)\n",
                reg[0], reg[1], reg[2]);
}

// Active Wi-Fi shutdown (BT line "wifi:off"): stop video and turn the radio off
// until the next provisioning, instead of leaving the laptop to just ignore it.
void stopWifiVideo() {
  videoState = VID_IDLE;
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  SerialBT.println("wifi:off,ok");
  Serial.println("Wi-Fi video stopped (BT request)");
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
  } else if (strcmp(line, "cal") == 0) {
    recalibrateImu();
  } else if (strncmp(line, "acal:", 5) == 0) {
    setAccelCal(line + 5);
  } else if (strcmp(line, "wifi:off") == 0) {   // before the wifi: prefix match
    stopWifiVideo();
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

void btSendSensor(const float q[4], const int16_t a[3], const int16_t g[3],
                  int16_t t) {
  if (!SerialBT.hasClient() && !debugMode) return;
  char buffer[200];
  buildTelemetry(buffer, sizeof(buffer), q, a, g, t);
  if (SerialBT.hasClient()) SerialBT.println(buffer);
  if (debugMode) Serial.println(buffer);   // exact line the PC would receive
}

// --- USB-serial debug console: "debug" toggles echoing everything the ESP32
// sends (BT telemetry verbatim + 1 Hz Wi-Fi/video status) to the monitor. ---
// Dump sensor config + a raw sample to USB Serial. Callers must own the I2C
// bus (boot before the IMU task starts, or inside an imuPause window).
void printSensorState(const char *tag) {
  int16_t a[3], g[3];
  mpu.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]);
  Serial.printf("dbg %s: fsr accel=%d gyro=%d | accel offs %d %d %d | "
                "gyro offs %d %d %d | raw a %d %d %d g %d %d %d\n",
                tag, mpu.getFullScaleAccelRange(), mpu.getFullScaleGyroRange(),
                mpu.getXAccelOffset(), mpu.getYAccelOffset(), mpu.getZAccelOffset(),
                mpu.getXGyroOffset(), mpu.getYGyroOffset(), mpu.getZGyroOffset(),
                a[0], a[1], a[2], g[0], g[1], g[2]);
}

void setDebugMode(bool on) {
  debugMode = on;
  Serial.printf("dbg: telemetry echo %s\n", on ? "ON" : "OFF");
  if (!on) return;
  prefs.begin("lifeos", false);
  Serial.printf("dbg: acal stored=%d ox=%d oy=%d oz=%d | bt client=%d\n",
                prefs.getBool("aov", false) ? 1 : 0,
                prefs.getShort("aox", 0), prefs.getShort("aoy", 0),
                prefs.getShort("aoz", 0), SerialBT.hasClient() ? 1 : 0);
  prefs.end();
  if (dmpReady) {                      // grab the bus safely for the dump
    imuPause = true;
    while (!imuPaused) vTaskDelay(1);
    printSensorState("now");
    imuPause = false;
  }
}

void debugVideoTick() {
  static unsigned long lastPrint = 0;
  static uint32_t lastSeq = 0;
  unsigned long now = millis();
  if (now - lastPrint < 1000) return;
  const char *st = (videoState == VID_STREAMING)  ? "streaming" :
                   (videoState == VID_CONNECTING) ? "connecting" : "idle";
  Serial.printf("dbg wifi: state=%s pkts/s=%u -> %s:%d\n",
                st, videoSeq - lastSeq, laptopIp.toString().c_str(), VIDEO_PORT);
  lastPrint = now;
  lastSeq = videoSeq;
}

void usbPoll() {
  static char line[64];
  static size_t idx = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (idx > 0) {
        line[idx] = '\0';
        idx = 0;
        if (strcmp(line, "debug") == 0) setDebugMode(!debugMode);
        else Serial.printf("dbg: unknown command '%s' (only: debug)\n", line);
      }
    } else if (idx < sizeof(line) - 1) {
      line[idx++] = c;
    }
  }
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
    // dmpInitialize() reset the chip, so offsets must be re-applied every boot.
    // Accel: stored 6-position offsets from NVS, else factory trim. NEVER the
    // library's CalibrateAccel() -- it drives flat Z to 2g instead of 1g
    // (measured on v1.4.4), the +1g Z bias that made DMP roll/pitch decay
    // toward wrong angles. Gyro bias cal is correct and orientation-agnostic
    // (needs stillness only), so it always runs.
    prefs.begin("lifeos", false);
    acalStored = prefs.getBool("aov", false);
    if (acalStored) {
      mpu.setXAccelOffset(prefs.getShort("aox", 0));
      mpu.setYAccelOffset(prefs.getShort("aoy", 0));
      mpu.setZAccelOffset(prefs.getShort("aoz", 0));
      Serial.println("Applied stored 6-position accel offsets from NVS");
    }
    prefs.end();
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    printSensorState("boot");
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
  usbPoll();             // USB serial monitor: "debug" toggle
  updateWifiVideo();     // Wi-Fi connect state machine + video frames
  if (debugMode) debugVideoTick();

  unsigned long currentTime = millis();
  if (currentTime - previousTime < interval) return;
  previousTime = currentTime;

  float    q[4];
  int16_t  a[3], g[3], t;
  uint32_t seq;

  portENTER_CRITICAL(&stateMux);
  q[0] = sQ[0]; q[1] = sQ[1]; q[2] = sQ[2]; q[3] = sQ[3];
  a[0] = sAccel[0]; a[1] = sAccel[1]; a[2] = sAccel[2];
  g[0] = sGyro[0]; g[1] = sGyro[1]; g[2] = sGyro[2];
  t    = sTemp;
  seq  = sSeq;
  portEXIT_CRITICAL(&stateMux);

  if (seq == lastSentSeq) return;
  lastSentSeq = seq;

  btSendSensor(q, a, g, t);
}
