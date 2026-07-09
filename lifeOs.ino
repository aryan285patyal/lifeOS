// lifeOs.ino  --  ESP32 + MPU6050 (DMP): sensor/servo link + Wi-Fi video
//
// Two boards, one firmware (select below):
//   * BOARD_WROOM32 (ESP-WROOM-32): the sensor/control feed is BLUETOOTH
//     Classic SPP (always on) -- reliable, network-independent, keeps the
//     laptop's Wi-Fi free.
//   * BOARD_S3CAM (GoouuuTech ESP32-S3-CAM, WROOM-1 N16R8 + OV3660): the S3
//     has NO Bluetooth Classic, so the feed starts on USB SERIAL (the CH343
//     "COM" USB-C port; same newline protocol). After Wi-Fi is provisioned
//     over USB, "feed:wifi" moves the sensor/servo feed onto Wi-Fi UDP
//     (telemetry -> laptop:5005, commands in on 5006) so the board can run
//     untethered; "feed:usb" brings it back.
//
// Either way, WI-FI (station, on demand) carries high-bandwidth VIDEO over
// UDP (port 5010). It is brought up only after the laptop sends Wi-Fi
// credentials + its own IP over the feed link, so a changing laptop DHCP
// address is handled every session and nothing is hardcoded. (Real camera is
// future work; for now a synthetic frame stream tests the Wi-Fi path.)
//
// Control lines (over the active feed link):
//   wifi:<ssid>|<password>|<laptop_ip>   provision Wi-Fi -> wifi:connected,<ip>
//   wifi:off                             stop video + radio -> wifi:off,ok
//   id?                                  -> id:lifeos,proto:1,servos:N,board:<b>
//   cal                                  gyro bias recal -> cal:start ... cal:done
//     (keep the sensor still ~2 s, any orientation; telemetry pauses. Accel
//      offsets are never touched here -- see recalibrateImu() for why the
//      library's accel cal is broken; use the GUI's 6-position wizard)
//   acal:set,<bx>,<by>,<bz>              -> acal:ok,<ox>,<oy>,<oz>
//     (per-axis accel bias in raw +/-2g counts from the GUI's six-face wizard;
//      converted to offset-register units, written to the MPU, persisted in NVS)
//   acal:clear                           -> acal:cleared (next boot auto-cals)
//   e0:1,e1:0                            servo enable/disable (0 = detach/limp)
//   feed:wifi / feed:usb                 S3 only: move the feed to Wi-Fi UDP / back
//   debug                                toggle telemetry echo + 1 Hz status on
//                                        the USB serial monitor
//
// Concurrency: an IMU task on core 0 drains the DMP FIFO (INT pin interrupt);
// loop() on core 1 runs the feed link + the Wi-Fi video sender.
//
// Libraries: "MPU6050" by Electronic Cats, "ESP32Servo". WiFi/WiFiUdp (and
// BluetoothSerial on the WROOM-32) ship with the ESP32 core.
// Partitions: WROOM-32 needs Tools > Partition Scheme > "Huge APP" (BT + Wi-Fi
// + DMP blob overflow the default). S3-CAM (16 MB flash) fits the default
// 16 MB scheme; select board "ESP32S3 Dev Module", Flash 16MB, PSRAM "OPI".

// --- Board selection: exactly one. Chooses pins + the feed transport. ---
#define BOARD_S3CAM        // GoouuuTech ESP32-S3-CAM (ESP32-S3-WROOM-1 N16R8)
//#define BOARD_WROOM32    // classic ESP-WROOM-32 dev board

#if defined(BOARD_S3CAM) && defined(BOARD_WROOM32)
#error "Select exactly one board (BOARD_S3CAM or BOARD_WROOM32)"
#endif
#if !defined(BOARD_S3CAM) && !defined(BOARD_WROOM32)
#error "Select a board: BOARD_S3CAM or BOARD_WROOM32"
#endif

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <ESP32Servo.h>
#if defined(BOARD_WROOM32)
#include "BluetoothSerial.h"       // BT Classic: exists only on the WROOM-32
#endif
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <stdarg.h>

const long   interval      = 20;     // sensor TX period, ms (50 Hz)
const int    VIDEO_PORT    = 5010;   // laptop receives video UDP here
const size_t VIDEO_PKT     = 1024;   // synthetic video packet size, bytes
const long   videoInterval = 20;     // video frame period, ms

// --- Pins + servo count per board. Signal wire only; power servos from a
// separate 5-6V supply sharing ground with the ESP32. ---
#define NUM_SERVOS 2
#if defined(BOARD_S3CAM)
// The S3-CAM's only free GPIOs are 1, 2, 3, 14, 21, 47: the camera owns
// 4-13/15-18, the SD slot 38-40, the WS2812 LED 48, native USB 19/20, and
// 0/45/46 are strapping pins. GPIO3 (strapping-ish) is left as the spare.
#define PIN_SDA 21                   // MPU6050 SDA
#define PIN_SCL 14                   // MPU6050 SCL
#define INTERRUPT_PIN 47             // MPU6050 INT
const int SERVO_PINS[NUM_SERVOS] = {1, 2};
#define BOARD_NAME "s3cam"
#else
#define PIN_SDA 21                   // MPU6050 SDA
#define PIN_SCL 22                   // MPU6050 SCL
#define INTERRUPT_PIN 4              // MPU6050 INT
const int SERVO_PINS[NUM_SERVOS] = {13, 25};
#define BOARD_NAME "wroom32"
#endif

Servo servos[NUM_SERVOS];
int   servoPos[NUM_SERVOS] = {90, 90};   // last commanded angle; echoed in telemetry
bool  servoEnabled[NUM_SERVOS] = {true, true};  // detached (limp) when false; echoed as e0/e1

MPU6050 mpu;
#if defined(BOARD_WROOM32)
const char* BT_NAME = "lifeos";
BluetoothSerial SerialBT;
#else
// S3 sensor/servo feed: USB serial by default; "feed:wifi" moves it to Wi-Fi
// UDP (telemetry out to laptop:UDP_PORT, command lines in on CMD_PORT --
// matches gui.py's WifiLink, which also sends "hello" to re-teach our peer IP).
const int UDP_PORT = 5005;           // telemetry -> laptop (matches gui.py)
const int CMD_PORT = 5006;           // command/hello listener (matches gui.py)
WiFiUDP dataUdp;                     // telemetry out while the feed is Wi-Fi
WiFiUDP cmdUdp;                      // command listener on CMD_PORT
bool    feedWifi = false;            // true = feed rides Wi-Fi UDP, not USB
bool    cmdUdpUp = false;            // cmdUdp.begin() done
#endif

// --- 6-position accel calibration: offset-register values persisted in NVS
// (the chip loses them on power cycle; dmpInitialize() also resets them) ---
Preferences prefs;
bool acalStored = false;             // NVS holds valid 6-position offsets

bool debugMode = false;              // "debug" over USB: echo telemetry to Serial
int  wifiRssi  = 0;                  // WiFi.RSSI() dBm, sampled 1 Hz; 0 = radio off

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
// Feed link: sensor telemetry out; commands + provisioning in. The transport
// differs per board (BT SPP / USB serial / Wi-Fi UDP) but every protocol line
// goes through linkSendLine, so the rest of the firmware is transport-blind.
// ---------------------------------------------------------------------------
void linkSendLine(const char *s) {
#if defined(BOARD_WROOM32)
  if (SerialBT.hasClient()) SerialBT.println(s);
  if (debugMode) Serial.println(s);    // exact line the PC would receive
#else
  if (feedWifi) {
    dataUdp.beginPacket(laptopIp, UDP_PORT);
    dataUdp.print(s);
    dataUdp.endPacket();
  }
  Serial.println(s);   // USB is the wired feed and doubles as the monitor
#endif
}

void linkPrintf(const char *fmt, ...) {
  char buf[240];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  linkSendLine(buf);
}

void buildTelemetry(char *buffer, size_t n, const float q[4],
                    const int16_t a[3], const int16_t g[3], int16_t t) {
  // dmp = MPU DMP producing orientation; wf = Wi-Fi video streaming. These let
  // the GUI status bar light the MPU and Wi-Fi indicators truthfully.
  // tp = die temperature, raw counts (degC = raw/340 + 36.53, done on the PC).
  // rs = Wi-Fi RSSI in dBm (1 Hz sample; 0 = radio off / not connected).
  snprintf(buffer, n,
           "q0:%.4f,q1:%.4f,q2:%.4f,q3:%.4f,ax:%d,ay:%d,az:%d,gx:%d,gy:%d,gz:%d,"
           "tp:%d,s0:%d,s1:%d,e0:%d,e1:%d,dmp:%d,wf:%d,rs:%d",
           q[0], q[1], q[2], q[3], a[0], a[1], a[2], g[0], g[1], g[2],
           t, servoPos[0], servoPos[1],
           servoEnabled[0] ? 1 : 0, servoEnabled[1] ? 1 : 0,
           dmpReady ? 1 : 0, (videoState == VID_STREAMING) ? 1 : 0, wifiRssi);
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
    linkSendLine("cal:error,dmp-not-ready");
    return;
  }
  linkSendLine("cal:start");
  imuPause = true;
  while (!imuPaused) vTaskDelay(1);    // wait for the IMU task to free the bus
  mpu.setDMPEnabled(false);
  mpu.CalibrateGyro(6);
  mpu.resetFIFO();
  mpu.setDMPEnabled(true);
  imuPause = false;
  linkSendLine("cal:done");
  Serial.println("Gyro biases recalibrated (link request)");
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
    linkSendLine("acal:cleared");
    Serial.println("6-position accel offsets cleared (link request)");
    return;
  }
  long b[3];
  if (sscanf(args, "set,%ld,%ld,%ld", &b[0], &b[1], &b[2]) != 3) {
    linkSendLine("acal:error,bad-args");
    return;
  }
  if (abs(b[0]) > 8000 || abs(b[1]) > 8000 || abs(b[2]) > 8000) {
    linkSendLine("acal:error,bias-too-large");
    return;
  }
  if (!dmpReady) {
    linkSendLine("acal:error,dmp-not-ready");
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
  linkPrintf("acal:ok,%d,%d,%d", reg[0], reg[1], reg[2]);
  Serial.printf("6-position accel offsets set: %d %d %d (persisted)\n",
                reg[0], reg[1], reg[2]);
}

// Active Wi-Fi shutdown (line "wifi:off"): stop video and turn the radio off
// until the next provisioning, instead of leaving the laptop to just ignore it.
// On the S3 this also kills a Wi-Fi feed, so it drops back to USB first.
void stopWifiVideo() {
#if defined(BOARD_S3CAM)
  feedWifi = false;                    // radio going down takes the feed with it
  if (cmdUdpUp) { cmdUdp.stop(); cmdUdpUp = false; }
#endif
  videoState = VID_IDLE;
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  linkSendLine("wifi:off,ok");
  Serial.println("Wi-Fi video stopped (link request)");
}

// S3 only: "feed:wifi" moves the sensor/servo feed onto Wi-Fi UDP (requires
// provisioned Wi-Fi); "feed:usb" brings it back to the USB serial link. The
// reply goes out before/after the switch such that both the old and new
// channel see it (linkSendLine always writes USB too on the S3).
void setFeedWifi(bool on) {
#if defined(BOARD_S3CAM)
  if (on) {
    if (WiFi.status() != WL_CONNECTED || (uint32_t)laptopIp == 0) {
      linkSendLine("feed:error,no-wifi");
      return;
    }
    if (!cmdUdpUp) { cmdUdp.begin(CMD_PORT); cmdUdpUp = true; }
    feedWifi = true;
    linkSendLine("feed:wifi,ok");
    Serial.println("Sensor/servo feed -> Wi-Fi UDP");
  } else {
    feedWifi = false;
    linkSendLine("feed:usb,ok");
    Serial.println("Sensor/servo feed -> USB serial");
  }
#else
  (void)on;
  linkSendLine("feed:error,unsupported");   // WROOM-32: Bluetooth is the feed
#endif
}

// Start (or restart) the Wi-Fi station link for video. args = "ssid|pass|ip".
void provisionWifi(char *args) {
  char *ssid = strtok(args, "|");
  char *pass = strtok(NULL, "|");
  char *ips  = strtok(NULL, "|");
  if (!(ssid && pass && ips)) {
    linkSendLine("wifi:error,bad-args");
    return;
  }
  if (!laptopIp.fromString(ips)) {
    linkSendLine("wifi:error,bad-ip");
    return;
  }
  wifiSsid = ssid;
  wifiPass = pass;
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  videoState = VID_CONNECTING;
  linkPrintf("wifi:connecting,%s", wifiSsid.c_str());
  Serial.printf("Provisioned Wi-Fi '%s', video -> %s:%d\n", wifiSsid.c_str(), ips, VIDEO_PORT);
}

void handleLine(char *line) {
  if (strcmp(line, "id?") == 0) {
    linkPrintf("id:lifeos,proto:1,servos:%d,board:%s", NUM_SERVOS, BOARD_NAME);
  } else if (strcmp(line, "cal") == 0) {
    recalibrateImu();
  } else if (strncmp(line, "acal:", 5) == 0) {
    setAccelCal(line + 5);
  } else if (strcmp(line, "wifi:off") == 0) {   // before the wifi: prefix match
    stopWifiVideo();
  } else if (strncmp(line, "wifi:", 5) == 0) {
    provisionWifi(line + 5);
  } else if (strcmp(line, "feed:wifi") == 0) {
    setFeedWifi(true);
  } else if (strcmp(line, "feed:usb") == 0) {
    setFeedWifi(false);
  } else if (strcmp(line, "debug") == 0) {
    setDebugMode(!debugMode);
  } else {
    applyServoCommand(line);
  }
}

// Accumulate incoming feed bytes into lines and dispatch each. On the
// WROOM-32 the feed is Bluetooth; on the S3 it's USB serial plus -- once
// "feed:wifi" is active -- UDP command datagrams on CMD_PORT ("hello"
// re-teaches our peer IP, everything else is a normal control line).
void feedPoll() {
  static char line[200];
  static size_t idx = 0;
#if defined(BOARD_WROOM32)
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c == '\n' || c == '\r') {
      if (idx > 0) { line[idx] = '\0'; handleLine(line); idx = 0; }
    } else if (idx < sizeof(line) - 1) {
      line[idx++] = c;
    }
  }
#else
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (idx > 0) { line[idx] = '\0'; handleLine(line); idx = 0; }
    } else if (idx < sizeof(line) - 1) {
      line[idx++] = c;
    }
  }
  if (cmdUdpUp) {
    while (cmdUdp.parsePacket() > 0) {
      char pkt[200];
      int len = cmdUdp.read(pkt, sizeof(pkt) - 1);
      if (len <= 0) continue;
      pkt[len] = '\0';
      while (len > 0 && (pkt[len - 1] == '\n' || pkt[len - 1] == '\r')) pkt[--len] = '\0';
      if (len == 0) continue;
      if (strcmp(pkt, "hello") == 0) {   // gui.py WifiLink.register_peer()
        laptopIp = cmdUdp.remoteIP();
        continue;
      }
      handleLine(pkt);
    }
  }
#endif
}

void sendSensor(const float q[4], const int16_t a[3], const int16_t g[3],
                int16_t t) {
#if defined(BOARD_WROOM32)
  if (!SerialBT.hasClient() && !debugMode) return;   // nobody listening
#endif
  char buffer[200];
  buildTelemetry(buffer, sizeof(buffer), q, a, g, t);
  linkSendLine(buffer);
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
#if defined(BOARD_WROOM32)
  Serial.printf("dbg: acal stored=%d ox=%d oy=%d oz=%d | bt client=%d\n",
                prefs.getBool("aov", false) ? 1 : 0,
                prefs.getShort("aox", 0), prefs.getShort("aoy", 0),
                prefs.getShort("aoz", 0), SerialBT.hasClient() ? 1 : 0);
#else
  Serial.printf("dbg: acal stored=%d ox=%d oy=%d oz=%d | feed=%s\n",
                prefs.getBool("aov", false) ? 1 : 0,
                prefs.getShort("aox", 0), prefs.getShort("aoy", 0),
                prefs.getShort("aoz", 0), feedWifi ? "wifi" : "usb");
#endif
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
  Serial.printf("dbg wifi: state=%s pkts/s=%u rssi=%d -> %s:%d\n",
                st, videoSeq - lastSeq, wifiRssi,
                laptopIp.toString().c_str(), VIDEO_PORT);
  lastPrint = now;
  lastSeq = videoSeq;
}

#if defined(BOARD_WROOM32)
// WROOM-32 only: USB serial is purely a monitor there, so it accepts just the
// "debug" toggle. (On the S3 the USB serial IS the feed -- feedPoll() reads it
// and "debug" is a normal control line via handleLine().)
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
#endif


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
      linkPrintf("wifi:connected,%s", WiFi.localIP().toString().c_str());
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
  // Explicit I2C pins: the S3's Arduino defaults (SDA 8 / SCL 9) are camera
  // data pins on the S3-CAM, so relying on Wire.begin() there would fight the
  // camera bus. Explicit on both boards for symmetry.
  Wire.begin(PIN_SDA, PIN_SCL);
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

#if defined(BOARD_WROOM32)
  SerialBT.begin(BT_NAME);
  Serial.printf("Bluetooth SPP up as \"%s\"; provision Wi-Fi/video over it.\n", BT_NAME);
#else
  Serial.println("USB serial feed up (board s3cam); provision Wi-Fi over it, "
                 "then 'feed:wifi' to go wireless.");
#endif

  xTaskCreatePinnedToCore(imuTask, "imuTask", 4096, NULL, 3, &imuTaskHandle, 0);
}


void loop() {
  feedPoll();            // sensor commands + Wi-Fi provisioning (BT/USB/UDP)
#if defined(BOARD_WROOM32)
  usbPoll();             // USB serial monitor: "debug" toggle
#endif
  updateWifiVideo();     // Wi-Fi connect state machine + video frames
  if (debugMode) debugVideoTick();

  unsigned long currentTime = millis();

  // 1 Hz Wi-Fi signal sample for the rs telemetry field (kept off the 50 Hz
  // hot path; WiFi.RSSI() is only meaningful while the station is connected).
  static unsigned long lastRssiTime = 0;
  if (currentTime - lastRssiTime >= 1000) {
    lastRssiTime = currentTime;
    wifiRssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
  }

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

  sendSensor(q, a, g, t);
}
