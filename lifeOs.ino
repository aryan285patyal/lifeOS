// lifeOs.ino  --  ESP32 + MPU6050 (DMP) -> telemetry over a swappable link
//
// The MPU6050's on-chip DMP produces a fused orientation quaternion; we also
// forward the raw accel/gyro counts and the current servo angles so the PC-side
// GUI (Monitor / Visualizer / Servos tabs) all work off one stream.
//
// TRANSPORT: set USE_BLUETOOTH below. Both transports carry the SAME newline-
// delimited ASCII protocol, so the PC side only changes where it reads bytes:
//   * 1 = Bluetooth Classic SPP  -> appears as a COM port on the laptop; keeps
//         the laptop's WiFi/internet free and ignores the local network (no
//         router, no client isolation, no mDNS needed).
//   * 0 = WiFi / UDP            -> mDNS discovery + two-way UDP (needs a network
//         that allows peer-to-peer traffic).
// The link is isolated behind linkBegin()/linkConnected()/linkSend()/linkPoll()
// so loop() and the IMU task are transport-agnostic (this is the seam the PC's
// connection-method selector will mirror).
//
// Concurrency (ESP-WROOM-32, dual core):
//   * IMU task pinned to core 0 drains the DMP FIFO promptly (interrupt-driven
//     on GPIO4) and publishes the latest sample into a spinlock-guarded struct.
//   * The Arduino loop() (core 1) reads that struct and transmits over the link.
//
// Requires the "MPU6050" library by Electronic Cats (bundles I2Cdev +
// MPU6050_6Axis_MotionApps20). WiFi mode also needs WIFI_SSID/WIFI_PASSWORD in
// secrets.h. Bluetooth mode needs the Arduino IDE "Tools > Partition Scheme" set
// to one with room for the BT stack + DMP blob (e.g. "Huge APP" / "Minimal
// SPIFFS"); the default partition may overflow.

#define USE_BLUETOOTH 1              // 1 = Bluetooth SPP link, 0 = WiFi/UDP link

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <ESP32Servo.h>

#if USE_BLUETOOTH
  #include "BluetoothSerial.h"
  BluetoothSerial SerialBT;
  const char* BT_NAME = "lifeos";    // name the laptop pairs with
#else
  #include <WiFi.h>
  #include <WiFiUdp.h>
  #include <ESPmDNS.h>
  #include "secrets.h"               // WIFI_SSID, WIFI_PASSWORD
  const char* MDNS_NAME = "lifeos";  // claims lifeos.local + _lifeos._udp service
  const int   UDP_PORT  = 5005;      // telemetry TX (ESP32 -> PC)
  const int   CMD_PORT  = 5006;      // servo commands + PC "hello" RX (PC -> ESP32)
  // Learned at runtime from the first packet the PC sends us; nothing hardcoded.
  IPAddress   pcIp;
  bool        havePc = false;
  WiFiUDP     udp;                    // telemetry out
  WiFiUDP     cmdUdp;                 // commands in
#endif

#define INTERRUPT_PIN 4              // MPU6050 INT -> ESP32 GPIO4
const long  interval = 20;           // telemetry TX period, ms (50 Hz)

// --- Servos: GPIOs clear of the MPU (I2C 21/22, INT 4), flash, strapping and
// input-only pins. Signal wire only; power servos from a separate 5-6V supply
// sharing ground with the ESP32. ---
#define NUM_SERVOS 2
const int SERVO_PINS[NUM_SERVOS] = {13, 25};
Servo servos[NUM_SERVOS];
int   servoPos[NUM_SERVOS] = {90, 90};   // last commanded angle; echoed in telemetry

MPU6050 mpu;

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
// Pinned to core 0 so it never waits behind the TX/radio work on core 1.
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


// ---------------------------------------------------------------------------
// Transport-agnostic helpers (shared by both links)
// ---------------------------------------------------------------------------

// Format one telemetry line into buffer.
void buildTelemetry(char *buffer, size_t n,
                    const float q[4], const int16_t a[3], const int16_t g[3]) {
  snprintf(buffer, n,
           "q0:%.4f,q1:%.4f,q2:%.4f,q3:%.4f,ax:%d,ay:%d,az:%d,gx:%d,gy:%d,gz:%d,s0:%d,s1:%d",
           q[0], q[1], q[2], q[3], a[0], a[1], a[2], g[0], g[1], g[2],
           servoPos[0], servoPos[1]);
}

// Apply a comma-separated servo command like "s0:90,s1:45" (clamped 0-180).
// A non-matching line (e.g. the WiFi "hello") is simply ignored.
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


// ---------------------------------------------------------------------------
// Link layer: one implementation per transport, same four-function contract.
// ---------------------------------------------------------------------------
#if USE_BLUETOOTH

void linkBegin() {
  SerialBT.begin(BT_NAME);
  Serial.printf("Bluetooth SPP up as \"%s\" - pair from the laptop, then open its COM port\n",
                BT_NAME);
}

bool linkConnected() { return SerialBT.hasClient(); }

void linkSend(const float q[4], const int16_t a[3], const int16_t g[3]) {
  if (!SerialBT.hasClient()) return;   // no paired client -> stay silent
  char buffer[200];
  buildTelemetry(buffer, sizeof(buffer), q, a, g);
  SerialBT.println(buffer);            // newline-delimited line
}

// Accumulate incoming bytes into lines and apply each complete command.
void linkPoll() {
  static char line[128];
  static size_t idx = 0;
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c == '\n' || c == '\r') {
      if (idx > 0) { line[idx] = '\0'; applyServoCommand(line); idx = 0; }
    } else if (idx < sizeof(line) - 1) {
      line[idx++] = c;
    }
  }
}

#else   // ---- WiFi / UDP ----

void linkBegin() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
  udp.begin(UDP_PORT);
  cmdUdp.begin(CMD_PORT);            // listen for the PC hello + servo commands

  if (MDNS.begin(MDNS_NAME)) {
    MDNS.addService("lifeos", "udp", CMD_PORT);
    Serial.printf("mDNS: %s.local advertising _lifeos._udp on %d\n", MDNS_NAME, CMD_PORT);
  } else {
    Serial.println("mDNS start failed");
  }
}

bool linkConnected() { return havePc; }

void linkSend(const float q[4], const int16_t a[3], const int16_t g[3]) {
  if (!havePc) return;               // no PC has connected yet -> stay silent
  char buffer[200];
  buildTelemetry(buffer, sizeof(buffer), q, a, g);
  udp.beginPacket(pcIp, UDP_PORT);
  udp.write((uint8_t*)buffer, strlen(buffer));
  udp.endPacket();
}

// Drain any pending datagram on CMD_PORT: the sender becomes our PC (so
// telemetry follows the laptop's current IP) and any servo command is applied.
void linkPoll() {
  int n = cmdUdp.parsePacket();
  if (n <= 0) return;

  IPAddress from = cmdUdp.remoteIP();
  if (!havePc || from != pcIp) {
    Serial.print("PC connected, streaming telemetry to ");
    Serial.println(from);
  }
  pcIp   = from;
  havePc = true;

  char buf[128];
  int len = cmdUdp.read(buf, sizeof(buf) - 1);
  if (len <= 0) return;
  buf[len] = '\0';
  applyServoCommand(buf);
}

#endif


void setup() {
  Wire.begin();
  Wire.setClock(400000);             // 400 kHz I2C for prompt FIFO reads
  Serial.begin(115200);

  // ESP32Servo shares the LEDC timers; reserve them before attaching.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].setPeriodHertz(50);              // standard 50 Hz servo frame
    servos[i].attach(SERVO_PINS[i], 500, 2400); // 0.5-2.4 ms pulse range
    servos[i].write(servoPos[i]);              // park at center on boot
  }

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

  linkBegin();                       // bring up the selected transport

  // IMU on core 0; loop()/radio run on core 1.
  xTaskCreatePinnedToCore(imuTask, "imuTask", 4096, NULL, 3, &imuTaskHandle, 0);
}


void loop() {
  linkPoll();                        // apply any pending command every loop

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

  linkSend(q, a, g);
}
