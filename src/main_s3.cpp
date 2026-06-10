#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_DRV2605.h>
#include "config.h"
#include "packets.h"

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;
Adafruit_DRV2605 drv;

uint8_t broadcastAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
S3ToC3Packet sendPacket;

// ---- 各节点最新绝对姿态 ----
// 索引 0=Chest(S3自己)  1=Node1  2=Node2  3=Node3
volatile float curP[4] = {0}, curR[4] = {0}, curY[4] = {0};
volatile uint32_t lastSeen[4] = {0};   // 最近收到时间(对C3)；Chest一直在线
volatile bool online[4] = {false,false,false,false};

// ---- 零点参考 ----
float zeroP[4] = {0}, zeroR[4] = {0}, zeroY[4] = {0};
bool  zeroSaved[4] = {false,false,false,false};

unsigned long lastVibrate[4] = {0,0,0,0};   // 每节点独立 cooldown 时间戳
bool nodeBad[4] = {false,false,false,false};   // 每节点当前姿态状态（滞回用）

// ---- 校准状态 ----
#define CALIB_DELAY_MS 8000
bool calibrated = false;
unsigned long bootTime = 0;
unsigned long lastPrint = 0;

const char* nodeName(int i) {
  switch(i){ case 0:return "Chest"; case 1:return "Node1";
             case 2:return "Node2"; case 3:return "Node3"; }
  return "?";
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(C3ToS3Packet)) return;
  C3ToS3Packet pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.nodeId >= 1 && pkt.nodeId <= 3) {
    curP[pkt.nodeId]=pkt.pitch; curR[pkt.nodeId]=pkt.roll; curY[pkt.nodeId]=pkt.yaw;
    lastSeen[pkt.nodeId]=millis();
    online[pkt.nodeId]=true;
  }
}

void setupESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  if (esp_now_init() != ESP_OK) { Serial.println("espnow init FAIL"); while(1) delay(1000); }
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastAddress, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) { Serial.println("add peer FAIL"); while(1) delay(1000); }
  Serial.println("S3 ESP-NOW ready");
}

bool setupIMU() {
  Wire.begin(D4, D5);
  Wire.setClock(400000);
  if (bno08x.begin_I2C(0x4A, &Wire))      Serial.println("Chest BNO085 at 0x4A");
  else if (bno08x.begin_I2C(0x4B, &Wire)) Serial.println("Chest BNO085 at 0x4B");
  else { Serial.println("Chest BNO085 not found!"); return false; }
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) { Serial.println("enableReport FAIL"); return false; }
  Serial.println("Chest BNO085 enabled");
  return true;
}

bool setupHaptic() {
  if (!drv.begin(&Wire)) { Serial.println("Chest DRV2605L not found!"); return false; }
  drv.selectLibrary(1);
  drv.useLRA();
  drv.setMode(DRV2605_MODE_INTTRIG);
  Serial.println("Chest DRV2605L ready (LRA)");
  return true;
}

void doLocalVibrate() {
  drv.setWaveform(0, 47);
  drv.setWaveform(1, 0);
  drv.go();
}

void quatToEuler(float qi, float qj, float qk, float qr,
                 float &pitch, float &roll, float &yaw) {
  float sqr=qr*qr, sqi=qi*qi, sqj=qj*qj, sqk=qk*qk;
  roll  = atan2(2.0f*(qi*qj+qk*qr),(sqi-sqj-sqk+sqr))*180.0f/PI;
  pitch = asin (-2.0f*(qi*qk-qj*qr)/(sqi+sqj+sqk+sqr))*180.0f/PI;
  yaw   = atan2(2.0f*(qj*qk+qi*qr),(-sqi-sqj+sqk+sqr))*180.0f/PI;
}

// yaw 环形差值归一化
float angleDiff(float current, float zero) {
  float diff = current - zero;
  if (diff > 180.0f)  diff -= 360.0f;
  else if (diff < -180.0f) diff += 360.0f;
  return diff;
}

void doCalibration() {
  for (int i = 0; i <= 3; i++) {
    // Chest(0) 始终在线；C3 节点只对收到过数据的存零点
    if (i == 0 || online[i]) {
      zeroP[i] = curP[i];
      zeroR[i] = curR[i];
      zeroY[i] = curY[i];
      zeroSaved[i] = true;
      Serial.printf("%s zero saved | P %.1f R %.1f Y %.1f\n",
                    nodeName(i), curP[i], curR[i], curY[i]);
    } else {
      Serial.printf("%s offline, zero skipped\n", nodeName(i));
    }
  }
  Serial.println("=== Calibration complete ===");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== S3 CALIBRATION TEST (PIO) ===");
  if (!setupIMU())    { while(1) delay(1000); }
  if (!setupHaptic()) { while(1) delay(1000); }   // 胸口无DRV则改成 setupHaptic();
  setupESPNow();
  online[0] = true;           // Chest 自己始终在线
  bootTime = millis();
  Serial.printf("[warmup] Chest P %.1f R %.1f Y %.1f | N1:%d N2:%d N3:%d\n",
              curP[0], curR[0], curY[0],
              online[1], online[2], online[3]);
}

void loop() {
  // 1) 读胸口 IMU（存到索引0）
  if (bno08x.wasReset()) bno08x.enableReport(SH2_ROTATION_VECTOR);
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float p,r,y;
      quatToEuler(sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j,
                  sensorValue.un.rotationVector.k, sensorValue.un.rotationVector.real, p,r,y);
      curP[0]=p; curR[0]=r; curY[0]=y;
    }
  }

  // 2) 8 秒到点，执行一次校准
  if (!calibrated && (millis() - bootTime >= CALIB_DELAY_MS)) {
    doCalibration();
    calibrated = true;
  }

  // 3) 定时打印
  if (millis() - lastPrint >= SEND_INTERVAL_MS) {
    lastPrint = millis();
    if (!calibrated) {
      // 校准前：打印绝对值，方便看 IMU 是否收敛稳定
      Serial.printf("[warmup] Chest P %.1f R %.1f Y %.1f\n", curP[0], curR[0], curY[0]);
    } else {
      for (int i = 0; i <= 3; i++) {
        if (zeroSaved[i]) {
          float dP = angleDiff(curP[i], zeroP[i]);
          float dR = angleDiff(curR[i], zeroR[i]);
          float dY = angleDiff(curY[i], zeroY[i]);

          // 取 pitch/roll 里偏差更大的那个做滞回判断
          float maxDev = fmax(fabs(dP), fabs(dR));

          // 滞回：NORMAL->BAD 用 20，BAD->NORMAL 用 15，中间保持
          if (!nodeBad[i] && maxDev > BAD_THRESHOLD) {
            nodeBad[i] = true;
          } else if (nodeBad[i] && maxDev < RECOVER_THRESHOLD) {
            nodeBad[i] = false;
          }
          // 15~20 之间：nodeBad[i] 保持不变

          Serial.printf("%s | dP %6.1f | dR %6.1f | dY %6.1f | %s\n",
                        nodeName(i), dP, dR, dY,
                        nodeBad[i] ? "BAD" : "NORMAL");

          if (nodeBad[i] && (millis() - lastVibrate[i] >= VIBRATION_COOLDOWN_MS)) {
            lastVibrate[i] = millis();
            if (i == 0) {
              Serial.println(">>> Chest VIBRATE (local)");
              doLocalVibrate();
            } else {
              Serial.printf(">>> CMD VIBRATE target=%d\n", i);
              sendPacket.targetNodeId = i;
              sendPacket.command = 1;
              esp_now_send(broadcastAddress, (uint8_t*)&sendPacket, sizeof(sendPacket));
            }
          }
        }
      }
      Serial.println("----");
    }
  }
}