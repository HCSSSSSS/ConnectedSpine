#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_DRV2605.h>
#include "config.h"
#include "packets.h"

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;
Adafruit_DRV2605 drv;

uint8_t broadcastAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
C3ToS3Packet sendPacket;
S3ToC3Packet recvPacket;
unsigned long lastSendTime = 0;

float curPitch = 0, curRoll = 0, curYaw = 0;
volatile bool vibrationRequested = false;

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("send: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(S3ToC3Packet)) return;
  memcpy(&recvPacket, data, sizeof(recvPacket));
  if (recvPacket.targetNodeId != NODE_ID && recvPacket.targetNodeId != 0) return;
  if (recvPacket.command == 1) {
    vibrationRequested = true;
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
  Serial.println("C3 ESP-NOW ready");
  uint8_t ch; wifi_second_chan_t sec;
  esp_wifi_get_channel(&ch, &sec);
  Serial.printf("C3 channel: %d\n", ch);
}

bool setupIMU() {
  Wire.begin(D4, D5);
  Wire.setClock(400000);
  delay(100);                      // 给 BNO085 上电稳定时间

  for (int attempt = 0; attempt < 5; attempt++) {
    if (bno08x.begin_I2C(0x4A, &Wire)) {
      Serial.println("BNO085 found at 0x4A");
      if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
        Serial.println("enableReport FAIL"); return false;
      }
      Serial.println("BNO085 game rotation vector enabled");
      return true;
    }
    if (bno08x.begin_I2C(0x4B, &Wire)) {
      Serial.println("BNO085 found at 0x4B");
      if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
        Serial.println("enableReport FAIL"); return false;
      }
      Serial.println("BNO085 game rotation vector enabled");
      return true;
    }
    Serial.printf("BNO085 init retry %d...\n", attempt+1);
    delay(200);
  }
  Serial.println("BNO085 not found after retries!");
  return false;
}

bool setupHaptic() {
  if (!drv.begin(&Wire)) {
    Serial.println("DRV2605L not found!");
    return false;
  }
  drv.selectLibrary(1);
  drv.useLRA();
  drv.setMode(DRV2605_MODE_INTTRIG);
  Serial.println("DRV2605L ready (LRA)");
  return true;
}

void doVibrate() {
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

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== C3 IMU+HAPTIC (GRV) ===");
  Serial.printf("NODE_ID=%d\n", NODE_ID);
  if (!setupIMU())    { while(1) delay(1000); }
  if (!setupHaptic()) { while(1) delay(1000); }
  setupESPNow();
}

void loop() {
  if (bno08x.wasReset()) bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      quatToEuler(sensorValue.un.gameRotationVector.i, sensorValue.un.gameRotationVector.j,
                  sensorValue.un.gameRotationVector.k, sensorValue.un.gameRotationVector.real,
                  curPitch, curRoll, curYaw);
    }
  }

  if (vibrationRequested) {
    vibrationRequested = false;
    Serial.printf("C3 N%d VIBRATE\n", NODE_ID);
    doVibrate();
  }

  if (millis() - lastSendTime >= SEND_INTERVAL_MS) {
    lastSendTime = millis();
    sendPacket.nodeId    = NODE_ID;
    sendPacket.pitch     = curPitch;
    sendPacket.roll      = curRoll;
    sendPacket.yaw       = curYaw;
    sendPacket.timestamp = millis();
    esp_now_send(broadcastAddress, (uint8_t*)&sendPacket, sizeof(sendPacket));
    Serial.printf("C3 N%d | P %.1f | R %.1f | Y %.1f\n", NODE_ID, curPitch, curRoll, curYaw);
  }
}