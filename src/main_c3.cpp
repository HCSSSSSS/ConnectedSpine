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

// ===== 填入 Step 0 抄到的 S3 的 STA MAC =====
static const uint8_t MAC_S3[6] = {0x90,0x70,0x69,0x12,0xE9,0xD0};    // S3 主节点

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;
Adafruit_DRV2605 drv;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
C3ToS3Packet sendPacket; // IMU 包（广播，保持原样）
unsigned long lastSendTime = 0;

float curPitch = 0, curRoll = 0, curYaw = 0;

// 震动命令：收命令置标志，回 ACK 也置标志，均在 loop 处理（回调不做串口/马达操作）
volatile bool vibrationRequested = false;
volatile bool ackPending = false;
volatile uint32_t pendingAckId = 0;
uint32_t lastMsgId = 0;

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
  // 只认发给本节点的震动命令；顺便滤掉串到的广播 IMU 包
  if (len != sizeof(VibratePacket) || data[0] != MSG_VIBRATE)
    return;
  VibratePacket p;
  memcpy(&p, data, sizeof(p));
  if (p.targetNodeId != NODE_ID)
    return;
  ackPending = true;
  pendingAckId = p.msgId; // 每个副本都回 ACK，让 S3 停止重发
  if (p.msgId != lastMsgId)
  { // 同一条只震一次
    lastMsgId = p.msgId;
    vibrationRequested = true;
  }
}

void setupESPNow()
{
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // 关 modem sleep，降低丢包
  WiFi.disconnect();
  delay(100);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("espnow init FAIL");
    while (1)
      delay(1000);
  }
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t bc = {}; // 广播 peer：发 IMU
  memcpy(bc.peer_addr, broadcastAddress, 6);
  bc.channel = ESPNOW_CHANNEL;
  bc.encrypt = false;
  if (esp_now_add_peer(&bc) != ESP_OK)
    Serial.println("bcast peer FAIL");

  esp_now_peer_info_t s3 = {}; // 单播 peer：回 ACK 给 S3
  memcpy(s3.peer_addr, MAC_S3, 6);
  s3.channel = ESPNOW_CHANNEL;
  s3.encrypt = false;
  if (esp_now_add_peer(&s3) != ESP_OK)
    Serial.println("s3 peer FAIL");

  Serial.println("C3 ESP-NOW ready (IMU bcast / ACK unicast)");
  uint8_t ch;
  wifi_second_chan_t sec;
  esp_wifi_get_channel(&ch, &sec);
  Serial.printf("C3 channel: %d\n", ch);
}

bool setupIMU()
{
  Wire.begin(D4, D5);
  Wire.setClock(400000);
  delay(100); // 给 BNO085 上电稳定时间
  for (int attempt = 0; attempt < 5; attempt++)
  {
    if (bno08x.begin_I2C(0x4A, &Wire) || bno08x.begin_I2C(0x4B, &Wire))
    {
      Serial.println("BNO085 found");
      if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR))
      {
        Serial.println("enableReport FAIL");
        return false;
      }
      Serial.println("BNO085 game rotation vector enabled");
      return true;
    }
    Serial.printf("BNO085 init retry %d...\n", attempt + 1);
    delay(200);
  }
  Serial.println("BNO085 not found after retries!");
  return false;
}

bool setupHaptic()
{
  if (!drv.begin(&Wire))
  {
    Serial.println("DRV2605L not found!");
    return false;
  }
  drv.selectLibrary(1);
  drv.useLRA();
  drv.setMode(DRV2605_MODE_INTTRIG);
  Serial.println("DRV2605L ready (LRA)");
  return true;
}

void doVibrate()
{
  drv.setWaveform(0, 47);
  drv.setWaveform(1, 0);
  drv.go();
}

void quatToEuler(float qi, float qj, float qk, float qr,
                 float &pitch, float &roll, float &yaw) {
  // 设备"上"方向（重力）在机体坐标系下的分量
  float gx = 2.0f * (qi*qk - qr*qj);
  float gy = 2.0f * (qr*qi + qj*qk);
  float gz = qr*qr - qi*qi - qj*qj + qk*qk;
  // 两个分量都用 atan2(_, sqrt(...))：范围 ±90°，全程连续，无奇点
  // 平放时 gx=gy=0 → pitch=roll=0；放平必回 0，BAD 一定能解除
  pitch = atan2f(gx, sqrtf(gy*gy + gz*gz)) * 180.0f / PI;   // 前后倾
  roll  = atan2f(gy, sqrtf(gx*gx + gz*gz)) * 180.0f / PI;   // 左右倾
  yaw   = 0.0f;   // 重力法给不出偏航；你本来就没判 yaw，置 0 即可
}

void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== C3 IMU+HAPTIC (GRV) ===");
  Serial.printf("NODE_ID=%d\n", NODE_ID);
  if (!setupIMU())
  {
    while (1)
      delay(1000);
  }
  if (!setupHaptic())
  {
    while (1)
      delay(1000);
  }
  setupESPNow();
}

void loop()
{
  // ===== 回 ACK（从回调挪出）=====
  if (ackPending)
  {
    ackPending = false;
    AckPacket a{(uint8_t)MSG_VIB_ACK, (uint8_t)NODE_ID, pendingAckId};
    esp_now_send(MAC_S3, (uint8_t *)&a, sizeof(a));
  }

  // ===== 执行震动 =====
  if (vibrationRequested)
  {
    vibrationRequested = false;
    Serial.printf("C3 N%d VIBRATE\n", NODE_ID);
    doVibrate();
  }

  // ===== 读 IMU =====
  if (bno08x.wasReset())
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
  if (bno08x.getSensorEvent(&sensorValue))
  {
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR)
    {
      quatToEuler(sensorValue.un.gameRotationVector.i, sensorValue.un.gameRotationVector.j,
                  sensorValue.un.gameRotationVector.k, sensorValue.un.gameRotationVector.real,
                  curPitch, curRoll, curYaw);
    }
  }

  // ===== 周期广播 IMU（保持原样）=====
  if (millis() - lastSendTime >= IMU_SEND_INTERVAL_MS)
  {
    lastSendTime = millis();
    sendPacket.nodeId = NODE_ID;
    sendPacket.pitch = curPitch;
    sendPacket.roll = curRoll;
    sendPacket.yaw = curYaw;
    sendPacket.timestamp = millis();
    esp_now_send(broadcastAddress, (uint8_t *)&sendPacket, sizeof(sendPacket));
    Serial.printf("C3 N%d | P %.1f | R %.1f | Y %.1f\n", NODE_ID, curPitch, curRoll, curYaw);
  }
}