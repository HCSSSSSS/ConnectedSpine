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

// ===== 填入 Step 0 抄到的各 C3 的 STA MAC =====
static const uint8_t MAC_NODE1[6] = {0x1C,0xDB,0xD4,0xEC,0x8B,0x84}; // Head
static const uint8_t MAC_NODE2[6] = {0x1C,0xDB,0xD4,0xEA,0x9B,0xC4}; // WaistL
static const uint8_t MAC_NODE3[6] = {0x1C,0xDB,0xD4,0xEA,0xC9,0x7C}; // WaistR
#define VIB_RETRY_INTERVAL_MS 40
#define VIB_MAX_RETRIES 6

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;
Adafruit_DRV2605 drv;

// 各节点最新绝对姿态  0=Chest 1=Head 2=WaistL 3=WaistR
volatile float curP[4] = {0}, curR[4] = {0}, curY[4] = {0};
volatile uint32_t lastSeen[4] = {0};
volatile bool online[4] = {false, false, false, false};

float zeroP[4] = {0}, zeroR[4] = {0}, zeroY[4] = {0};
bool zeroSaved[4] = {false, false, false, false};

// 分轴滞回状态
bool pitchBad[4] = {false, false, false, false};
bool rollBad[4] = {false, false, false, false};
unsigned long lastVibrate[4] = {0, 0, 0, 0};

// 校准（纯按钮触发）
bool calibrated = false;
unsigned long lastPrint = 0;

// 按钮去抖
int lastButtonState = HIGH;
int lastButtonReading = HIGH;
unsigned long lastDebounceTime = 0;

// ===== 震动命令：单播 + ACK + 重传 =====
struct PendingVib
{
  bool active;
  uint32_t msgId;
  uint8_t retries;
  uint32_t lastTx;
};
PendingVib pend[4] = {};
uint32_t vibSeq = 0;
// ACK 打印挪出回调（回调只置标志，打印在 loop）
volatile bool ackPrintPending[4] = {false, false, false, false};
volatile uint8_t ackPrintRetries[4] = {0, 0, 0, 0};

// ===== RX drop 调试（回调只记录，loop 打印）=====
// Step A 验证通过后，删除这三个变量 + onDataRecv 末尾三行 + loop 里对应 block
volatile bool rxDropPrintPending = false;
volatile int rxDropLen = 0;
volatile int rxDropHead = -1;

const uint8_t *macOf(int n)
{
  switch (n)
  {
  case 1:
    return MAC_NODE1;
  case 2:
    return MAC_NODE2;
  case 3:
    return MAC_NODE3;
  }
  return nullptr;
}

const char *nodeName(int i)
{
  switch (i)
  {
  case 0:
    return "Chest";
  case 1:
    return "Head";
  case 2:
    return "WaistL";
  case 3:
    return "WaistR";
  }
  return "?";
}

// 节点是否活跃：Chest 始终在线；C3 需在线且数据新鲜
bool nodeActive(int i)
{
  if (i == 0)
    return true;
  return online[i] && (millis() - lastSeen[i] <= NODE_TIMEOUT_MS);
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
  // IMU 包：广播（逻辑同原版）
  if (len == sizeof(C3ToS3Packet))
  {
    C3ToS3Packet pkt;
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.nodeId >= 1 && pkt.nodeId <= 3)
    {
      curP[pkt.nodeId] = pkt.pitch;
      curR[pkt.nodeId] = pkt.roll;
      curY[pkt.nodeId] = pkt.yaw;
      lastSeen[pkt.nodeId] = millis();
      online[pkt.nodeId] = true;
    }
    return;
  }
  // 震动 ACK：单播 —— 回调里只清状态 + 置打印标志，不做串口输出
  if (len == sizeof(AckPacket) && data[0] == MSG_VIB_ACK)
  {
    AckPacket a;
    memcpy(&a, data, sizeof(a));
    if (a.nodeId >= 1 && a.nodeId <= 3 &&
        pend[a.nodeId].active && a.msgId == pend[a.nodeId].msgId)
    {
      ackPrintRetries[a.nodeId] = pend[a.nodeId].retries;
      ackPrintPending[a.nodeId] = true;
      pend[a.nodeId].active = false;
    }
    return;
  }
  // 调试兜底：回调里只记录，不打印（标志最后置，保证 loop 读到时另两个值已写好）
  rxDropLen = len;
  rxDropHead = (len > 0) ? data[0] : -1;
  rxDropPrintPending = true;
}

void addPeer(const uint8_t mac[6])
{
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK)
    Serial.println("add peer FAIL");
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
  addPeer(MAC_NODE1);
  addPeer(MAC_NODE2);
  addPeer(MAC_NODE3);
  Serial.println("S3 ESP-NOW ready (IMU bcast in / vibrate unicast out)");
}

void startVibrate(int n)
{
  vibSeq++;
  VibratePacket p{(uint8_t)MSG_VIBRATE, (uint8_t)n, vibSeq};
  esp_now_send(macOf(n), (uint8_t *)&p, sizeof(p));
  pend[n] = {true, vibSeq, 0, millis()};
}

void serviceVibrate()
{ // loop 每轮都调，不受打印间隔限制
  for (int n = 1; n <= 3; n++)
  {
    if (!pend[n].active)
      continue;
    if (millis() - pend[n].lastTx < VIB_RETRY_INTERVAL_MS)
      continue;
    if (pend[n].retries >= VIB_MAX_RETRIES)
    {
      Serial.printf("N%d vibrate: no ACK, give up\n", n);
      pend[n].active = false;
      continue;
    }
    VibratePacket p{(uint8_t)MSG_VIBRATE, (uint8_t)n, pend[n].msgId};
    esp_now_send(macOf(n), (uint8_t *)&p, sizeof(p));
    pend[n].retries++;
    pend[n].lastTx = millis();
  }
}

bool setupIMU()
{
  Wire.begin(D4, D5);
  Wire.setClock(400000);
  delay(100);
  for (int attempt = 0; attempt < 5; attempt++)
  {
    if (bno08x.begin_I2C(0x4A, &Wire) || bno08x.begin_I2C(0x4B, &Wire))
    {
      Serial.println("Chest BNO085 found");
      if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR))
      {
        Serial.println("enableReport FAIL");
        return false;
      }
      Serial.println("Chest BNO085 game rotation vector enabled");
      return true;
    }
    Serial.printf("Chest BNO085 init retry %d...\n", attempt + 1);
    delay(200);
  }
  Serial.println("Chest BNO085 not found after retries!");
  return false;
}

bool setupHaptic()
{
  if (!drv.begin(&Wire))
  {
    Serial.println("Chest DRV2605L not found!");
    return false;
  }
  drv.selectLibrary(1);
  drv.useLRA();
  drv.setMode(DRV2605_MODE_INTTRIG);
  Serial.println("Chest DRV2605L ready (LRA)");
  return true;
}

void doLocalVibrate()
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

float angleDiff(float current, float zero)
{
  float diff = current - zero;
  if (diff > 180.0f)
    diff -= 360.0f;
  else if (diff < -180.0f)
    diff += 360.0f;
  return diff;
}

void getThresholds(int i, float &pBad, float &pRec, float &rBad, float &rRec)
{
  if (i == 1)
  { // Head
    pBad = HEAD_PITCH_BAD;
    pRec = HEAD_PITCH_RECOVER;
    rBad = HEAD_ROLL_BAD;
    rRec = HEAD_ROLL_RECOVER;
  }
  else if (i == 0)
  { // Chest
    pBad = CHEST_PITCH_BAD;
    pRec = CHEST_PITCH_RECOVER;
    rBad = CHEST_ROLL_BAD;
    rRec = CHEST_ROLL_RECOVER;
  }
  else
  { // Waist
    pBad = WAIST_PITCH_BAD;
    pRec = WAIST_PITCH_RECOVER;
    rBad = WAIST_ROLL_BAD;
    rRec = WAIST_ROLL_RECOVER;
  }
}

void doCalibration()
{
  for (int i = 0; i <= 3; i++)
  {
    if (nodeActive(i))
    {
      zeroP[i] = curP[i];
      zeroR[i] = curR[i];
      zeroY[i] = curY[i];
      zeroSaved[i] = true;
      pitchBad[i] = false;
      rollBad[i] = false;
      lastVibrate[i] = millis();
      Serial.printf("%s zero saved | P %.1f R %.1f Y %.1f\n",
                    nodeName(i), curP[i], curR[i], curY[i]);
    }
    else
    {
      zeroSaved[i] = false;
      pitchBad[i] = false;
      rollBad[i] = false;
      Serial.printf("%s offline or stale, zero skipped\n", nodeName(i));
    }
  }
  Serial.println("=== Calibration complete ===");
}

void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== S3 FULL SYSTEM (GRV) ===");
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
  pinMode(CAL_BUTTON_PIN, INPUT_PULLUP);
  online[0] = true;
  Serial.println("Waiting for button calibration...");
}

void loop()
{
  // ===== 震动命令重传服务（每轮都跑）=====
  serviceVibrate();

  // ===== ACK 打印（从回调挪出）=====
  for (int n = 1; n <= 3; n++)
  {
    if (ackPrintPending[n])
    {
      ackPrintPending[n] = false;
      Serial.printf("N%d vibrate ACK (after %d retr.)\n", n, ackPrintRetries[n]);
    }
  }

  // ===== RX drop 调试打印（验证通过后删除此 block）=====
  if (rxDropPrintPending)
  {
    rxDropPrintPending = false;
    Serial.printf("RX drop: len=%d head=%d\n", rxDropLen, rxDropHead);
  }

  // ===== 按钮检测：按下校准 =====
  int reading = digitalRead(CAL_BUTTON_PIN);
  if (reading != lastButtonReading)
    lastDebounceTime = millis();
  if (millis() - lastDebounceTime > BUTTON_DEBOUNCE_MS)
  {
    if (reading != lastButtonState)
    {
      lastButtonState = reading;
      if (lastButtonState == LOW)
      {
        Serial.println(">>> BUTTON: Calibrating...");
        doCalibration();
        calibrated = true;
      }
    }
  }
  lastButtonReading = reading;

  // ===== 读胸口 IMU =====
  if (bno08x.wasReset())
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
  if (bno08x.getSensorEvent(&sensorValue))
  {
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR)
    {
      float p, r, y;
      quatToEuler(sensorValue.un.gameRotationVector.i, sensorValue.un.gameRotationVector.j,
                  sensorValue.un.gameRotationVector.k, sensorValue.un.gameRotationVector.real, p, r, y);
      curP[0] = p;
      curR[0] = r;
      curY[0] = y;
    }
  }

  // ===== 定时打印 / 判断 =====
  if (millis() - lastPrint >= JUDGE_INTERVAL_MS)
  {
    lastPrint = millis();
    if (!calibrated)
    {
      Serial.printf("[waiting] Press button to calibrate | Chest P %.1f R %.1f Y %.1f | N1:%d N2:%d N3:%d\n",
                    curP[0], curR[0], curY[0],
                    nodeActive(1), nodeActive(2), nodeActive(3));
    }
    else
    {
      for (int i = 0; i <= 3; i++)
      {
        if (zeroSaved[i] && nodeActive(i))
        {
          float dP = angleDiff(curP[i], zeroP[i]);
          float dR = angleDiff(curR[i], zeroR[i]);
          float dY = angleDiff(curY[i], zeroY[i]);

          float pBad, pRec, rBad, rRec;
          getThresholds(i, pBad, pRec, rBad, rRec);

          if (!pitchBad[i] && fabs(dP) > pBad)
            pitchBad[i] = true;
          else if (pitchBad[i] && fabs(dP) < pRec)
            pitchBad[i] = false;
          if (!rollBad[i] && fabs(dR) > rBad)
            rollBad[i] = true;
          else if (rollBad[i] && fabs(dR) < rRec)
            rollBad[i] = false;

          bool nodeIsBad = pitchBad[i] || rollBad[i];

          Serial.printf("%s | dP %6.1f | dR %6.1f | dY %6.1f | %s%s\n",
                        nodeName(i), dP, dR, dY,
                        nodeIsBad ? "BAD" : "NORMAL",
                        nodeIsBad ? (pitchBad[i] && rollBad[i] ? " (P+R)" : pitchBad[i] ? " (P)"
                                                                                        : " (R)")
                                  : "");

          if (nodeIsBad && (millis() - lastVibrate[i] >= VIBRATION_COOLDOWN_MS))
          {
            if (i == 0)
            {
              lastVibrate[i] = millis();
              Serial.println(">>> Chest VIBRATE (local)");
              doLocalVibrate();
            }
            else if (!pend[i].active)
            { // 上一条还没确认就不叠新的
              lastVibrate[i] = millis();
              Serial.printf(">>> CMD VIBRATE target=%d\n", i);
              startVibrate(i);
            }
          }
        }
        else if (zeroSaved[i] && !nodeActive(i))
        {
          Serial.printf("%s | (offline)\n", nodeName(i));
          pitchBad[i] = false;
          rollBad[i] = false;
        }
      }
      Serial.println("----");
    }
  }
}