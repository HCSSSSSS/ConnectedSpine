#ifndef PACKETS_H
#define PACKETS_H

#include <stdint.h>

// ===== C3 -> S3：姿态数据包，保持 packed（避免 C3/S3 结构体对齐不一致）=====
typedef struct __attribute__((packed))
{
  uint8_t nodeId;
  float pitch;
  float roll;
  float yaw;
  uint32_t timestamp;
} C3ToS3Packet; // sizeof = 17

// ===== 消息类型：震动命令链路 =====
enum MsgType : uint8_t
{
  MSG_VIBRATE = 2, // S3 -> C3：震动命令
  MSG_VIB_ACK = 3  // C3 -> S3：命令收到 ACK（command-received，不是物理振动确认）
};

// ===== S3 -> C3：震动命令包，单播 =====
typedef struct __attribute__((packed))
{
  uint8_t type;         // MSG_VIBRATE
  uint8_t targetNodeId; // 1/2/3
  uint32_t msgId;       // ACK 匹配 + 去重
} VibratePacket;        // sizeof = 6

// ===== C3 -> S3：震动 ACK 包，单播 =====
typedef struct __attribute__((packed))
{
  uint8_t type;   // MSG_VIB_ACK
  uint8_t nodeId; // 1/2/3
  uint32_t msgId; // 对应 VibratePacket 的 msgId
} AckPacket;      // sizeof = 6

#endif