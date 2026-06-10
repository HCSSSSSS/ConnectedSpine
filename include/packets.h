#ifndef PACKETS_H
#define PACKETS_H

#include <stdint.h>

// C3 -> S3：姿态数据包
typedef struct __attribute__((packed)) {
  uint8_t  nodeId;
  float    pitch;
  float    roll;
  float    yaw;
  uint32_t timestamp;   // millis()，用来观察刷新/丢包
} C3ToS3Packet;

// S3 -> C3：命令包（暂未使用，保留）
typedef struct __attribute__((packed)) {
  uint8_t targetNodeId;
  uint8_t command;
} S3ToC3Packet;

#endif