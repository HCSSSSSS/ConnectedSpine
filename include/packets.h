#ifndef PACKETS_H
#define PACKETS_H

#include <stdint.h>

// C3 -> S3：姿态数据包
typedef struct __attribute__((packed)) {
  uint8_t  nodeId;
  float    pitch;
  float    roll;
  float    yaw;
  uint32_t timestamp;
} C3ToS3Packet;

// S3 -> C3：命令包
typedef struct __attribute__((packed)) {
  uint8_t targetNodeId;   // 1/2/3=指定节点  0=全部
  uint8_t command;        // 1=VIBRATE
} S3ToC3Packet;

#endif