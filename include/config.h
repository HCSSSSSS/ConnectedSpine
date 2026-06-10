#ifndef CONFIG_H
#define CONFIG_H

// 烧录 C3 时改这里：1=HEAD  2=LEFT_WAIST  3=RIGHT_WAIST
// 烧录 S3 时这个值不影响（S3 是主节点）
#define NODE_ID 1

#define ESPNOW_CHANNEL 1      // 两端必须一致
#define SEND_INTERVAL_MS 1000 // C3 发送间隔
#define CMD_INTERVAL_MS 3000  // S3 发命令间隔
#define BAD_THRESHOLD     20.0f
#define RECOVER_THRESHOLD 15.0f

#define VIBRATION_COOLDOWN_MS 2000

#endif