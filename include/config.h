#ifndef CONFIG_H
#define CONFIG_H

// ===== 节点标识 =====
// 烧录 C3 时改这里：1=HEAD  2=LEFT_WAIST  3=RIGHT_WAIST
// 烧录 S3 时此值不影响（S3 是主节点/胸口）
#define NODE_ID 1

// 节点掉线判定：超过此时间没收到数据视为离线
#define NODE_TIMEOUT_MS  3000

// ===== 通信 =====
#define ESPNOW_CHANNEL    1       // 两端必须一致
#define SEND_INTERVAL_MS  1000    // C3 发送间隔 / S3 判断打印间隔

// ===== 震动 =====
#define VIBRATION_COOLDOWN_MS 2000  // 同一节点最短震动间隔

// ===== 校准按钮（胸口 S3，D7 接 GND，内部上拉，按下读 LOW）=====
#define CAL_BUTTON_PIN     D7
#define BUTTON_DEBOUNCE_MS 50

// ===== 姿势阈值（基于 RULA/REBA，相对校准基准的偏移）=====
// pitch=前后倾(屈伸)  roll=左右倾(侧屈)  yaw不参与判断

// Head / Node1
#define HEAD_PITCH_BAD      18.0f
#define HEAD_PITCH_RECOVER  13.0f
#define HEAD_ROLL_BAD       15.0f
#define HEAD_ROLL_RECOVER   10.0f

// Chest / S3
#define CHEST_PITCH_BAD     20.0f
#define CHEST_PITCH_RECOVER 15.0f
#define CHEST_ROLL_BAD      15.0f
#define CHEST_ROLL_RECOVER  10.0f

// Waist / Node2, Node3
#define WAIST_PITCH_BAD     20.0f
#define WAIST_PITCH_RECOVER 15.0f
#define WAIST_ROLL_BAD      15.0f
#define WAIST_ROLL_RECOVER  10.0f

#endif