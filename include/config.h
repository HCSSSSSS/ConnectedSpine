#ifndef CONFIG_H
#define CONFIG_H

// =====================================================================
//  Connected Spine — 共享配置
//  烧录前每块板要确认：
//    · C3：把下面 NODE_ID 改成 1/2/3；main_c3.cpp 顶部填 MAC_S3
//    · S3：NODE_ID 不影响；main_s3.cpp 顶部填 MAC_NODE1/2/3
//    · 四块板的 ESPNOW_CHANNEL 必须一致
//    · 任何改动后，四块板全部重新 build + upload（长度分发依赖两端一致）
// =====================================================================

// ===== 节点标识 =====
// C3 烧录时改这里：1=HEAD  2=LEFT_WAIST(WaistL)  3=RIGHT_WAIST(WaistR)
// S3（胸口/主节点）固定为 index 0，不读取本宏，留默认即可
#define NODE_ID 3

// ===== 通信 =====
#define ESPNOW_CHANNEL 1 // 两端必须一致，范围 1..13

// C3 广播 IMU 姿态的间隔。周期数据，丢帧无所谓，发快一点数据更新鲜、nodeActive 余量更大
#define IMU_SEND_INTERVAL_MS 50 // 20Hz；100=10Hz 对姿态也足够

// 节点掉线判定：超过此时间没收到该 C3 的 IMU 包即视为离线
// 容错帧数 = NODE_TIMEOUT_MS / IMU_SEND_INTERVAL_MS（当前 1000/50 = 20 帧，足够吸收突发丢包）
#define NODE_TIMEOUT_MS 1000

// ===== 震动命令可靠性（单播 + ACK + 重传）=====
#define VIB_RETRY_INTERVAL_MS 40 // 没收到 ACK 时多久重发一次
#define VIB_MAX_RETRIES 6        // 最多重发次数
// 总重传窗口 = 40 × 6 = 240ms，必须 < VIBRATION_COOLDOWN_MS（见文件末尾编译检查）

// ===== 震动行为 =====
#define VIBRATION_COOLDOWN_MS 2000 // 同一节点两次震动的最短间隔

// ===== S3 姿态判断 / 串口打印间隔 =====
// 注意：在 S3 loop 里判断和打印是同一个节拍（耦合）
// 越小 → 弯腰到震动的延迟越低，但串口刷得越快
// 200ms ≈ 5Hz：延迟 ≤200ms（手感即时），串口仍可读；嫌吵就调大，嫌慢就调小
#define JUDGE_INTERVAL_MS 200

// ===== 校准按钮（S3，D7 接 GND，内部上拉，按下读 LOW）=====
#define CAL_BUTTON_PIN D7
#define BUTTON_DEBOUNCE_MS 50

// ===== 姿势阈值（基于 RULA/REBA，相对校准基准的偏移，单位：度）=====
// pitch=前后倾(屈伸)  roll=左右倾(侧屈)  yaw 不参与判断
// 每对必须 BAD > RECOVER（滞回，防止临界点反复触发）
// 以下数值为标定结果，未改动

// Head / Node1
#define HEAD_PITCH_BAD 18.0f
#define HEAD_PITCH_RECOVER 13.0f
#define HEAD_ROLL_BAD 15.0f
#define HEAD_ROLL_RECOVER 10.0f

// Chest / S3
#define CHEST_PITCH_BAD 20.0f
#define CHEST_PITCH_RECOVER 15.0f
#define CHEST_ROLL_BAD 15.0f
#define CHEST_ROLL_RECOVER 10.0f

// Waist / Node2, Node3
#define WAIST_PITCH_BAD 20.0f
#define WAIST_PITCH_RECOVER 15.0f
#define WAIST_ROLL_BAD 15.0f
#define WAIST_ROLL_RECOVER 10.0f

// ===== 编译期防呆检查 =====
#if (ESPNOW_CHANNEL < 1) || (ESPNOW_CHANNEL > 13)
#error "ESPNOW_CHANNEL must be 1..13"
#endif
#if (VIB_RETRY_INTERVAL_MS * VIB_MAX_RETRIES) >= VIBRATION_COOLDOWN_MS
#error "Retry window (VIB_RETRY_INTERVAL_MS * VIB_MAX_RETRIES) must be < VIBRATION_COOLDOWN_MS"
#endif

#endif