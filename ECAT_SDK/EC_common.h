#ifndef EC_COMMON_H
#define EC_COMMON_H

#include "servo_types.h"  // 只放 enum 定義：ServoCommand / JogMode（無任何 SHM 結構）
#include <atomic>
#include <cstdint>
#include <cstdbool>
#include <iostream>

#define SHM_NAME "/ecat_shm"
#define MAX_SERVO_COUNT 5
#define PERIOD_NS 2000000LL
#define DC_TICK_NS 10ULL
#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * 1000000000ULL + (T).tv_nsec)
#define SHM_MAGIC 1234
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))   // 定義 ARRAY_SIZE 宏

extern bool use_dc;
extern volatile int need_restart;

// ===== ServoState 狀態定義 =====
typedef enum {
    SERVO_NOT_READY,
    SERVO_SWITCH_ON_DISABLED,
    SERVO_READY_TO_SWITCH_ON,
    SERVO_SWITCHED_ON,
    SERVO_OPERATION_ENABLED,
    SERVO_QUICK_STOP_ACTIVE,
    SERVO_FAULT_REACTION,
    SERVO_FAULT,
    SERVO_UNKNOWN,
    SERVO_WARNING
} ServoState;

/*
 * 重要：本檔所有 SHM 結構都用 fixed-width + alignas(64)
 * 不要使用 pragma pack，也不要放任何指標成員   
 * 為何?
 */

// 64B 對齊的指令封包（單一 cache line）
struct alignas(64) CommandEntry {
    int32_t  servo_id;         // 4
    int32_t  target_position;  // 8 （CSP: 每拍增量 或 絕對/相對目標）
    int8_t   command_mode;     // 9（例：8=CSP）
    int8_t   jog_mode;         // 10
    int16_t  _pad0;            // 12
    int32_t  command;          // 16（ServoCommand）
    uint32_t seq;              // 20
    int32_t  homingSpeed_A;    // 24  (0=不覆寫；預設200000)
    int32_t  homingSpeed_B;    // 28  (0=不覆寫；預設20000)
    int8_t   homingMethod;     // 29
    // 填滿到 64B：
    int8_t   _pad1[3];         // 32
    uint32_t _pad2[8];         // 64
};

struct alignas(64) AxisMailbox {
    std::atomic<uint32_t> ready_seq; // producer: store-release(seq), consumer: load-acquire()
    uint8_t _pad0[64 - sizeof(std::atomic<uint32_t>)];

    CommandEntry slot;               
};

// 64B 對齊的回饋資料（單一 cache line）
struct alignas(64) ServoData {
    // 32-bit
    int32_t actual_position;   //  0..3   CiA 0x6064
    int32_t actual_velocity;   //  4..7   (未用)
    int32_t follow_error;      //  8..11  0x60F4

    // 16-bit
    uint16_t status_word;      // 12..13  0x6041
    int16_t  error_code;       // 14..15  0x603F

    // 8-bit
    int8_t   actual_mode;      // 16      0x6061
    uint8_t  servo_on;         // 17      由 status 解析
    uint8_t  alarm;            // 18      由 status bit3 解析
    uint8_t  homing_active;    // 19      SDK 狀態回報

    // Touch Probe
    uint16_t probe_function;   // 20..21  0x60B8 (UINT, RW)
    uint16_t probe_status;     // 22..23  0x60B9 (UINT, RO)
    int32_t  probe1_pos;       // 24..27  0x60BA (DINT, RO)
    int32_t  probe1_neg;       // 28..31  0x60BB (DINT, RO)
    int32_t  probe2_pos;       // 32..35  0x60BC (DINT, RO)
    int32_t  probe2_neg;       // 36..39  0x60BD (DINT, RO)

    // 填滿到 64B
    uint8_t  _pad[64 - 40];    // 40..63
};
static_assert(sizeof(ServoData) == 64, "ServoData must be 64B");



struct SharedData {
    alignas(64) AxisMailbox  mbox[MAX_SERVO_COUNT]; // ★ 每軸 mailbox：上位 2ms → RT 2ms 零等待交接
    alignas(64) ServoData    servos[MAX_SERVO_COUNT];
    // alignas(64) RingBuffer   ring_buffer;           // ★ 突發/多筆命令排隊（可選） //0822 移除ring buffer
    uint32_t                 magic;
    
    // 命令幀序（送命令時由命令端遞增）
    alignas(64) std::atomic<uint32_t> frame_seq;
    uint8_t _pad_fs[64 - sizeof(std::atomic<uint32_t>)];

    // ★ 回饋幀序（每拍由 daemon 遞增；SDK/SCUT 只讀）
    alignas(64) std::atomic<uint32_t> fb_seq;
    uint8_t _pad_fb[64 - sizeof(std::atomic<uint32_t>)];
};

#endif // EC_COMMON_H
