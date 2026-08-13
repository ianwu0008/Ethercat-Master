#ifndef EC_COMMON_H
#define EC_COMMON_H

#include "servo_types.h"  // 只放 enum 定義：ServoCommand / JogMode（無任何 SHM 結構）
#include <atomic>
#include <cstdint>
#include <cstdbool>
#include <iostream>
#include <cstddef>
#include <type_traits>

#define SHM_LEGACY_NAME "/ecat_shm"
#ifndef SHM_NAME
#define SHM_NAME "/ecat_shm_v2"
#endif
#define MAX_SERVO_COUNT 6
#define PERIOD_NS 2000000LL
#define DC_TICK_NS 10ULL
#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * 1000000000ULL + (T).tv_nsec)
#define SHM_MAGIC 0x45434154U
#define SHM_ABI_VERSION 2U
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))   // 定義 ARRAY_SIZE 宏

// ====== 新隊列參數 ======
#define CMD_QUEUE_SIZE 8
#define QUEUE_MASK     (CMD_QUEUE_SIZE - 1)

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
    int32_t  zero_raw;         // 24  (0=不覆寫；預設200000)
    int32_t  homingSpeed_A;    // 28  (0=不覆寫；預設20000)
    int8_t   homingMethod;     // 29
    // 填滿到 64B：
    int8_t   _pad1[3];         // 32
    uint32_t _pad2[8];         // 64
};

// struct alignas(64) AxisMailbox {
//     std::atomic<uint32_t> ready_seq; // producer: store-release(seq), consumer: load-acquire()
//     uint8_t _pad0[64 - sizeof(std::atomic<uint32_t>)];

//     CommandEntry slot;               
// };

static_assert(sizeof(CommandEntry) == 64, "CommandEntry must be 64B");

// ====== 新版 AxisMailbox：Head/Tail + Ring Buffer ======
struct alignas(64) AxisMailbox {
    std::atomic<uint32_t> head;      // write index (producer)
    std::atomic<uint32_t> tail;      // read  index (consumer)
    std::atomic<int32_t> safety_command; // out-of-band STOP/QUICK_STOP/SERVO_OFF
    alignas(64) CommandEntry slots[CMD_QUEUE_SIZE];
};
static_assert(sizeof(AxisMailbox) == 576, "AxisMailbox ABI size changed");

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



// struct SharedData {
//     alignas(64) AxisMailbox  mbox[MAX_SERVO_COUNT]; // ★ 每軸 mailbox：上位 2ms → RT 2ms 零等待交接
//     alignas(64) ServoData    servos[MAX_SERVO_COUNT];
//     // alignas(64) RingBuffer   ring_buffer;           // ★ 突發/多筆命令排隊（可選） //0822 移除ring buffer
//     uint32_t                 magic;
    
//     // 命令幀序（送命令時由命令端遞增）
//     alignas(64) std::atomic<uint32_t> frame_seq;
//     uint8_t _pad_fs[64 - sizeof(std::atomic<uint32_t>)];

//     // ★ 回饋幀序（每拍由 daemon 遞增；SDK/SCUT 只讀）
//     alignas(64) std::atomic<uint32_t> fb_seq;
//     uint8_t _pad_fb[64 - sizeof(std::atomic<uint32_t>)];
// };


// ====== SharedData：Mailbox改為Ring Buffer版本 ======
enum SharedDaemonState : uint32_t {
    SHM_DAEMON_INITIALIZING = 0,
    SHM_DAEMON_READY = 1,
    SHM_DAEMON_RUN = 2,
    SHM_DAEMON_FAULT = 3,
    SHM_DAEMON_STOPPING = 4
};

enum SharedTransportFault : uint32_t {
    SHM_FAULT_NONE = 0,
    SHM_FAULT_RING_FULL = 1,
    SHM_FAULT_PRODUCER_STALE = 2,
    SHM_FAULT_SESSION_INVALID = 3
};

struct alignas(64) SharedHeader {
    uint32_t magic;
    uint32_t abi_version;
    uint32_t shared_data_size;
    uint32_t max_axis_count;
    uint32_t session_id;
    std::atomic<uint32_t> active_axis_count;
    std::atomic<uint32_t> daemon_state;
    std::atomic<uint32_t> daemon_heartbeat;
    std::atomic<uint32_t> producer_heartbeat;
    std::atomic<uint32_t> producer_sequence;
    std::atomic<uint32_t> transport_fault;
    uint32_t reserved[5];
};
static_assert(sizeof(SharedHeader) == 64, "SharedHeader must be 64B");
static_assert(offsetof(SharedHeader, session_id) == 16, "Unexpected session offset");
static_assert(offsetof(SharedHeader, active_axis_count) == 20, "Unexpected axis offset");
static_assert(offsetof(SharedHeader, transport_fault) == 40, "Unexpected fault offset");

struct SharedData {
    alignas(64) SharedHeader header;
    alignas(64) AxisMailbox  mbox[MAX_SERVO_COUNT];
    alignas(64) ServoData    servos[MAX_SERVO_COUNT];
    std::atomic<int32_t> latched_tgt_host[MAX_SERVO_COUNT] = {0};  //(ECAT 寫入鎖存)
    std::atomic<int32_t> shm_soft_zero[MAX_SERVO_COUNT] = {0};  //(共享軟零)

    alignas(64) std::atomic<uint32_t> frame_seq;
    uint8_t _pad_fs[64 - sizeof(std::atomic<uint32_t>)];

    alignas(64) std::atomic<uint32_t> fb_seq;
    uint8_t _pad_fb[64 - sizeof(std::atomic<uint32_t>)];
};

static_assert(CMD_QUEUE_SIZE >= 2 && (CMD_QUEUE_SIZE & (CMD_QUEUE_SIZE - 1)) == 0,
              "CMD_QUEUE_SIZE must be a power of two");
static_assert(sizeof(std::atomic<uint32_t>) == sizeof(uint32_t),
              "Shared atomics must be 32-bit");
static_assert(std::is_standard_layout<SharedHeader>::value,
              "SharedHeader must have standard layout");
static_assert(std::is_standard_layout<SharedData>::value,
              "SharedData must have standard layout");
static_assert(offsetof(SharedData, header) == 0, "SHM header must be first");
static_assert(offsetof(SharedData, mbox) == 64, "Unexpected mailbox offset");
static_assert(offsetof(SharedData, servos) == 3520, "Unexpected feedback offset");
static_assert(offsetof(SharedData, frame_seq) == 3968, "Unexpected frame offset");
static_assert(offsetof(SharedData, fb_seq) == 4032, "Unexpected feedback sequence offset");
static_assert(sizeof(SharedData) == 4096, "Unexpected SharedData ABI size");

#endif // EC_COMMON_H
