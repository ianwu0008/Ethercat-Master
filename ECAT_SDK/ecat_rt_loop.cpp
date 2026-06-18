#include "ecat_rt_loop.h"
#include "ecat_pdo_config.h"
#include "EC_common.h"
#include "motion_control.h"

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <sched.h>
#include <signal.h>
#include <math.h>
#include <ecrt.h>
#include <limits.h>

// ---- 外部由其它模組提供 ----
extern volatile sig_atomic_t running;
extern SharedData *shm_ptr;
extern ec_master_t *master;
extern ec_domain_t *domain;
extern uint8_t *domain_pd;
extern bool use_dc;

extern ec_master_state_t master_state;
extern ec_domain_state_t domain_state;
extern ec_slave_config_t *sc[MAX_SERVO_COUNT];
extern ec_slave_config_state_t sc_state[MAX_SERVO_COUNT];

extern uint32_t off_control_word[MAX_SERVO_COUNT];
extern uint32_t off_status_word[MAX_SERVO_COUNT];
extern uint32_t off_mode_display[MAX_SERVO_COUNT];
extern uint32_t off_target_pos[MAX_SERVO_COUNT];
extern uint32_t off_mode_cmd[MAX_SERVO_COUNT];
extern uint32_t off_Pos_Act_Val[MAX_SERVO_COUNT];

extern uint32_t off_Probe_Function[MAX_SERVO_COUNT]; // 0x60B8 (UINT16, RW)
extern uint32_t off_Probe_Status  [MAX_SERVO_COUNT]; // 0x60B9 (UINT16, RO)
extern uint32_t off_Probe1_Pos    [MAX_SERVO_COUNT]; // 0x60BA (DINT,  RO)
extern uint32_t off_Probe1_Neg    [MAX_SERVO_COUNT]; // 0x60BB (DINT,  RO)
extern uint32_t off_Probe2_Pos    [MAX_SERVO_COUNT]; // 0x60BC (DINT,  RO)
extern uint32_t off_Probe2_Neg    [MAX_SERVO_COUNT]; // 0x60BD (DINT,  RO)

// ---- 參數 ----
#define NSEC_PER_SEC               1000000000ULL

// 暖機：Activate 後，先跑一段只收送/處理、不做 motion（給從站轉態與域整理空間）
#define PRIME_CYCLES               2000      // (不同計數器,這是loop_counter)

// OP 判定：連續穩定拍數 + WKC 比率
#define OP_CHECK_INTERVAL_CYCLES   50
#define OP_STABLE_CYCLES           20       // 500*2ms = 1 秒連續穩定才算 OP 穩定
#define WKC_OK_RATIO_NUM           9         // WKC >= 0.9 * expected_wkc 即視為 ok
#define WKC_OK_RATIO_DEN           10

#define DC_LOCK_DELAY_CYCLES       10       // OP 穩定後等 1 秒再進 RUN
#define RUN_WKC_BAD_LIMIT          3        // RUN 中容忍短暫 WKC 抖動；任一軸非 OP 仍立即退出

// （避免狂刷）
#define PRINT_INTERVAL_CYCLES      500       // 每 1 秒最多印一次周期性訊息

// ---- 內部狀態 ----
static uint64_t wakeup_time_ns;
static unsigned expected_wkc = 0;

static int loop_counter = 0;
static int SYNC_REF_INTERVAL_CYCLES = 1;    // 每  1 拍同步一次
static int last_info_print = -PRINT_INTERVAL_CYCLES;

static int op_consecutive = 0;
static int dc_guard = 0;
static int sync_ref_div = 0;
static int run_wkc_bad_count = 0;

static bool motion_enabled = false;

// ---- 幫助函式 ----
static inline uint64_t timespec_to_ns(const struct timespec& ts) {
    return ((uint64_t)ts.tv_sec) * NSEC_PER_SEC + ts.tv_nsec;
}

static inline void sleep_until_and_step_period(uint64_t period_ns) {
    struct timespec ts;
    ts.tv_sec  = wakeup_time_ns / NSEC_PER_SEC;
    ts.tv_nsec = wakeup_time_ns % NSEC_PER_SEC;

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

    wakeup_time_ns += period_ns;

    // 若超前或落後很多（overrun），用現在時間對齊下一拍
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    uint64_t now_ns = timespec_to_ns(now);
    if ((int64_t)(wakeup_time_ns - now_ns) < -(int64_t)period_ns) {
         printf("ℹ️ Overrun in RT loop\n");
        // 落後多拍，重新以現在時間對齊
        uint64_t missed = (now_ns - wakeup_time_ns) / period_ns + 1;
        wakeup_time_ns += missed * period_ns;
    }
}

static inline int domain_wkc() {
    ec_domain_state_t ds;
    ecrt_domain_state(domain, &ds);
    return (int)ds.working_counter;
}

static inline bool wkc_is_ok(unsigned wkc) {
    if (!expected_wkc) return (wkc > 0);
    return (wkc * WKC_OK_RATIO_DEN >= expected_wkc * WKC_OK_RATIO_NUM);
}

static inline bool all_slaves_in_op() {
    bool all_op = true;
    for (int i = 0; i < MAX_SERVO_COUNT; ++i) {
        ecrt_slave_config_state(sc[i], &sc_state[i]);
        if (sc_state[i].al_state != EC_AL_STATE_OP) {
            all_op = false;
        }
    }
    return all_op;
}

//印製系統時間
static double getBootTime() {
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static void print_stability_detail(const char* reason, int wkc, bool wkc_ok)
{
    printf("[%.6f] %s wkc=%d expected=%u wkc_ok=%d states:",
           getBootTime(),
           reason,
           wkc,
           expected_wkc,
           wkc_ok ? 1 : 0);
    for (int i = 0; i < MAX_SERVO_COUNT; ++i) {
        printf(" ax%d=0x%02x", i, sc_state[i].al_state);
    }
    printf("\n");
}

static inline void apply_dc_sync(uint64_t linux_time_ns) {
    ecrt_master_application_time(master, linux_time_ns);
    
    if (++sync_ref_div >= SYNC_REF_INTERVAL_CYCLES) {
        ecrt_master_sync_reference_clock(master);
        sync_ref_div = 0;
    }
    if(motion_enabled ){
        SYNC_REF_INTERVAL_CYCLES = 10;  //同步頻率
    }else{
        SYNC_REF_INTERVAL_CYCLES = 1; 
    }
       
    ecrt_master_sync_slave_clocks(master);
}

// ---- 狀態機 ----
typedef enum {
    PH_PRIME = 0,   // 暖機：僅收送/處理/DC 報時；不進 motion
    PH_WAIT_OP,     // 等 OP + WKC 穩定，鎖定 expected_wkc
    PH_DC_LOCK,     // OP 穩定後再等待 DC 穩住（延遲一段時間）
    PH_RUN          // 進入運動：每拍呼叫 motion_control_update_servos()
} rt_phase_t;

static rt_phase_t phase = PH_PRIME;

// ---- 狀態監測/轉移 ----
static void update_phase_machine() {
    // 降頻做昂貴的檢查
    if ((loop_counter % OP_CHECK_INTERVAL_CYCLES) != 0) return;

    ecrt_master_state(master, &master_state);
    const bool slaves_ok = all_slaves_in_op();
    const int  wkc       = domain_wkc();
    const bool wkc_ok    = wkc_is_ok((unsigned)wkc);

    switch (phase) {
    case PH_PRIME:
        // 只要主站 alive 就前進到 WAIT_OP（暖機時間主要靠 PRIME_CYCLES）
        if (loop_counter >= PRIME_CYCLES) {
            phase = PH_WAIT_OP;
            op_consecutive = 0;
            dc_guard = 0;
            run_wkc_bad_count = 0;
            expected_wkc = 0;
            if (loop_counter - last_info_print >= PRINT_INTERVAL_CYCLES) {
                printf("ℹ️  PRIME done -> WAIT_OP\n");
                last_info_print = loop_counter;
            }
        }
        break;

    case PH_WAIT_OP:
        if (slaves_ok && wkc_ok) {
            if (op_consecutive < OP_STABLE_CYCLES) ++op_consecutive;
        } else {
            op_consecutive = 0;
        }

        if (op_consecutive >= OP_STABLE_CYCLES) {
            // 鎖定 expected_wkc（穩定時量測）
            if (!expected_wkc) expected_wkc = (unsigned)wkc;
            phase = PH_DC_LOCK;
            dc_guard = 0;
            run_wkc_bad_count = 0;
            if (loop_counter - last_info_print >= PRINT_INTERVAL_CYCLES) {
                printf("[%.6f] OP stable. -> DC_LOCK \n", getBootTime());
                last_info_print = loop_counter;
            }
        }
        break;

    case PH_DC_LOCK:
        // 繼續要求維持 OP + WKC ok，並等待一段延遲讓 DC 真正穩住
        if (slaves_ok && wkc_ok) {
            if (++dc_guard >= DC_LOCK_DELAY_CYCLES) {
                phase = PH_RUN;
                motion_enabled = true;
                run_wkc_bad_count = 0;
                if (loop_counter - last_info_print >= PRINT_INTERVAL_CYCLES) {
                    printf("[%.6f] 🔧 DC lock. -> RUN \n", getBootTime());
                    last_info_print = loop_counter;
                }
            }
        } else {
            // 失穩則退回 WAIT_OP
            phase = PH_WAIT_OP;
            op_consecutive = 0;
            dc_guard = 0;
            run_wkc_bad_count = 0;
            motion_enabled = false;
            if (loop_counter - last_info_print >= PRINT_INTERVAL_CYCLES) {                
                printf("[%.6f] Lost stability in DC_LOCK. -> WAIT_OP \n", getBootTime());
                last_info_print = loop_counter;
            }
        }
        break;

    case PH_RUN:
        if (!slaves_ok) {
            // 任一軸離開 OP，立即退出 RUN。
            phase = PH_WAIT_OP;
            op_consecutive = 0;
            dc_guard = 0;
            run_wkc_bad_count = 0;
            motion_enabled = false;
            if (loop_counter - last_info_print >= PRINT_INTERVAL_CYCLES) {
                printf("[%.6f] RUN lost OP -> WAIT_OP \n", getBootTime());
                print_stability_detail("RUN axis not OP", wkc, wkc_ok);
                last_info_print = loop_counter;
            }
        } else if (!wkc_ok) {
            // 軸仍 OP 時，容忍短暫 WKC 抖動，避免單次封包抖動造成 RUN 反覆退出。
            if (++run_wkc_bad_count >= RUN_WKC_BAD_LIMIT) {
                phase = PH_WAIT_OP;
                op_consecutive = 0;
                dc_guard = 0;
                run_wkc_bad_count = 0;
                motion_enabled = false;
            }
        } else {
            run_wkc_bad_count = 0;
        }
        break;
    }
}

// ---- 主 RT 迴圈 ----
void run_rt_loop(void) {
    printf("====== RT Loop (refactored) ======\n");

    // 初始化絕對節拍基準
    {
        struct timespec t0;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        wakeup_time_ns = timespec_to_ns(t0) + PERIOD_NS;

        printf("[%.6f] RT 起始系統時間 \n", getBootTime());
    }

    // 初始狀態
    phase = PH_PRIME;
    loop_counter = 0;
    last_info_print = -PRINT_INTERVAL_CYCLES;
    op_consecutive = 0;
    dc_guard = 0;
    run_wkc_bad_count = 0;
    expected_wkc = 0;
    motion_enabled = false;
    sync_ref_div = SYNC_REF_INTERVAL_CYCLES; // 讓第一個降頻點儘快觸發一次

    while (running) {
        ++loop_counter;

        ecrt_master_receive(master);
        ecrt_domain_process(domain);
        

        // 2) 報告應用時間 + DC 同步
        //    IGH 建議每拍做 application_time() + sync_slave_clocks()，
        //    sync_reference_clock() 則降頻。
        {
            struct timespec ts_now;
            clock_gettime(CLOCK_MONOTONIC, &ts_now);
            const uint64_t linux_time_ns = timespec_to_ns(ts_now);
            apply_dc_sync(linux_time_ns);
        }
        // 3) 狀態機（限頻檢查）
        update_phase_machine();
        

        // 4) 運動（僅 RUN 才啟動）：你的 motion_control_update_servos 內部會做 PDO 寫入
        if (motion_enabled && (phase == PH_RUN)) {
            motion_control_update_servos();
        }

        ecrt_domain_queue(domain);
        ecrt_master_send(master);

        // 絕對節拍
        sleep_until_and_step_period(PERIOD_NS);
    }

    printf("RT loop exit.\n");
}
