// motion_control.cpp 
// SDK
#include "motion_control.h"
#include "ecat_pdo_config.h"
#include <iostream>
#include <cstring>
#include <ctime>   /* 時間相關函數 */
#include <limits>

using namespace std;

// ==== 參數（依 2ms 週期，可自行調）====
static constexpr int   kCSPMode = 8;

static constexpr int32_t ABS_MAX_STEP_PER_CYCLE =5000; // ABS 逼近每拍最大步距 //安全測試用
static constexpr int32_t ABS_DEADBAND           = 1;  // 到位死區
static constexpr int     ABS_SETTLE_CYCLES      = 3;  // 連續幾拍在死區視為到位


static bool auto_enable[MAX_SERVO_COUNT]; // 新增旗標，控制自動推回 OP

// Velocity feedforward (0x60B1) for X and Y.
// Set only command units per motor revolution per axis. A zero disables an
// axis until its setting is known.
static constexpr uint32_t VELOCITY_FF_AXIS_MASK = 0x03;
static constexpr int64_t  VELOCITY_FF_PERCENT = 100;
static constexpr int64_t  MOTOR_INC_PER_REV = 67108864; // 2701:1
static constexpr int64_t  VELOCITY_INC_PER_SEC = 64;    // 2702:1 / 2702:2
static constexpr int64_t  COMMAND_UNITS_PER_REV[MAX_SERVO_COUNT] = {
    50000, // X
    50000, // Y
    5000,  // Z
    5000,  // U
    5000   // V
};
static constexpr int64_t  VELOCITY_FF_MAX_ABS = 500000;

struct AxisCtx {
    bool     mode_ready      = false;
    bool     pos_initialized = false;
    int64_t  target_tracker  = 0;   // 內部插補器的目標
    int32_t  stream_step     = 0;   // 連續步距（JOG_CONTINUOUS）

    // ABS 目標
    bool     abs_active      = false;
    bool     rel_stream      = false;
    int64_t  abs_target      = 0;
    int      abs_settle      = 0;
    int32_t  last_abs_cmd    = std::numeric_limits<int32_t>::min();

    // 幀/日誌
    uint32_t last_seq        = 0;
    uint32_t log_counter     = 0;
    int64_t  last_motion_step = 0;

    // 回授/對齊
    int32_t  last_actual_raw = 0;

    // 軟零點：對外回報時把 actual_pos + soft_zero 視為「工藝零」
    int32_t  soft_zero       = 0;
    bool    halt_active      = false;         // Halt active (bit8=1)
    
};

static AxisCtx ax[MAX_SERVO_COUNT];


// ===== parse_state：需與 motion_control.h 的宣告一致（不可 static）=====
ServoState parse_state(uint16_t status) {
    // 先檢查 Warning 和 Fault（bit 7 和 bit 3）
    if (status & 0x0080) return SERVO_WARNING;
    if (status & 0x0008) return SERVO_FAULT;

    // 標準 CiA 402 狀態檢查 (bit 0-3, 5-6)
    switch (status & 0x006F) {
        case 0x0000: return SERVO_NOT_READY;
        case 0x0040: return SERVO_SWITCH_ON_DISABLED;
        case 0x0021: return SERVO_READY_TO_SWITCH_ON;
        case 0x0023: return SERVO_SWITCHED_ON;
        case 0x0027: return SERVO_OPERATION_ENABLED;
        case 0x0007: return SERVO_QUICK_STOP_ACTIVE;
        case 0x000F: return SERVO_FAULT_REACTION;
    }


    // 若 bit 2=0 且無 Fault，可能處於 Quick Stop
    if ((status & 0x0004) == 0 && !(status & 0x0008)) return SERVO_QUICK_STOP_ACTIVE;

    return SERVO_UNKNOWN;
}


// ---- 32-bit clamp（避免溢位）----
static inline int32_t clamp_to_i32(int64_t v) {
    if (v > std::numeric_limits<int32_t>::max()) return std::numeric_limits<int32_t>::max();
    if (v < std::numeric_limits<int32_t>::min()) return std::numeric_limits<int32_t>::min();
    return (int32_t)v;
}

// ---- 是否用「上位座標＝加 soft_zero」送到驅動器 ----
// 預設 false：送機械原始座標
static bool kUseHostCoordsForOutput = false; //true / false;

// ---- 取得要寫到 0x607A 的最終目標 ----
static inline int32_t out_target_pos(int i, int64_t tracker) {
    int64_t v = tracker;
    if (kUseHostCoordsForOutput) v += (int64_t)ax[i].soft_zero;  // 切到「上位座標」模式時才加
    return clamp_to_i32(v);
}


static inline ServoState read_feedback_and_update_ctx(
    int i, int32_t& actual_pos_raw, int8_t& actual_mode)
{
    const uint16_t status     = EC_READ_U16(domain_pd + off_status_word[i]);
    actual_mode               = EC_READ_S8 (domain_pd + off_mode_display[i]);
    actual_pos_raw            = EC_READ_S32(domain_pd + off_Pos_Act_Val[i]);
    const int32_t Pos_error   = EC_READ_S32(domain_pd + off_Pos_error[i]);
    const int16_t error_code  = EC_READ_S16(domain_pd + off_error_code[i]);
    
    // const uint16_t cmd_probe  = EC_READ_U16(domain_pd + off_Probe_Function[i]);// 60B8
    const uint16_t st_probe   = EC_READ_U16(domain_pd + off_Probe_Status[i]);  // 60B9
    const int32_t  p1_pos     = EC_READ_S32(domain_pd + off_Probe1_Pos[i]);    // 60BA
    const int32_t  p1_neg     = EC_READ_S32(domain_pd + off_Probe1_Neg[i]);    // 60BB
    const int32_t  p2_pos     = EC_READ_S32(domain_pd + off_Probe2_Pos[i]);    // 60BC
    const int32_t  p2_neg     = EC_READ_S32(domain_pd + off_Probe2_Neg[i]);    // 60BD
    

    // 對上位回報軟零後的座標（保持原始也可另開欄位）
    // const int32_t actual_pos_for_host = actual_pos_raw + ax[i].soft_zero;
    //actual_pos_for_host 完全沒有其他用途
    shm_ptr->shm_soft_zero[i].store(ax[i].soft_zero, std::memory_order_relaxed);
    ax[i].last_actual_raw = actual_pos_raw;

    shm_ptr->servos[i].actual_position = actual_pos_raw + ax[i].soft_zero;
    shm_ptr->servos[i].follow_error    = Pos_error;
    shm_ptr->servos[i].status_word     = status;
    shm_ptr->servos[i].actual_mode     = actual_mode;
    shm_ptr->servos[i].error_code      = error_code;
    shm_ptr->servos[i].alarm           = (status & 0x0008) != 0;

    shm_ptr->servos[i].probe_status = st_probe;
    shm_ptr->servos[i].probe1_pos   = p1_pos;
    shm_ptr->servos[i].probe1_neg   = p1_neg;
    shm_ptr->servos[i].probe2_pos   = p2_pos;
    shm_ptr->servos[i].probe2_neg   = p2_neg;

    ServoState cur = parse_state(status);
    shm_ptr->servos[i].servo_on = (cur == SERVO_OPERATION_ENABLED || cur == SERVO_SWITCHED_ON);
    return cur;
}


static inline void auto_reset_and_servo_on_once(int i, ServoState cur) {
    static bool fault_reset_pending[MAX_SERVO_COUNT] = {false}; // 靜態陣列，記錄 Fault Reset 狀態
    
    // 若 auto_enable 為 false（由 CMD_SERVO_OFF/CMD_STOP 設置），則不自動推回
    if (!auto_enable[i]) {
        return;
    }


    if (cur == SERVO_FAULT) {
        if (!fault_reset_pending[i]) {
            // 發送 Fault Reset (bit 7=1)
            EC_WRITE_U16(domain_pd + off_control_word[i], CONTROL_FAULT_RESET);
            fault_reset_pending[i] = true;
            ax[i].pos_initialized = false;
        } else {
            // 下一拍清除 Fault Reset (bit 7=0)，恢復正常控制字
            EC_WRITE_U16(domain_pd + off_control_word[i], CONTROL_SHUTDOWN);
            fault_reset_pending[i] = false;
        }
        return;
    }

    if (cur == SERVO_QUICK_STOP_ACTIVE) {   //必須是CONTROL_SHUTDOWN , 否則某一軸會卡在這
        EC_WRITE_U16(domain_pd + off_control_word[i], CONTROL_SHUTDOWN);
        ax[i].pos_initialized = false;
        return;
    }

    if (cur == SERVO_NOT_READY || cur == SERVO_SWITCH_ON_DISABLED) {
        EC_WRITE_U16(domain_pd + off_control_word[i], CONTROL_SHUTDOWN);
        ax[i].pos_initialized = false;
        return;
    }

    if (cur == SERVO_READY_TO_SWITCH_ON) {
        EC_WRITE_U16(domain_pd + off_control_word[i], CONTROL_SWITCH_ON);
        ax[i].pos_initialized = false;
        return;
    }

    if (cur == SERVO_SWITCHED_ON) {
        EC_WRITE_U16(domain_pd + off_control_word[i], CONTROL_ENABLE);
        ax[i].mode_ready = false;
        ax[i].pos_initialized = false;
        return;
    }
}

static inline bool ensure_op_and_csp(int i, ServoState cur, int32_t actual_pos_raw, int8_t actual_mode)
{
    if (cur != SERVO_OPERATION_ENABLED) {
        auto_reset_and_servo_on_once(i, cur);
        ax[i].mode_ready      = (actual_mode == kCSPMode);
        ax[i].pos_initialized = false;
        ax[i].stream_step     = 0;
        ax[i].abs_active      = false;
        ax[i].abs_settle      = 0;
        ax[i].target_tracker  = (int64_t)actual_pos_raw;
        EC_WRITE_S32(domain_pd + off_target_pos[i], out_target_pos(i, ax[i].target_tracker));
        return false;
    }
    if (!ax[i].mode_ready) {
        if (actual_mode != kCSPMode) {
            EC_WRITE_S8(domain_pd + off_mode_cmd[i], kCSPMode);
            printf("[%d] ensure_op_and_csp: 推 mode=8\n", i);
            return false;
        }
        ax[i].mode_ready = true;
    }
    if (!ax[i].pos_initialized) {
        ax[i].target_tracker  = (int64_t)actual_pos_raw;
        ax[i].pos_initialized = true;
        EC_WRITE_S32(domain_pd + off_target_pos[i], out_target_pos(i, ax[i].target_tracker));    
        return false;
    }
    return true;
}

static inline void process_axis_commands(int i)
{
    AxisMailbox& mb = shm_ptr->mbox[i];

    while (true) {
        uint32_t tail = mb.tail.load(std::memory_order_relaxed);
        uint32_t head = mb.head.load(std::memory_order_acquire);

        if (tail == head) break;  // 隊列空

        // 先複製出一份 command，再把 tail 往前推
        CommandEntry e = mb.slots[tail];  // tail 已經被約束在 0..SIZE-1
        ServoCommand cmd = static_cast<ServoCommand>(e.command);
        JogMode jmode    = static_cast<JogMode>(e.jog_mode);
        int32_t val      = e.target_position;

        // ★ 先釋放這格：即使後面 early return，也不會卡住 queue
        mb.tail.store((tail + 1) & QUEUE_MASK, std::memory_order_release);

        // === 以下 switch 完全用你的語意 ===
        switch (cmd) {
            case CMD_FAULT_RESET:
                EC_WRITE_U16(domain_pd + off_control_word[i], CONTROL_FAULT_RESET);
                auto_enable[i] = false; // 防止自動推回 OP
                // 不 return，允許同一拍處理多個命令
                break;

            case CMD_SERVO_ON:
                ax[i].pos_initialized = false;
                ax[i].stream_step = 0;
                ax[i].abs_active = false;
                ax[i].abs_settle = 0;
                auto_enable[i] = true;
                cout << "[Axis " << i << "] SERVO_ON received ! \n";
                break;

            case CMD_SERVO_OFF:
                ax[i].stream_step = 0;
                ax[i].abs_active = false;
                ax[i].abs_settle = 0;
                ax[i].pos_initialized = false;
                ax[i].mode_ready = false;
                auto_enable[i] = false;
                EC_WRITE_U16(domain_pd + off_control_word[i], CONTROL_QUICK_STOP);
                cout << "[Axis " << i << "] SERVO_OFF received, shutdown ! \n";
                break;

            case CMD_STOP:
                ax[i].abs_active     = false;
                ax[i].abs_settle     = 0;
                ax[i].stream_step    = 0;
                ax[i].halt_active    = true;
                auto_enable[i]       = true;

                EC_WRITE_S8 (domain_pd + off_mode_cmd[i], kCSPMode);
                EC_WRITE_U16(domain_pd + off_control_word[i], CONTROL_HALT);
                cout << "[Axis " << i << "] STOP received (FEED_HOLD).\n";
                // 不 return，後面 handle_axis 會看到 halt_active=true 並走 HALT 分支
                break;

            case CMD_QUICK_STOP:
                ax[i].pos_initialized = false;
                ax[i].stream_step = 0;
                ax[i].abs_active = false;
                ax[i].abs_settle = 0;
                auto_enable[i] = false;

                EC_WRITE_U16(domain_pd + off_control_word[i], CONTROL_QUICK_STOP);
                cout << "[Axis " << i << "] STOP received, entering Quick Stop.\n";
                break;

            case CMD_SET_MODE:
                EC_WRITE_S8(domain_pd + off_mode_cmd[i],
                            static_cast<int8_t>(e.command_mode));
                ax[i].mode_ready = false;
                break;

            case CMD_JOG:
                if (jmode == JOG_ABS) {
                    ax[i].abs_active = true;
                    ax[i].rel_stream = false;
                    ax[i].abs_target = (int64_t)val - (int64_t)ax[i].soft_zero;
                    ax[i].abs_settle = 0;
                    ax[i].stream_step = 0;
                    ax[i].last_abs_cmd = val;
                } else if (jmode == JOG_ABS_RAW) {
                    ax[i].abs_active = true;
                    ax[i].rel_stream = false;
                    ax[i].abs_target = (int64_t)val;
                    ax[i].abs_settle = 0;
                    ax[i].stream_step = 0;
                } else if (jmode == JOG_REL) {
                    if (val == 0) {
                        ax[i].abs_active  = false;
                        ax[i].stream_step = 0;
                        ax[i].abs_settle  = 0;
                    } else {
                        int64_t base = ax[i].target_tracker;
                        int64_t tgt  = base + (int64_t)val;
                        ax[i].abs_active  = true;
                        ax[i].rel_stream  = false;
                        ax[i].abs_target  = tgt;
                        ax[i].abs_settle  = 0;
                        ax[i].stream_step = 0;
                    }
                } else if (jmode == JOG_CONTINUOUS) {
                    ax[i].abs_active = false;
                    ax[i].rel_stream = false;
                    ax[i].stream_step = val;
                }
                break;

            case CMD_ZERO_POS:
                {
                    int ax_id = e.servo_id;
                    int32_t r = e.zero_raw;
                    cout << "[DAEMON ZERO] axis=" << ax_id
                        << " soft_zero(before)=" << ax[ax_id].soft_zero << "\n";

                    ax[ax_id].soft_zero = -r;  // 以這個 raw 為「當前位置=0」

                    cout << "soft_zero(after)=" << ax[ax_id].soft_zero << "\n";

                    ax[ax_id].abs_active   = false;
                    ax[ax_id].abs_settle   = 0;
                    ax[ax_id].stream_step  = 0;
                    ax[ax_id].mode_ready = true;
                    ax[ax_id].pos_initialized = true;
                    break;
                }
            default:
                // do nothing
                break;
        }
    }
}

static inline int64_t divide_round_nearest(int64_t numerator, int64_t denominator)
{
    if (numerator >= 0) return (numerator + denominator / 2) / denominator;
    return (numerator - denominator / 2) / denominator;
}

static inline int32_t velocity_feedforward_value(int i)
{
    if ((VELOCITY_FF_AXIS_MASK & (1u << i)) == 0 ||
        VELOCITY_FF_PERCENT == 0 ||
        COMMAND_UNITS_PER_REV[i] <= 0 ||
        ax[i].last_motion_step == 0) {
        return 0;
    }

    static_assert(PERIOD_NS > 0 && (1000000000LL % PERIOD_NS) == 0,
                  "Velocity feedforward expects an integer number of cycles per second");
    static_assert(MOTOR_INC_PER_REV > 0 && VELOCITY_INC_PER_SEC > 0,
                  "Velocity feedforward unit settings must be positive");

    const int64_t cycles_per_second = 1000000000LL / PERIOD_NS;
    const int64_t numerator =
        ax[i].last_motion_step *
        cycles_per_second *
        MOTOR_INC_PER_REV *
        VELOCITY_FF_PERCENT;
    const int64_t denominator =
        COMMAND_UNITS_PER_REV[i] *
        VELOCITY_INC_PER_SEC *
        100;

    int64_t value = divide_round_nearest(numerator, denominator);
    if (value > VELOCITY_FF_MAX_ABS) value = VELOCITY_FF_MAX_ABS;
    if (value < -VELOCITY_FF_MAX_ABS) value = -VELOCITY_FF_MAX_ABS;
    return clamp_to_i32(value);
}


static inline void run_motion(int i)
{
    ax[i].last_motion_step = 0;
    if (ax[i].abs_active) {
        int64_t err = ax[i].abs_target - ax[i].target_tracker;

        if (!ax[i].rel_stream) {
            // ← 只有在「非 REL 串流」（純 ABS 到點）才啟用到位自動關閉
            if (std::llabs(err) < ABS_DEADBAND) {
                if (++ax[i].abs_settle >= ABS_SETTLE_CYCLES) {
                    ax[i].abs_active  = false;
                    ax[i].stream_step = 0;
                }
            } else {
                ax[i].abs_settle = 0;
            }
        } else {
            // REL 串流：永不自動關閉，避免「偶發 0 誤差 → 關掉一拍」造成卡頓
            ax[i].abs_settle = 0;
        }

        // 逼近一步（兩種模式都要）
        if (err != 0) {
            int64_t step = err;
            if (step >  ABS_MAX_STEP_PER_CYCLE) step =  ABS_MAX_STEP_PER_CYCLE;
            if (step < -ABS_MAX_STEP_PER_CYCLE) step = -ABS_MAX_STEP_PER_CYCLE;
            ax[i].target_tracker += step;
            ax[i].last_motion_step = step;
        }
        return;
    }

    if (ax[i].stream_step != 0) {
        ax[i].target_tracker += (int64_t)ax[i].stream_step;
        ax[i].last_motion_step = ax[i].stream_step;
    }
}


static inline void handle_axis(int i)
{
    int32_t actual_pos_raw = 0;
    int8_t actual_mode = 0;
    ServoState cur = read_feedback_and_update_ctx(i, actual_pos_raw, actual_mode);
    EC_WRITE_S32(domain_pd + off_velocity_offset[i], 0);

    //CMD 
    const uint32_t frame_now = shm_ptr->frame_seq.load(std::memory_order_acquire);
    // consume_frame_cmd(i, frame_now);
    process_axis_commands(i);


    if (ax[i].halt_active) {
        EC_WRITE_U16(domain_pd + off_control_word[i], CONTROL_HALT);
        EC_WRITE_S32(domain_pd + off_target_pos[i], out_target_pos(i, ax[i].target_tracker));

        const uint16_t sw = shm_ptr->servos[i].status_word;

        // 若出現跟隨誤差，可選擇記錄/告警（不自動離開 HALT）
        if (sw & SW_FOLLOWING_ERROR) {
            // TODO: log/告警（保持 HALT）
            cout << "[Axis " << i << "] HALT: Following Error detected (bit13).\n";
            return;
        }

        // 目標值有效（bit12=1）且到達目標（bit10=1）才離開 Halt
        if ( sw & SW_TARGET_REACHED) {
            ax[i].halt_active    = false;          // 退出 Halt
            ax[i].pos_initialized = true;          // 目標與實位一致
            ax[i].target_tracker  = ax[i].last_actual_raw;
            cout << "[Axis " << i << "] Exiting HALT, back to CSP mode.\n";

            // 回到「正常 OP + CSP」，但不推動軸（維持原點）
            EC_WRITE_U16(domain_pd + off_control_word[i], CONTROL_ENABLE);
            EC_WRITE_S8 (domain_pd + off_mode_cmd[i], kCSPMode);
            EC_WRITE_S32(domain_pd + off_target_pos[i], out_target_pos(i, ax[i].target_tracker));
        }
        return;
    }


    if (!ensure_op_and_csp(i, cur, actual_pos_raw, actual_mode)) return;    
    
    run_motion(i);
    const int32_t velocity_ff = velocity_feedforward_value(i);

    EC_WRITE_U16(domain_pd + off_control_word[i], CONTROL_ENABLE);
    EC_WRITE_S32(domain_pd + off_target_pos[i], out_target_pos(i, ax[i].target_tracker));
    EC_WRITE_S32(domain_pd + off_velocity_offset[i], velocity_ff);

    // if (++ax[i].log_counter % 500 == 0) {
    //     cout << "[Axis " << i << "] "
    //          << "Tracker: " << ax[i].target_tracker
    //          << ", Actual: " << shm_ptr->servos[i].actual_position
    //          << ", Mode: " << static_cast<int>(actual_mode)
    //          << ", State: " << static_cast<int>(cur)
    //          << ", Abs Active: " << ax[i].abs_active
    //          << ", Abs Target: " << ax[i].abs_target
    //          << endl;
    // }
}

static bool motion_inited = false;
bool motion_control_init() {
    motion_inited = true;
    for (int i = 0; i < MAX_SERVO_COUNT; ++i) auto_enable[i] = true;
    cout << "motion_control_init() , auto_enable = true \n";
    return true;
}

void motion_control_update_servos()
{
    // if(!motion_inited)motion_control_init();

    if (!shm_ptr || !domain_pd) return;
    for (int i = 0; i < MAX_SERVO_COUNT; ++i) handle_axis(i);
    shm_ptr->fb_seq.fetch_add(1, std::memory_order_release);
}
