// test_writer.cpp
#include "shmRW.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <string>
#include <csignal>
#include <cerrno>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sched.h>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <iomanip>   // for std::setw

static std::atomic<bool> running(true);
static std::atomic<bool> streaming(false);   // 是否持續 2ms 串流
static std::atomic<int>  stream_step(20);    // 每週期增量（脈衝）

// 步距序列（針對單一軸的 REL demo）
static std::atomic<bool> seq_active(false);
static std::atomic<int>  seq_step(0);
static std::atomic<int>  seq_left(0);

static void on_sigint(int){ running=false; }

static void elevate_priority() {
    struct sched_param p{}; p.sched_priority = 95;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &p) != 0) {
        perror("pthread_setschedparam");
    } else {
        std::cout << "[DEBUG] test_writer set FIFO/80\n";
    }
}

// --- 幀式同拍 helpers -------------------------------------------------

// 在同一幀中發一筆 NOP 給指定軸（保持同拍，但不改該軸狀態）
static inline void publish_nop(SharedData* shm, int ax, uint32_t seq) {
    CommandEntry e{};
    e.servo_id        = ax;
    e.command         = CMD_NOP;
    e.jog_mode        = JOG_NONE;
    e.command_mode    = 0;
    e.target_position = 0;
    e.seq             = seq;
    shm_publish_axis(shm, &e);
}

// 同一幀，對「所有軸」送出相同命令（同拍）
static void commit_frame_all(SharedData* shm,
                             ServoCommand cmd,
                             int8_t mode,
                             JogMode jog,
                             int32_t pos,
                             int axes = MAX_SERVO_COUNT) {
    const uint32_t seq = shm_begin_frame(shm);
    for (int ax = 0; ax < axes; ++ax) {
        CommandEntry e{};
        e.servo_id        = ax;
        e.command         = cmd;
        e.command_mode    = mode;
        e.jog_mode        = jog;
        e.target_position = pos;
        e.seq             = seq;
        shm_publish_axis(shm, &e);
    }
    shm_commit_frame(shm);
}
static void Homing_all(SharedData* shm,
                             ServoCommand cmd,
                             int8_t mode,
                             JogMode jog,
                             int32_t pos,
                             int32_t HomingSpeed_B,
                             int axes = MAX_SERVO_COUNT) {
    const uint32_t seq = shm_begin_frame(shm);
    for (int ax = 0; ax < axes; ++ax) {
        CommandEntry e{};
        e.servo_id        = ax;
        e.command         = cmd;
        e.command_mode    = mode;
        e.jog_mode        = jog;
        e.target_position = pos;
        e.homingSpeed_B   = HomingSpeed_B;
        e.homingMethod    = 34; //33 or 34
        e.seq             = seq;
        shm_publish_axis(shm, &e);
    }
    shm_commit_frame(shm);
}

// 同一幀，只對「單一軸」送命令，其它軸送 NOP（同拍不動）
static void commit_frame_one(SharedData* shm,
                             int target_axis,
                             ServoCommand cmd,
                             int8_t mode,
                             JogMode jog,
                             int32_t pos,
                             int axes = MAX_SERVO_COUNT) {
    const uint32_t seq = shm_begin_frame(shm);
    for (int ax = 0; ax < axes; ++ax) {
        if (ax == target_axis) {
            CommandEntry e{};
            e.servo_id        = ax;
            e.command         = cmd;
            e.command_mode    = mode;
            e.jog_mode        = jog;
            e.target_position = pos;
            e.seq             = seq;
            shm_publish_axis(shm, &e);
        } else {
            publish_nop(shm, ax, seq);
        }
    }
    shm_commit_frame(shm);
}

// ----------------------------------------------------------------------

// 2ms 串流執行緒：同拍送 6 軸 JOG_CONTINUOUS（這裡用 sin 當示例）
static void streaming_thread(SharedData* shm)
{
    struct sched_param p{}; p.sched_priority = 70;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &p);

    using namespace std::chrono;
    auto next = steady_clock::now();
    const int axes = MAX_SERVO_COUNT;
    ServoData snap[MAX_SERVO_COUNT];
    uint32_t  read_seq = 0;

    // 前一拍狀態，用來偵測「變化才印」
    static uint16_t prev_stat[MAX_SERVO_COUNT] = {0};
    static int32_t  prev_p1p [MAX_SERVO_COUNT] = {0};
    static int32_t  prev_p1n [MAX_SERVO_COUNT] = {0};
    static int32_t  prev_p2p [MAX_SERVO_COUNT] = {0};
    static int32_t  prev_p2n [MAX_SERVO_COUNT] = {0};

    int tick = 0;

    while (running) {
        // ① 每拍先抓快照（同一幀）
        (void)shm_snapshot_feedback(shm, snap, axes, &read_seq);

        // ② 即時偵測：只要 probe 資料有變化就列印一行
        for (int ax = 0; ax < axes; ++ax) {
            const auto &s = snap[ax];
            bool ch = false;

            if (s.probe_status != prev_stat[ax]) ch = true;
            if (s.probe1_pos  != prev_p1p [ax]) ch = true;
            if (s.probe1_neg  != prev_p1n [ax]) ch = true;
            if (s.probe2_pos  != prev_p2p [ax]) ch = true;
            if (s.probe2_neg  != prev_p2n [ax]) ch = true;

            if (ch) {
                // 解讀一些常用旗標（依你的驅動定義，這裡先原樣印 hex）
                std::cout << "[PROBE] frame=" << read_seq
                          << " ax=" << ax
                          << " status=0x" << std::hex << std::setw(4) << std::setfill('0') << s.probe_status << std::dec << std::setfill(' ')
                          << " P1+= " << s.probe1_pos
                          << " P1-= " << s.probe1_neg
                          << " P2+= " << s.probe2_pos
                          << " P2-= " << s.probe2_neg
                          << " pos= " << s.actual_position
                          << " mode=" << int(s.actual_mode)
                          << "\n";
                prev_stat[ax] = s.probe_status;
                prev_p1p [ax] = s.probe1_pos;
                prev_p1n [ax] = s.probe1_neg;
                prev_p2p [ax] = s.probe2_pos;
                prev_p2n [ax] = s.probe2_neg;
            }
        }

        // ③ 每 2 秒印一次總覽
        if (++tick >= 1000) {
            tick = 0;
            std::cout << "\n  ==== FEEDBACK  frame_seq = " << read_seq << " ====\n";
            std::cout << "ax |   pos     | mode |  SW   | probe |  P1+     P1-     P2+     P2-\n";
            std::cout << "---+-----------+------+-------+-------+-------------------------------\n";
            for (int i = 0; i < axes; ++i) {
                const auto& s = snap[i];
                std::cout << std::setw(2) << i << " | "
                          << std::setw(9) << s.actual_position << " | "
                          << std::setw(4) << int(s.actual_mode) << " | 0x"
                          << std::hex << std::setw(4) << std::setfill('0') << s.status_word
                          << std::dec << std::setfill(' ') << " | 0x"
                          << std::hex << std::setw(4) << std::setfill('0') << s.probe_status
                          << std::dec << std::setfill(' ')
                          << " | " << std::setw(8) << s.probe1_pos
                          << " "  << std::setw(8) << s.probe1_neg
                          << " "  << std::setw(8) << s.probe2_pos
                          << " "  << std::setw(8) << s.probe2_neg
                          << "\n";
            }
            std::cout.flush();
        }

        // ② 每 2 秒印一次（1000 x 2ms）
        // if (++tick >= 1000) {
        //     tick = 0;
        //     // std::cout << "\n  ==== FEEDBACK  frame_seq = " << read_seq << " ====\n";
        //     std::cout << "axis |   position  | mode | status  | alarm | on\n";
        //     std::cout << "-----+-------------+------+---------+-------+----\n";
        //     for (int i = 0; i < axes; ++i) {
        //         const auto& s = snap[i];
        //         std::cout << std::setw(4) << i << " | "
        //                   << std::setw(11) << s.actual_position << " | "
        //                   << std::setw(4)  << int(s.actual_mode)   << " | 0x"
        //                   << std::hex << std::setw(4) << std::setfill('0') << s.status_word
        //                   << std::dec << std::setfill(' ') << " |   "
        //                   << int(s.alarm) << "   | "
        //                   << int(s.servo_on)
        //                   << "\n";
        //     }
        //     std::cout.flush();
        // }

        // ③ 同拍命令：依你的兩種情境擇一送
        // 3a. 單軸步距序列（軸0，相對位移）
        if (seq_active.load(std::memory_order_relaxed)) {
            int left = seq_left.load(std::memory_order_relaxed);
            if (left > 0) {
                const int32_t step = seq_step.load(std::memory_order_relaxed);
                const uint32_t seq = shm_begin_frame(shm);
                for (int ax = 0; ax < axes; ++ax) {
                    CommandEntry e{};
                    e.servo_id = ax;
                    if (ax == 0) {
                        e.command         = CMD_JOG;
                        e.command_mode    = 8;       // CSP
                        e.jog_mode        = JOG_REL;
                        e.target_position = step;
                    } else {
                        e.command         = CMD_NOP; // 其它軸同拍送 NOP
                    }
                    e.seq = seq;
                    shm_publish_axis(shm, &e);
                }
                shm_commit_frame(shm);
                seq_left.store(left - 1, std::memory_order_relaxed);
            } else {
                seq_active.store(false, std::memory_order_relaxed);
            }
        }
        // 3b. 串流：所有軸同拍 JOG_CONTINUOUS（這裡用同相位 sin 當示例）
        else if (streaming.load(std::memory_order_relaxed)) {
            const uint32_t seq = shm_begin_frame(shm);

            //////////////////////////////////////////
            // 速度（每拍位移Δ，單位=脈衝）：用現有的 stream_step
            const int32_t step = stream_step.load(std::memory_order_relaxed);

            for (int ax = 0; ax < axes; ++ax) {
                CommandEntry e{};
                e.servo_id        = ax;
                e.command         = CMD_JOG;
                e.command_mode    = 8;          // CSP
                e.jog_mode        = JOG_REL;    // ★相對位移：每拍 +step
                e.target_position = step;
                e.seq             = seq;
                shm_publish_axis(shm, &e);
            }
            ///////////////////////////////////////
            //SIN 波
            // int amplitude = 200; // ±200
            // int target = static_cast<int>(amplitude * std::sin(phase));
            // phase += 0.02;       // 調整頻率

            // for (int ax = 0; ax < axes; ++ax) {
            //     CommandEntry e{};
            //     e.servo_id        = ax;
            //     e.command         = CMD_JOG;
            //     e.jog_mode        = JOG_CONTINUOUS;
            //     e.command_mode    = 8;            // CSP
            //     e.target_position = target;       // 同相位；要相移可自行 offset
            //     e.seq             = seq;
            //     shm_publish_axis(shm, &e);
            // }
            ////////////////////////////////////////

            shm_commit_frame(shm);
        }
        // else: 本拍不送命令，但仍有 ① 的快照讀取

        // ④ 固定 2ms 節拍
        next += microseconds(2000);
        std::this_thread::sleep_until(next);
    }
}

static void command_loop(SharedData* shm)
{
    std::cout <<
      "\n指令:\n"
      " 1: 所有軸 SERVO ON   2: 所有軸 SERVO OFF   3: 所有軸 FAULT RESET\n"
      " 4: 所有軸 HOMING     5: 所有軸 STOP\n"
      " 6: 開始 CSP 串流     7: 停止串流\n"
      "11: 軸0 走一次相對 +100\n"
      "12: 所有軸 走一次相對 +100\n"
      "13: 軸0 走到絕對 100\n"
      "14: 所有軸 走到絕對 100\n"
      "21: 軸0 步距100、共+1000（示例序列）\n"
      "22: 軸1 步距-100、共-1000（示例序列）\n"
      " q: 離開\n";

    std::string s;
    while (running) {
        std::cout << "\n> ";
        if (!std::getline(std::cin, s)) break;
        if (s.empty()) continue;
        if (s=="q" || s=="Q") { running=false; break; }

        int cmd = 0;
        try { cmd = std::stoi(s); } catch(...) { std::cout<<"無效輸入\n"; continue; }

        switch (cmd) {
            case 1:  // 所有軸 ON
                streaming = false;
                commit_frame_all(shm, CMD_SERVO_ON, /*mode*/0, JOG_NONE, 0);
                std::cout << "→ 所有軸 SERVO ON\n";
                break;

            case 2:  // 所有軸 OFF
                streaming = false;
                commit_frame_all(shm, CMD_SERVO_OFF, 0, JOG_NONE, 0);
                std::cout << "→ 所有軸 SERVO OFF\n";
                break;

            case 3:  // 所有軸 RESET
                streaming = false;
                commit_frame_all(shm, CMD_FAULT_RESET, 0, JOG_NONE, 0);
                std::cout << "→ 所有軸 FAULT RESET\n";
                break;

            case 4:  // 所有軸 HOMING（mode=6）
                streaming = false;
                Homing_all(shm, CMD_HOMING, /*mode*/6, JOG_NONE, 0, 10000);
                std::cout << "→ 所有軸 HOMING (mode=6)\n";
                break;

            case 5:  // 所有軸 STOP（硬停）
                streaming = false; // 停止串流執行緒
                commit_frame_all(shm, CMD_STOP, 0, JOG_NONE, 0);
                std::cout << "→ 所有軸 STOP\n";
                break;

            case 6:  // 開始 CSP 串流
                streaming = true;
                // stream_step.store(-5, std::memory_order_relaxed);
                std::cout << "→ 開始串流 (CSP, 2ms 同拍)\n";
                break;

            case 7:  // 停止串流（不送 STOP）
                streaming = false;
                // commit_frame_all(shm, CMD_JOG, /*mode*/8, JOG_REL, +0);
                // 可用於方波
                std::cout << "→ 停止串流（保持當前狀態）\n";
                break;

            // ======= 依照需求四個案例 =======
            case 11: // 軸0 走一次相對 +100
                streaming = false;
                commit_frame_one(shm, /*axis=*/0, CMD_JOG, /*mode*/8, JOG_REL, +1000);
                std::cout<<"→ 軸0 走一次相對 +1000\n";
                break;

            case 12: // 所有軸 走一次相對 +100
                streaming = false;
                commit_frame_one(shm, /*axis=*/0, CMD_JOG, /*mode*/8, JOG_REL, -1000);
                std::cout<<"→ 軸0 走一次相對 -1000\n";
                // commit_frame_all(shm, CMD_JOG, /*mode*/8, JOG_REL, +100);
                // std::cout<<"→ 所有軸 走一次相對 +100\n";
                break;

            case 13: // 軸0 走到絕對 100
                streaming = false;
                commit_frame_one(shm, /*axis=*/0, CMD_JOG, /*mode*/8, JOG_ABS, 100);
                std::cout<<"→ 軸0 走到絕對 100\n";
                break;

            case 14: // 所有軸 走到絕對 100
                streaming = false;
                commit_frame_all(shm, CMD_JOG, /*mode*/8, JOG_ABS, 100);
                std::cout<<"→ 所有軸 走到絕對 100\n";
                break;
            // =================================================

            case 21: { // 軸0 步距 100、共 +1000（示例：以序列逐拍送）
                streaming = false;
                const int total = 1000, step = 100;
                int n = total / std::abs(step);
                int rem = total % std::abs(step);
                seq_step = +step; seq_left = n; seq_active = true;
                if (rem != 0) { // 收尾補一次
                    commit_frame_one(shm, 0, CMD_JOG, 8, JOG_REL, +rem);
                }
                std::cout << "→ 軸0 步距 100 x " << n << "（餘數 " << rem << " 已補）\n";
                break;
            }

            case 22: { // 軸1 步距 -100、共 -1000
                streaming = false;
                const int total = -1000, step = -100;
                int n = std::abs(total) / std::abs(step);
                int rem = std::abs(total) % std::abs(step);
                // 這裡示例：直接一次送 n+1 幀；也可像上面用 seq_active 的機制做成單獨執行緒
                for (int k=0; k<n; ++k)
                    commit_frame_one(shm, 1, CMD_JOG, 8, JOG_REL, step);
                if (rem != 0)
                    commit_frame_one(shm, 1, CMD_JOG, 8, JOG_REL, (total<0 ? -rem : rem));
                std::cout << "→ 軸1 步距 -100 x " << n << "（餘數 " << rem << " 已補）\n";
                break;
            }

            default:
                std::cout << "不支援: " << cmd << "\n";
        }
    }
}

int main()
{
    std::signal(SIGINT, on_sigint);
    elevate_priority();

    SharedData* shm = init_shared_memory(true);
    if (!shm) { std::cerr << "SHM attach 失敗\n"; return 1; }

    std::thread tx(streaming_thread, shm);
    command_loop(shm);

    running = false;
    if (tx.joinable()) tx.join();
    munmap(shm, sizeof(SharedData));
    std::cout << "[test_writer] bye\n";
    return 0;
}
