//ecat_main.cpp
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sched.h>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#include <sys/stat.h> // 用於 fstat

#include "EC_common.h"
#include "ecat_pdo_config.h"
#include "ecat_rt_loop.h"

/*
+--------------------+------------------------------------------------------------------+
| 項目               | 說明                                                             |
+--------------------+------------------------------------------------------------------+
| 日期               | 2025-08-15                              |
+--------------------+------------------------------------------------------------------+
| 版本               | v1.09  (IgH EtherCAT Master 1.6)                                  |
+--------------------+------------------------------------------------------------------+
| 作者               | Wu                                                               |
+--------------------+------------------------------------------------------------------+
| 主要函式            | run_rt_loop(): 實時 EtherCAT 主循環，PDO 讀寫、DC 同步             |
|                    | 共享記憶體、motion SDK                                            |
+--------------------+------------------------------------------------------------------+
| 背景環境            | - 運行於 Linux (PREEMPT_RT )                                     |
|                    |   避免中斷，確保週期穩定 (<1 µs jitter)                             |
|                    | - RT 優先級 99 (SCHED_FIFO): 最高實時優先級，確保循環               |
|                    | - 安裝時 移除EOE                                                  |
|                    | - 單核心 舊主板 1核心                                             |
+--------------------+------------------------------------------------------------------+
| 變更歷史           | - 2025-07-31: 初始版本，整合 DC PI 控制器與 jitter 補償。       |
+--------------------+------------------------------------------------------------------+
*/

// 保留修改紀錄 重要  Pn407速度上限 10000->2000  2025/08/08

bool use_dc = true;
static bool is_shm_creator = false;

// 控制主迴圈的旗標
volatile sig_atomic_t running = 1;
void signal_handler(int sig) {
    running = 0;
}

SharedData* shm_ptr = nullptr;
SharedData* setup_shared_memory(bool verbose) {
    shm_unlink(SHM_NAME);  // ⚠️ 僅限測試階段使用
    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_EXCL | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open create");
        return nullptr;
    }

    if (ftruncate(shm_fd, sizeof(SharedData)) == -1) {
        perror("ftruncate");
        close(shm_fd);
        return nullptr;
    }

    struct stat shm_stat;
    fstat(shm_fd, &shm_stat);
    if (verbose)
        printf("[DEBUG] SHM inode=%lu, size=%ld\n", shm_stat.st_ino, shm_stat.st_size);

    SharedData* shm_ptr = (SharedData*)mmap(NULL, sizeof(SharedData),
                                        PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_ptr == MAP_FAILED) {
        perror("mmap");
        close(shm_fd);
        return nullptr;
    }
    close(shm_fd);

    shm_ptr->magic = SHM_MAGIC;


    memset(shm_ptr->servos, 0, sizeof(shm_ptr->servos));
    msync(shm_ptr, sizeof(SharedData), MS_SYNC);

    if (verbose)
        printf("✅ 共享記憶體初始化完成 at %p\n", shm_ptr);

    return shm_ptr;
}

// 設定實時優先權
void elevate_to_realtime() {
    struct sched_param sp;
    sp.sched_priority = 99;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) == -1) {
        perror("sched_setscheduler");
    } else {
        printf("✅ Realtime thread priority set to %d\n", sp.sched_priority);
    }
}

// CPU 綁定
void bind_to_cpu(int cpu) {
    cpu_set_t *mask = CPU_ALLOC(1024);  // 支援最多 1024 個 CPU
    if (!mask) {
        perror("CPU_ALLOC");
        return;
    }

    size_t size = CPU_ALLOC_SIZE(1024);
    CPU_ZERO_S(size, mask);
    CPU_SET_S(cpu, size, mask);

    if (sched_setaffinity(0, size, mask) == -1) {
        perror("sched_setaffinity");
        printf("❌ 無法綁定到 CPU %d (可能是無效或不存在)\n", cpu);
    } else {
        printf("✅ CPU 綁定成功，執行於 CPU%d\n", cpu);
    }

    CPU_FREE(mask);
}


void shutdown_servo_cleanly() {
    printf("[SHUTDOWN] 開始關閉所有 Servo...\n");

    if (!master || !domain_pd) {
        printf("[SHUTDOWN] ⛔ 無 Master 或未初始化 domain_pd，跳過清理。\n");
        return;
    }

    // Step 1: 停止所有 Servo：送出 Shutdown（0x06）
    for (int i = 0; i < MAX_SERVO_COUNT; i++) {
        EC_WRITE_U16(domain_pd + off_control_word[i], 0x0006);
    }
    ecrt_domain_queue(domain);
    ecrt_master_send(master);
    usleep(2000); // 等 2ms

    // Step 2: Switch Off（0x00）
    for (int i = 0; i < MAX_SERVO_COUNT; i++) {
        EC_WRITE_U16(domain_pd + off_control_word[i], 0x0000);
    }
    ecrt_domain_queue(domain);
    ecrt_master_send(master);
    usleep(2000);

    // Step 3: 停止 domain 處理
    if (domain) {
        ecrt_domain_process(domain);
        domain = NULL;
        domain_pd = NULL;
    }

    // Step 4: 釋放 Master
    printf("[SHUTDOWN] 釋放 EtherCAT Master...\n");
    ecrt_release_master(master);
    master = NULL;

    printf("[SHUTDOWN] ✅ 所有 Servo 關閉完成，Master 與資源已清理。\n");
}

// 清理共享記憶體和其他資源
void cleanup_resources() {
    printf("[DEBUG] Cleaning up resources...\n");
    if (shm_ptr) {
        munmap(shm_ptr, sizeof(SharedData));
        printf("[DEBUG] SHM unmapped.\n");
        if (is_shm_creator) {
            shm_unlink(SHM_NAME);
            printf("[DEBUG] SHM file unlinked.\n");
        }
        shm_ptr = NULL;
    }
    munlockall();
    printf("[DEBUG] Memory unlocked.\n");
}

int main() {
    signal(SIGINT, signal_handler); // 註冊 Ctrl+C 處理函式
    // 共享記憶體建立 + mutex 初始化
    shm_ptr = setup_shared_memory(true);
    if (!shm_ptr) {
        fprintf(stderr, "❌ 建立共享記憶體失敗！\n");
        return -1;
    }
    is_shm_creator = true;
    
    mlockall(MCL_CURRENT | MCL_FUTURE); // 鎖住記憶體
    elevate_to_realtime(); // 提昇為即時排程
    bind_to_cpu(0); // 綁定到 CPU1

    // 初始化 EtherCAT
    if (init_ecat(true, PERIOD_NS) != 0) {
        fprintf(stderr, "❌ EtherCAT 初始化失敗！\n");
        return -1;
    }
    run_rt_loop();

    // 收尾：Shutdown Servo
    shutdown_servo_cleanly();
    cleanup_resources();

    return 0;
}

//PDO 分為tx rx 效能比較好嗎? 或其他方案? N
//共享記憶體要扁平化 同一瞬間 對6軸下命令 Y
//通訊中斷 堵塞卡死 懷疑接收端堵塞 ?