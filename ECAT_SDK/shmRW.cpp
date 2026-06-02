#include "shmRW.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>

SharedData* init_shared_memory(bool verbose) {
    int fd = shm_open(SHM_NAME, O_RDWR, 0666);
    if (fd == -1) { perror("shm_open"); return nullptr; }

    SharedData* shm = (SharedData*)mmap(nullptr, sizeof(SharedData),
                                        PROT_READ | PROT_WRITE,
                                        MAP_SHARED, fd, 0);
    close(fd);
    if (shm == MAP_FAILED) { perror("mmap"); return nullptr; }

    if (shm->magic != SHM_MAGIC) {
        fprintf(stderr, "❌ SHM magic 不一致: %u\n", shm->magic);
        return nullptr;
    }

    if (verbose)
        printf("[SHM] Attached at %p\n", shm);

    return shm;
}


// ====== 回寫伺服回饋 ======
void write_servo_feedback(SharedData* shm, int axis,
                          int32_t actual_pos, uint16_t status_word,
                          int8_t actual_mode, int16_t error_code,
                          bool alarm, bool servo_on)
{
    if (!shm || axis < 0 || axis >= MAX_SERVO_COUNT) return;

    ServoData& s = shm->servos[axis];
    s.actual_position = actual_pos;
    s.status_word     = status_word;
    s.actual_mode     = actual_mode;
    s.error_code      = error_code;
    s.alarm           = alarm ? 1 : 0;
    s.servo_on        = servo_on ? 1 : 0;
}



// ====== Begin frame（保持相容） ======
uint32_t shm_begin_frame(SharedData* shm) {
    return shm->frame_seq.load(std::memory_order_relaxed) + 1;
}


// ====== Ring Buffer Producer：上位寫入命令 ======
static inline void shm_publish_command(SharedData* shm, const CommandEntry* cmd)
{
    if (!shm || !cmd) return;
    const int ax = cmd->servo_id;
    if (ax < 0 || ax >= MAX_SERVO_COUNT) return;

    AxisMailbox& mb = shm->mbox[ax];

    uint32_t head = mb.head.load(std::memory_order_relaxed);
    uint32_t next = (head + 1) & QUEUE_MASK;

    // 覆蓋最舊資料（永不丟最新命令）
    CommandEntry* slot = &mb.slots[head];
    *slot = *cmd;         // 複製整筆 64B
    slot->seq = head;     // optional debug

    mb.head.store(next, std::memory_order_release);
}


// ====== 舊介面：保留相容 ======
void shm_publish_axis(SharedData* shm, const CommandEntry* cmd) {
    shm_publish_command(shm, cmd);
}

void shm_commit_frame(SharedData* shm) {
    // Ring buffer 不需要 commit，保留空函式以保持相容性
}


// ====== Snapshot feedback ======
bool shm_snapshot_feedback(SharedData* shm, ServoData* out, int axes,
                           uint32_t* out_seq)
{
    if (!shm || !out || axes <= 0 || axes > MAX_SERVO_COUNT) return false;

    for (int attempt = 0; attempt < 3; ++attempt) {
        uint32_t s1 = shm->fb_seq.load(std::memory_order_acquire);

        for (int i = 0; i < axes; ++i)
            std::memcpy(&out[i], &shm->servos[i], sizeof(ServoData));

        uint32_t s2 = shm->fb_seq.load(std::memory_order_acquire);

        if (s1 == s2) {
            if (out_seq) *out_seq = s1;
            return true;
        }
    }
    return false;
}
