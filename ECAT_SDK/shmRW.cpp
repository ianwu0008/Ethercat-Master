#include "shmRW.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>

SharedData* init_shared_memory(bool verbose) {
    int fd = shm_open(SHM_NAME, O_RDWR, 0666);
    if (fd == -1) { perror("shm_open"); return nullptr; }

    SharedData* shm = (SharedData*)mmap(nullptr, sizeof(SharedData),
                                        PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (shm == MAP_FAILED) { perror("mmap"); return nullptr; }

    if (shm->magic != SHM_MAGIC) {
        fprintf(stderr, "❌ SHM magic 不一致: %u\n", shm->magic);
        return nullptr;
    }
    if (verbose) printf("[SHM] Attached at %p\n", shm);
    return shm;
}

void write_servo_feedback(SharedData* shm, int axis,
                          int32_t actual_pos, uint16_t status_word, int8_t actual_mode,
                          int16_t error_code, bool alarm, bool servo_on) {
    if (!shm || axis < 0 || axis >= MAX_SERVO_COUNT) return;
    ServoData& s = shm->servos[axis];
    s.actual_position = actual_pos;
    s.actual_velocity = 0; // 若無速度 PDO，先 0
    s.status_word     = status_word;
    s.actual_mode     = actual_mode;
    s.error_code      = error_code;
    s.alarm           = alarm ? 1 : 0;
    s.servo_on        = servo_on ? 1 : 0;
}


static inline void store_release_u32(std::atomic<uint32_t>& a, uint32_t v) {
    a.store(v, std::memory_order_release);
}

uint32_t shm_begin_frame(SharedData* shm) {
    // 下一幀 = 目前 frame_seq + 1（relaxed 即可）
    return shm->frame_seq.load(std::memory_order_relaxed) + 1;
}

void shm_publish_axis(SharedData* shm, const CommandEntry* cmd) {
    if (!shm || !cmd) return;
    int ax = cmd->servo_id;
    if (ax < 0 || ax >= MAX_SERVO_COUNT) return;
    shm->mbox[ax].slot = *cmd;  // 只寫 slot，不動序號
}

void shm_commit_frame(SharedData* shm) {
    const uint32_t new_seq = shm->frame_seq.load(std::memory_order_relaxed) + 1;

    // 先讓每軸 ready_seq = new_seq（release），確保消費端 acquire 可見完整 slot
    for (int ax = 0; ax < MAX_SERVO_COUNT; ++ax) {
        // 把 slot.seq 也寫成 new_seq（方便除錯，不是必須）
        shm->mbox[ax].slot.seq = new_seq;
        store_release_u32(shm->mbox[ax].ready_seq, new_seq);
    }
    // 最後才更新全域 frame_seq（release）
    shm->frame_seq.store(new_seq, std::memory_order_release);
}

bool shm_snapshot_feedback(SharedData* shm, ServoData* out, int axes, uint32_t* out_seq){
    if (!shm || !out || axes<=0 || axes>MAX_SERVO_COUNT) return false;
    for (int attempt=0; attempt<3; ++attempt){
        uint32_t s1 = shm->fb_seq.load(std::memory_order_acquire);
        for (int i=0;i<axes;++i) std::memcpy(&out[i], &shm->servos[i], sizeof(ServoData));
        uint32_t s2 = shm->fb_seq.load(std::memory_order_acquire);
        if (s1 == s2) { if (out_seq) *out_seq = s1; return true; }
    }
    return false;
}