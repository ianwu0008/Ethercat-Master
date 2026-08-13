#include "shmRW.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>

static bool is_safety_command(int32_t command)
{
    return command == CMD_STOP ||
           command == CMD_QUICK_STOP ||
           command == CMD_SERVO_OFF;
}

static int safety_rank(int32_t command)
{
    if (command == CMD_SERVO_OFF) return 3;
    if (command == CMD_QUICK_STOP) return 2;
    if (command == CMD_STOP) return 1;
    return 0;
}

bool shm_validate_layout(const SharedData* shm, int required_axes, bool verbose)
{
    if (!shm) return false;

    const SharedHeader& h = shm->header;
    const uint32_t active = h.active_axis_count.load(std::memory_order_acquire);
    const bool ok =
        h.magic == SHM_MAGIC &&
        h.abi_version == SHM_ABI_VERSION &&
        h.shared_data_size == sizeof(SharedData) &&
        h.max_axis_count == MAX_SERVO_COUNT &&
        required_axes >= 0 &&
        required_axes <= MAX_SERVO_COUNT &&
        (required_axes == 0 || active >= static_cast<uint32_t>(required_axes)) &&
        active <= MAX_SERVO_COUNT;

    if (!ok && verbose) {
        fprintf(stderr,
                "[SHM] ABI rejected: magic=0x%08x version=%u size=%u "
                "max_axes=%u active_axes=%u required_axes=%d\n",
                h.magic, h.abi_version, h.shared_data_size,
                h.max_axis_count, active, required_axes);
    }
    return ok;
}

SharedData* init_shared_memory(bool verbose)
{
    const int fd = shm_open(SHM_NAME, O_RDWR, 0666);
    if (fd == -1) {
        if (verbose) perror("shm_open");
        return nullptr;
    }

    struct stat st {};
    if (fstat(fd, &st) != 0) {
        if (verbose) perror("fstat");
        close(fd);
        return nullptr;
    }
    if (st.st_size != static_cast<off_t>(sizeof(SharedData))) {
        if (verbose) {
            fprintf(stderr, "[SHM] size rejected: file=%lld expected=%zu\n",
                    static_cast<long long>(st.st_size), sizeof(SharedData));
        }
        close(fd);
        return nullptr;
    }

    void* mapping = mmap(nullptr, sizeof(SharedData),
                         PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (mapping == MAP_FAILED) {
        if (verbose) perror("mmap");
        return nullptr;
    }

    SharedData* shm = static_cast<SharedData*>(mapping);
    if (!shm_validate_layout(shm, 0, verbose)) {
        munmap(shm, sizeof(SharedData));
        return nullptr;
    }

    if (verbose) {
        printf("[SHM] attached session=%u axes=%u at %p\n",
               shm->header.session_id,
               shm->header.active_axis_count.load(std::memory_order_acquire),
               static_cast<void*>(shm));
    }
    return shm;
}

void close_shared_memory(SharedData* shm)
{
    if (shm) munmap(shm, sizeof(SharedData));
}

void shm_session_watch_begin(ShmSessionWatch* watch, const SharedData* shm)
{
    if (!watch) return;
    watch->session_id = shm ? shm->header.session_id : 0;
    watch->last_daemon_heartbeat =
        shm ? shm->header.daemon_heartbeat.load(std::memory_order_acquire) : 0;
    watch->stale_cycles = 0;
    watch->heartbeat_advanced = false;
}

ShmSessionHealth shm_session_check(ShmSessionWatch* watch,
                                   const SharedData* shm,
                                   int required_axes,
                                   int stale_limit,
                                   bool allow_transport_fault)
{
    if (!watch || !shm || !shm_validate_layout(shm, required_axes, false))
        return SHM_SESSION_ABI_MISMATCH;
    if (shm->header.session_id != watch->session_id)
        return SHM_SESSION_CHANGED;

    const uint32_t fault =
        shm->header.transport_fault.load(std::memory_order_acquire);
    if (!allow_transport_fault && fault != SHM_FAULT_NONE)
        return SHM_SESSION_TRANSPORT_FAULT;

    const uint32_t state =
        shm->header.daemon_state.load(std::memory_order_acquire);
    if (state == SHM_DAEMON_INITIALIZING || state == SHM_DAEMON_STOPPING ||
        (!allow_transport_fault && state == SHM_DAEMON_FAULT))
        return SHM_SESSION_DAEMON_UNAVAILABLE;

    const uint32_t heartbeat =
        shm->header.daemon_heartbeat.load(std::memory_order_acquire);
    if (heartbeat == watch->last_daemon_heartbeat) {
        if (++watch->stale_cycles >= stale_limit)
            return SHM_SESSION_HEARTBEAT_STALE;
    } else {
        watch->last_daemon_heartbeat = heartbeat;
        watch->stale_cycles = 0;
        watch->heartbeat_advanced = true;
    }
    return SHM_SESSION_OK;
}


void write_servo_feedback(SharedData* shm, int axis,
                          int32_t actual_pos, uint16_t status_word,
                          int8_t actual_mode, int16_t error_code,
                          bool alarm, bool servo_on)
{
    if (!shm || axis < 0 || axis >= MAX_SERVO_COUNT) return;

    ServoData& s = shm->servos[axis];
    s.actual_position = actual_pos;
    s.status_word = status_word;
    s.actual_mode = actual_mode;
    s.error_code = error_code;
    s.alarm = alarm ? 1 : 0;
    s.servo_on = servo_on ? 1 : 0;
}

uint32_t shm_begin_frame(SharedData* shm)
{
    if (!shm) return 0;
    return shm->header.producer_sequence.load(std::memory_order_relaxed) + 1;
}

bool shm_publish_axis(SharedData* shm, const CommandEntry* cmd)
{
    if (!shm || !cmd) return false;
    const int axis = cmd->servo_id;
    if (axis < 0 || axis >= MAX_SERVO_COUNT) return false;

    AxisMailbox& mb = shm->mbox[axis];
    if (is_safety_command(cmd->command)) {
        int32_t current = mb.safety_command.load(std::memory_order_relaxed);
        while (safety_rank(cmd->command) > safety_rank(current) &&
               !mb.safety_command.compare_exchange_weak(
                   current, cmd->command,
                   std::memory_order_release,
                   std::memory_order_relaxed)) {
        }
        return true;
    }

    const uint32_t head = mb.head.load(std::memory_order_relaxed);
    const uint32_t next = (head + 1U) & QUEUE_MASK;
    const uint32_t tail = mb.tail.load(std::memory_order_acquire);
    if (next == tail) {
        shm->header.transport_fault.store(SHM_FAULT_RING_FULL,
                                          std::memory_order_release);
        return false;
    }

    mb.slots[head] = *cmd;
    mb.head.store(next, std::memory_order_release);
    return true;
}

bool shm_axis_has_space(SharedData* shm, int axis)
{
    if (!shm || axis < 0 || axis >= MAX_SERVO_COUNT) return false;
    AxisMailbox& mb = shm->mbox[axis];
    const uint32_t head = mb.head.load(std::memory_order_relaxed);
    const uint32_t tail = mb.tail.load(std::memory_order_acquire);
    return ((head + 1U) & QUEUE_MASK) != tail;
}

void shm_commit_frame(SharedData* shm)
{
    if (!shm) return;
    shm->header.producer_sequence.fetch_add(1, std::memory_order_release);
    shm->header.producer_heartbeat.fetch_add(1, std::memory_order_release);
}

void shm_flush_axis(SharedData* shm, int axis)
{
    if (!shm || axis < 0 || axis >= MAX_SERVO_COUNT) return;
    AxisMailbox& mb = shm->mbox[axis];
    const uint32_t head = mb.head.load(std::memory_order_acquire);
    mb.tail.store(head, std::memory_order_release);
}

void shm_flush_all_commands(SharedData* shm)
{
    if (!shm) return;
    for (int axis = 0; axis < MAX_SERVO_COUNT; ++axis) {
        shm_flush_axis(shm, axis);
        shm->mbox[axis].safety_command.store(CMD_NOP, std::memory_order_release);
    }
}

int32_t shm_take_safety_and_flush(SharedData* shm, int axis)
{
    if (!shm || axis < 0 || axis >= MAX_SERVO_COUNT) return CMD_NOP;
    AxisMailbox& mb = shm->mbox[axis];
    const int32_t command =
        mb.safety_command.exchange(CMD_NOP, std::memory_order_acquire);
    if (command != CMD_NOP) shm_flush_axis(shm, axis);
    return command;
}

bool shm_pop_axis(SharedData* shm, int axis, CommandEntry* out)
{
    if (!shm || !out || axis < 0 || axis >= MAX_SERVO_COUNT) return false;
    AxisMailbox& mb = shm->mbox[axis];
    const uint32_t tail = mb.tail.load(std::memory_order_relaxed);
    const uint32_t head = mb.head.load(std::memory_order_acquire);
    if (tail == head) return false;
    *out = mb.slots[tail];
    mb.tail.store((tail + 1U) & QUEUE_MASK, std::memory_order_release);
    return true;
}

bool shm_snapshot_feedback(SharedData* shm, ServoData* out, int axes,
                           uint32_t* out_seq)
{
    if (!shm || !out || axes <= 0 || axes > MAX_SERVO_COUNT) return false;

    for (int attempt = 0; attempt < 4; ++attempt) {
        const uint32_t before = shm->fb_seq.load(std::memory_order_seq_cst);
        if (before & 1U) continue;

        for (int i = 0; i < axes; ++i) {
            std::memcpy(&out[i], &shm->servos[i], sizeof(ServoData));
        }

        std::atomic_thread_fence(std::memory_order_seq_cst);
        const uint32_t after = shm->fb_seq.load(std::memory_order_seq_cst);
        if (before == after && !(after & 1U)) {
            if (out_seq) *out_seq = after;
            return true;
        }
    }
    return false;
}
