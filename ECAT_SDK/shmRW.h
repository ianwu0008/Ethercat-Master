#pragma once
#include "EC_common.h"

#ifdef __cplusplus
extern "C" {
#endif

SharedData* init_shared_memory(bool verbose);
void close_shared_memory(SharedData* shm);
bool shm_validate_layout(const SharedData* shm, int required_axes, bool verbose);

enum ShmSessionHealth {
    SHM_SESSION_OK = 0,
    SHM_SESSION_ABI_MISMATCH,
    SHM_SESSION_CHANGED,
    SHM_SESSION_DAEMON_UNAVAILABLE,
    SHM_SESSION_HEARTBEAT_STALE,
    SHM_SESSION_TRANSPORT_FAULT
};

struct ShmSessionWatch {
    uint32_t session_id;
    uint32_t last_daemon_heartbeat;
    int32_t stale_cycles;
    bool heartbeat_advanced;
};

void shm_session_watch_begin(ShmSessionWatch* watch, const SharedData* shm);
ShmSessionHealth shm_session_check(ShmSessionWatch* watch,
                                   const SharedData* shm,
                                   int required_axes,
                                   int stale_limit,
                                   bool allow_transport_fault);

void write_servo_feedback(SharedData* shm, int axis,
                          int32_t actual_pos, uint16_t status_word, int8_t actual_mode,
                          int16_t error_code, bool alarm, bool servo_on);

uint32_t shm_begin_frame(SharedData* shm);
bool shm_publish_axis(SharedData* shm, const CommandEntry* cmd);
bool shm_axis_has_space(SharedData* shm, int axis);
void shm_commit_frame(SharedData* shm);
void shm_flush_axis(SharedData* shm, int axis);
void shm_flush_all_commands(SharedData* shm);
int32_t shm_take_safety_and_flush(SharedData* shm, int axis);
bool shm_pop_axis(SharedData* shm, int axis, CommandEntry* out);

bool shm_snapshot_feedback(SharedData* shm,
                           ServoData* out_array, int axes,
                           uint32_t* out_frame_seq);

#ifdef __cplusplus
}
#endif
