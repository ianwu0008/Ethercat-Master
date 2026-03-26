#pragma once
#include "EC_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// 連上既有 SHM（由 ecat_main 建立）
SharedData* init_shared_memory(bool verbose);

// 回寫伺服回饋（方便上位側讀）
void write_servo_feedback(SharedData* shm, int axis,
                          int32_t actual_pos, uint16_t status_word, int8_t actual_mode,
                          int16_t error_code, bool alarm, bool servo_on);

// 開始一幀，回傳新幀序（可寫入 CommandEntry.seq 方便除錯）
uint32_t shm_begin_frame(SharedData* shm);

// 在同一幀中，對指定軸寫入命令（只寫 slot，不動 ready_seq）
void shm_publish_axis(SharedData* shm, const CommandEntry* cmd);

// 提交本幀：把所有軸的 ready_seq 一次性設為 new_seq，最後再更新 frame_seq
void shm_commit_frame(SharedData* shm);

bool shm_snapshot_feedback(SharedData* shm,
                           ServoData* out_array, int axes,
                           uint32_t* out_frame_seq);

#ifdef __cplusplus
}
#endif
