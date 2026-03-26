#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <ecrt.h>
#include "EC_common.h"
#include "shmRW.h"
#include <array>

// 定義狀態與控制字元的命名常數（基於 EtherCAT 協議）
#define STATUS_MASK_BASE      0x004F
#define STATUS_MASK_ENABLE    0x006F

#define STATUS_NOT_READY      0x0000
#define STATUS_SWITCH_ON_DISABLED 0x0040
#define STATUS_READY_TO_SWITCH_ON 0x0021
#define STATUS_SWITCHED_ON    0x0023
#define STATUS_OPERATION_ENABLED 0x0027
#define STATUS_QUICK_STOP_ACTIVE 0x0007
#define STATUS_FAULT_REACTION 0x000F
#define STATUS_FAULT          0x0008

#define SW_TARGET_REACHED   0x0400  // bit10
#define SW_TARGET_VALID     0x1000  // bit12（1=目標值有效；0=忽略）
#define SW_FOLLOWING_ERROR  0x2000  // bit13


#define CONTROL_SHUTDOWN      0x0006
#define CONTROL_ENABLE        0x000F
#define CONTROL_FAULT_RESET   0x0080
#define CONTROL_DISABLE       0x0000
#define CONTROL_SWITCH_ON     0x0007
#define CONTROL_QUICK_STOP    0x0002
#define CONTROL_HOMING_START  0x0010
#define CONTROL_TARGET_VALID  0x003F
#define CONTROL_HALT          0x010F

// 重要：使用共享記憶體與 PDO 空間
extern uint8_t *domain_pd;
extern SharedData *shm_ptr;

// 函數原型
ServoState parse_state(uint16_t status);
void motion_control_update_servos();

#endif // MOTION_CONTROL_H
