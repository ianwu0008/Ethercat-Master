#pragma once

#include <stdint.h>
#include <ecrt.h>
#include "EC_common.h"

// 全域資源 extern
extern ec_master_t *master;
extern ec_master_state_t master_state;
extern ec_domain_t *domain;
extern ec_domain_state_t domain_state;
extern ec_slave_config_t *sc[MAX_SERVO_COUNT];
extern ec_slave_config_state_t sc_state[MAX_SERVO_COUNT];
extern uint8_t *domain_pd;
extern SharedData* shm_ptr;
extern uint32_t zero;
extern int ret;

// offset arrays
extern uint32_t off_control_word[MAX_SERVO_COUNT];
extern uint32_t off_status_word[MAX_SERVO_COUNT];
extern uint32_t off_mode_display[MAX_SERVO_COUNT];
extern uint32_t off_target_pos[MAX_SERVO_COUNT];
extern uint32_t off_mode_cmd[MAX_SERVO_COUNT];
extern uint32_t off_Pos_Act_Val[MAX_SERVO_COUNT];
extern uint32_t off_error_code[MAX_SERVO_COUNT];
extern uint32_t off_Homing_Method[MAX_SERVO_COUNT];
extern uint32_t off_HomingSpeed_A[MAX_SERVO_COUNT];
// extern uint32_t off_HomingSpeed_B[MAX_SERVO_COUNT];
extern uint32_t off_Pos_error[MAX_SERVO_COUNT];

// Touch Probe offsets
extern uint32_t off_Probe_Function[MAX_SERVO_COUNT]; // 0x60B8 (UINT16, RW)
extern uint32_t off_Probe_Status  [MAX_SERVO_COUNT]; // 0x60B9 (UINT16, RO)
extern uint32_t off_Probe1_Pos    [MAX_SERVO_COUNT]; // 0x60BA (DINT,  RO)
extern uint32_t off_Probe1_Neg    [MAX_SERVO_COUNT]; // 0x60BB (DINT,  RO)
extern uint32_t off_Probe2_Pos    [MAX_SERVO_COUNT]; // 0x60BC (DINT,  RO)
extern uint32_t off_Probe2_Neg    [MAX_SERVO_COUNT]; // 0x60BD (DINT,  RO)
// 初始化 EtherCAT
int init_ecat(bool use_dc, int period_ns );


//Alias = S1 × 16 + S2
//軸名稱	Alias 	S1 (高位)   S2 (低位)	    旋鈕
// X	    101	    0x6 0x5                 S1=6, S2=5
// Y	    102	    0x6	0x6                 S1=6, S2=6
// Z	    103	    0x6	0x7	                S1=6, S2=7
// U	    104	    0x6	0x8	                S1=6, S2=8
// V	    105	    0x6	0x9	                S1=6, S2=9
// C	    106	    0x6	0xA                 S1=6, S2=10

//實體旋鈕似乎無效 
//sudo ethercat alias -p 0 101
//sudo ethercat alias -p 1 102
