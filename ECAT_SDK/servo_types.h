#ifndef SERVO_TYPES_H
#define SERVO_TYPES_H

typedef enum {
  CMD_NOP = 0,
  CMD_SERVO_ON, 
  CMD_SERVO_OFF, 
  CMD_FAULT_RESET, 
  CMD_STOP, CMD_QUICK_STOP,
  CMD_HOMING,
  CMD_SET_MODE,            // 例如 set 8 = CSP
  CMD_SET_STREAM_PARAM,    // 若未來要RT內產生波形，可先保留
  CMD_STREAM_ENABLE,
  CMD_STREAM_ARM_GROUP,
  CMD_STREAM_GO,
  CMD_JOG,                 // ABS / REL / CONTINUOUS 都歸在這
  CMD_ZERO_POS,
} ServoCommand;

typedef enum {
  JOG_NONE = 0,
  JOG_CONTINUOUS = 1,
  JOG_ABS_RAW = 3,
  JOG_ABS = 6,
  JOG_REL = 7
} JogMode;

#endif
