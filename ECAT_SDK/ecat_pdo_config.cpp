// ecat_pdo_config.cpp
#include "ecat_pdo_config.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "EC_common.h"   // 提供 MAX_SERVO_COUNT, ARRAY_SIZE 等

// ====== 供你依實機調整 ======
#define VENDOR_ID     0x00000539
#define PRODUCT_CODE  0x02200901

static const uint16_t AXIS_ALIAS[MAX_SERVO_COUNT] = {
    101, 102 , 103 ,104 ,105 // 依實機填滿前 MAX_SERVO_COUNT 個
};

// ====== 全域物件 ======
ec_master_t* master = nullptr;
ec_master_state_t master_state = {};
ec_domain_t* domain = nullptr;
ec_domain_state_t domain_state = {};
ec_slave_config_t* sc[MAX_SERVO_COUNT] = {nullptr};
ec_slave_config_state_t sc_state[MAX_SERVO_COUNT] = {};
uint8_t* domain_pd = nullptr;

// PDO offsets
uint32_t off_control_word [MAX_SERVO_COUNT];
uint32_t off_status_word  [MAX_SERVO_COUNT];
uint32_t off_mode_cmd     [MAX_SERVO_COUNT];
uint32_t off_mode_display [MAX_SERVO_COUNT];
uint32_t off_target_pos   [MAX_SERVO_COUNT];
uint32_t off_Pos_Act_Val  [MAX_SERVO_COUNT];
uint32_t off_error_code   [MAX_SERVO_COUNT];
uint32_t off_Homing_Method[MAX_SERVO_COUNT];
uint32_t off_Pos_error    [MAX_SERVO_COUNT];

// Touch Probe offsets
uint32_t off_Probe_Function[MAX_SERVO_COUNT]; // 0x60B8 (UINT16, RW)
uint32_t off_Probe_Status  [MAX_SERVO_COUNT]; // 0x60B9 (UINT16, RO)
uint32_t off_Probe1_Pos    [MAX_SERVO_COUNT]; // 0x60BA (DINT,  RO)
uint32_t off_Probe1_Neg    [MAX_SERVO_COUNT]; // 0x60BB (DINT,  RO)
uint32_t off_Probe2_Pos    [MAX_SERVO_COUNT]; // 0x60BC (DINT,  RO)
uint32_t off_Probe2_Neg    [MAX_SERVO_COUNT]; // 0x60BD (DINT,  RO)

// ====== PDO 定義 ======
static ec_pdo_entry_info_t rx_pdo_entries[] = {
    {0x6040, 0x00, 16}, // Controlword             UINT16
    {0x6060, 0x00,  8}, // Modes of Operation      INT8
    {0x6098, 0x00,  8}, // Homing Method           INT8
    {0x607A, 0x00, 32}, // Target Position         INT32
    // {0x60B8, 0x00, 16}, // Touch Probe Function    UINT16 
};

static ec_pdo_entry_info_t tx_pdo_entries[] = {
    {0x6041, 0x00, 16}, // Statusword              UINT16
    {0x6061, 0x00,  8}, // Modes Display           INT8
    {0x6064, 0x00, 32}, // Position Actual         INT32
    {0x603F, 0x00, 16}, // Error Code              UINT16
    {0x60F4, 0x00, 32}, // Position Error          INT32
    {0x60B9, 0x00, 16}, // Touch Probe Status      UINT16  
    {0x60BA, 0x00, 32}, // Probe1 positive edge    DINT   
    {0x60BB, 0x00, 32}, // Probe1 negative edge    DINT   
    {0x60BC, 0x00, 32}, // Probe2 positive edge    DINT   
    {0x60BD, 0x00, 32}, // Probe2 negative edge    DINT   
};

static ec_pdo_info_t pdos[] = {
    {0x1600, (uint8_t)ARRAY_SIZE(rx_pdo_entries), rx_pdo_entries}, // RxPDO
    {0x1A00, (uint8_t)ARRAY_SIZE(tx_pdo_entries), tx_pdo_entries}, // TxPDO
};

static ec_sync_info_t syncs[] = {
    {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
    {1, EC_DIR_INPUT , 0, nullptr, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, pdos    , EC_WD_ENABLE },
    {3, EC_DIR_INPUT , 1, pdos + 1, EC_WD_DISABLE},
    {0xFF}
};

// ====== 配置 slaves（僅 alias 定址）+ PDO ======
static int configure_slaves_and_pdos(void) {
    for (int i = 0; i < MAX_SERVO_COUNT; ++i) {
        const uint16_t alias = AXIS_ALIAS[i]; // 依序取每軸 alias
        sc[i] = ecrt_master_slave_config(master, alias, /*position*/0,
                                         VENDOR_ID, PRODUCT_CODE);
        if (!sc[i]) {
            fprintf(stderr, "❌ ecrt_slave_config(alias=%u) 失敗\n", alias);
            return -1;
        }
        if (ecrt_slave_config_pdos(sc[i], EC_END, syncs)) {
            fprintf(stderr, "❌ ecrt_slave_config_pdos(alias=%u) 失敗\n", alias);
            return -1;
        }
        std ::cout << "[PDO] Slave " << i << " (alias=" << alias << ") PDO 配置完成\n";

        // === 在此下發 SDO ===        
        //與HOMING 相關,
        // ecrt_slave_config_sdo32(sc[i],  0x6099, 0x01, 200000);   // speed during search for switch（fast）
        // ecrt_slave_config_sdo32(sc[i],  0x6099, 0x02, 20000);    // speed during search for zero（slow）
        // ecrt_slave_config_sdo32(sc[i],  0x609A, 0x00, 20000);    // acceleration
        // ecrt_slave_config_sdo32(sc[i],  0x607C, 0x00, 0);        // home offset     
        ecrt_slave_config_sdo16(sc[i],  0x60B8, 0x00, 0x0017);      //  Touch Probe Function: Bit0+Bit1+Bit4+Bit5=0x17
        // sleep(0.1); // 避免 SDO 下太快衝突
    }
    return 0;
}

// ====== Domain 註冊 PDO entries（只針對現有軸數）======
static int register_pdo_entries(void) {
    // 每軸 15 個 entry（含 Touch Probe）+ 終止
    static ec_pdo_entry_reg_t regs[MAX_SERVO_COUNT * 14 + 1]; 
    int idx = 0;

    for (int i = 0; i < MAX_SERVO_COUNT; ++i) {
        uint16_t alias = AXIS_ALIAS[i];
        uint16_t pos   = 0; // alias 模式固定填 0

        regs[idx++] = (ec_pdo_entry_reg_t){ alias, pos, VENDOR_ID, PRODUCT_CODE, 0x6040, 0, &off_control_word[i]  };
        regs[idx++] = (ec_pdo_entry_reg_t){ alias, pos, VENDOR_ID, PRODUCT_CODE, 0x6041, 0, &off_status_word[i]   };
        regs[idx++] = (ec_pdo_entry_reg_t){ alias, pos, VENDOR_ID, PRODUCT_CODE, 0x6060, 0, &off_mode_cmd[i]      };
        regs[idx++] = (ec_pdo_entry_reg_t){ alias, pos, VENDOR_ID, PRODUCT_CODE, 0x6061, 0, &off_mode_display[i]  };
        regs[idx++] = (ec_pdo_entry_reg_t){ alias, pos, VENDOR_ID, PRODUCT_CODE, 0x607A, 0, &off_target_pos[i]    };
        regs[idx++] = (ec_pdo_entry_reg_t){ alias, pos, VENDOR_ID, PRODUCT_CODE, 0x6064, 0, &off_Pos_Act_Val[i]   };
        regs[idx++] = (ec_pdo_entry_reg_t){ alias, pos, VENDOR_ID, PRODUCT_CODE, 0x603F, 0, &off_error_code[i]    };
        regs[idx++] = (ec_pdo_entry_reg_t){ alias, pos, VENDOR_ID, PRODUCT_CODE, 0x6098, 0, &off_Homing_Method[i] };
        regs[idx++] = (ec_pdo_entry_reg_t){ alias, pos, VENDOR_ID, PRODUCT_CODE, 0x60F4, 0, &off_Pos_error[i]     };

        // Touch Probe entries
        // regs[idx++] = (ec_pdo_entry_reg_t){ alias, pos, VENDOR_ID, PRODUCT_CODE, 0x60B8, 0, &off_Probe_Function[i] };
        regs[idx++] = (ec_pdo_entry_reg_t){ alias, pos, VENDOR_ID, PRODUCT_CODE, 0x60B9, 0, &off_Probe_Status[i]   };
        regs[idx++] = (ec_pdo_entry_reg_t){ alias, pos, VENDOR_ID, PRODUCT_CODE, 0x60BA, 0, &off_Probe1_Pos[i]     };
        regs[idx++] = (ec_pdo_entry_reg_t){ alias, pos, VENDOR_ID, PRODUCT_CODE, 0x60BB, 0, &off_Probe1_Neg[i]     };
        regs[idx++] = (ec_pdo_entry_reg_t){ alias, pos, VENDOR_ID, PRODUCT_CODE, 0x60BC, 0, &off_Probe2_Pos[i]     };
        regs[idx++] = (ec_pdo_entry_reg_t){ alias, pos, VENDOR_ID, PRODUCT_CODE, 0x60BD, 0, &off_Probe2_Neg[i]     };
    }
    regs[idx] = (ec_pdo_entry_reg_t){0}; // terminator

    return ecrt_domain_reg_pdo_entry_list(domain, regs);
}

// ====== DC 設定（Sync0；均分相位）======
static int configure_dc_all(bool use_dc, int period_ns) {
    if (!use_dc) return 0;

    for (int i = 0; i < MAX_SERVO_COUNT; ++i) {
        uint32_t shift_time = period_ns / (MAX_SERVO_COUNT + 1);
        // Ensure shift_time is a multiple of 62500
        shift_time = ((shift_time + 31250) / 62500) * 62500; // Round to nearest multiple of 62500
        
        std::cout << "shift_time=" << (i+1)*shift_time << std::endl;
        if (ecrt_slave_config_dc(sc[i], 0x0300, period_ns, (i+1)*shift_time, 0, 0)) {
            fprintf(stderr, "❌ ecrt_slave_config_dc(%d) 失敗\n", i);
            return -1;
        }
        
    }
    // shift_time 寫入失敗 使用軟體設定值!!!!
    if (sc[0]) ecrt_master_select_reference_clock(master, sc[0]);
    return 0;
}

// ====== 對外初始化 ======
int init_ecat(bool use_dc, int period_ns) {
    
    master = ecrt_request_master(0);
    if (!master) { fprintf(stderr, "❌ ecrt_request_master 失敗\n"); return -1; }

    domain = ecrt_master_create_domain(master);
    if (!domain) { fprintf(stderr, "❌ ecrt_master_create_domain 失敗\n"); return -1; }

    if (configure_slaves_and_pdos() < 0) return -1;

    if (register_pdo_entries() < 0) {
        fprintf(stderr, "❌ PDO regs 配置失敗\n");
        return -1;
    }

    if (configure_dc_all(use_dc, period_ns) < 0) return -1;

    if (ecrt_master_activate(master)) {
        fprintf(stderr, "❌ ecrt_master_activate 失敗\n");
        return -1;
    }
    
    domain_pd = ecrt_domain_data(domain);
    if (!domain_pd) { fprintf(stderr, "❌ 取得 domain_pd 失敗\n"); return -1; }

    return 0;
}
