# ===== 編譯器/旗標 =====
CXX      = g++
CFLAGS   = -Wall -std=c++17 -I/usr/local/ethercat/include
LDFLAGS  = -lrt -lpthread
ECAT_LIB = -L/usr/local/ethercat/lib -lethercat

# ===== 檔案清單 =====
# 共享記憶體
SHMRW_SRC   = shmRW.cpp
SHM_HDR     = shmRW.h  EC_common.h servo_types.h

# EtherCAT RT/SDK
ECAT_SRC = ecat_main.cpp ecat_pdo_config.cpp ecat_rt_loop.cpp motion_control.cpp

# 上位工具
APP_SRC  = test_writer.cpp

ECAT_BIN = ecat_main
APP_BIN  = test_writer

# ===== 物件 =====
SHMRW_OBJ   = shmRW.o

# ===== 預設 =====
all: $(ECAT_BIN) $(APP_BIN)

# ===== 共用物件 =====
$(SHMRW_OBJ): $(SHMRW_SRC) $(SHM_HDR)
	$(CXX) $(CFLAGS) -c -o $@ $(SHMRW_SRC)


# ===== RT 主程式（64-bit）=====
$(ECAT_BIN): $(ECAT_SRC) $(SHMRW_OBJ) 
	$(CXX) $(CFLAGS) -o $@ $(ECAT_SRC) $(SHMRW_OBJ)  $(ECAT_LIB) $(LDFLAGS)

# ===== 上位工具 =====
#  32-bit：
$(APP_BIN): $(APP_SRC) shmRW_32.o 
	$(CXX) $(CFLAGS) -m32 -o $@ $(APP_SRC) shmRW_32.o -lrt -lpthread

shmRW_32.o: $(SHMRW_SRC) $(SHM_HDR)
	$(CXX) $(CFLAGS) -m32 -c -o $@ $(SHMRW_SRC)


clean:
	rm -f $(ECAT_BIN) $(APP_BIN) *.o