#ifndef __ECAT_OPTIONS_H__
#define __ECAT_OPTIONS_H__

#include "cc.h"

// ⭐️ 메일박스 설정
#define MBXSIZE          128
#define MBXSIZEBOOT      128
#define MBXBUFFERS       3
#define MBX0_sma         0x1000
#define MBX0_sml         128
#define MBX0_smc         0x26
#define MBX1_sma         0x1080
#define MBX1_sml         128
#define MBX1_smc         0x22

// ⭐️ SyncManager 2 (Outputs) 설정
#define SM2_sma          0x1100
#define SM2_smc          0x24
#define SM2_act          1

// ⭐️ SyncManager 3 (Inputs) 설정 - SM2(96바이트) 종료지점에 밀착
#define SM3_sma          0x1220
#define SM3_smc          0x20
#define SM3_act          1

// ⭐️ 매핑 엔트리 제한 확장 (32개 이상 수용)
#define MAX_MAPPINGS_SM2 48
#define MAX_MAPPINGS_SM3 24
#define MAX_RXPDO_SIZE   128
#define MAX_TXPDO_SIZE   128

#endif /* __ECAT_OPTIONS_H__ */
