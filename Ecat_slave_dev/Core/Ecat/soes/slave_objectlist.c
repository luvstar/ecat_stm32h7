#include "esc_coe.h"
#include "utypes.h"
#include <stddef.h>

_Rbuffer Rb;
_Wbuffer Wb;

#ifndef HW_REV
#define HW_REV "1.0"
#endif

#ifndef SW_REV
#define SW_REV "1.0"
#endif

// -------------------------------------------------------------
// [추가] SOES 내부 연산용 전역 변수 및 배열 선언 (NULL 방지)
// -------------------------------------------------------------
uint8_t rxpdo_map_count = 18;  // 0x1600의 SubIndex 갯수
uint8_t txpdo_map_count = 6;   // 0x1A00의 SubIndex 갯수
uint8_t sm2_assign_count = 1;  // 0x1C12에 할당된 PDO 갯수
uint8_t sm3_assign_count = 1;  // 0x1C13에 할당된 PDO 갯수

uint8_t rxpdo_target_count = 18; // 0x7000 갯수
uint8_t txpdo_actual_count = 6;  // 0x6000 갯수

uint16_t sm2_pdo_assign = 0x1600;
uint16_t sm3_pdo_assign = 0x1A00;

// SM 통신 타입 (0x1C00)
uint8_t sm_comm_type_count = 4;
uint8_t sm_comm_types[4] = {1, 2, 3, 4}; // SM0~SM3 통신 타입

// Identity Object (0x1018) 변수
uint8_t id_count = 4;
uint32_t vendor_id = 0x1337;
uint32_t product_code = 1234;
uint32_t revision_num = 0;
uint32_t serial_num = 0;

// [핵심] RxPDO (0x1600) 실제 매핑 데이터 배열
uint32_t rxpdo_mapping_entries[18] = {
  0x70000120, 0x70000220, 0x70000320, 0x70000420, // Target Pos 1~4
  0x70000508, 0x70000608, 0x70000708, 0x70000808, // Direction 1~4
  0x70000908, 0x70000A08, 0x70000B08, 0x70000C08, // Target Speed 1~4
  0x70000D08, 0x70000E08, 0x70000F08, 0x70001008, // Accel Pattern 1~4
  0x70001110, 0x70001210                          // Control Word, Dummy
};

// [핵심] TxPDO (0x1A00) 실제 매핑 데이터 배열
uint32_t txpdo_mapping_entries[6] = {
  0x60000120, 0x60000220, 0x60000320, 0x60000420, // Actual Pos 1~4
  0x60000510, 0x60000610                          // Status Word, Error Code
};

// -------------------------------------------------------------
// 1. Object Dictionary 이름(String) 정의
// -------------------------------------------------------------
static const char acName1000[] = "Device Type";
static const char acName1008[] = "Device Name";
static const char acName1009[] = "Hardware Version";
static const char acName100A[] = "Software Version";

static const char acName1018[] = "Identity Object";
static const char acName1018_00[] = "Max SubIndex";
static const char acName1018_01[] = "Vendor ID";
static const char acName1018_02[] = "Product Code";
static const char acName1018_03[] = "Revision Number";
static const char acName1018_04[] = "Serial Number";

static const char acNameMapping[] = "PDO Mapping Entry";

static const char acName1600[] = "RxPDO Mapping";
static const char acName1600_00[] = "Max SubIndex";
static const char acName1A00[] = "TxPDO Mapping";
static const char acName1A00_00[] = "Max SubIndex";

static const char acName1C00[] = "Sync Manager Communication Type";
static const char acName1C00_00[] = "Max SubIndex";
static const char acName1C00_01[] = "Communication Type SM0";
static const char acName1C00_02[] = "Communication Type SM1";
static const char acName1C00_03[] = "Communication Type SM2";
static const char acName1C00_04[] = "Communication Type SM3";

static const char acName1C12[] = "Sync Manager 2 PDO Assignment";
static const char acName1C12_00[] = "Max SubIndex";
static const char acName1C12_01[] = "PDO Mapping";

static const char acName1C13[] = "Sync Manager 3 PDO Assignment";
static const char acName1C13_00[] = "Max SubIndex";
static const char acName1C13_01[] = "PDO Mapping";

static const char acName6000[] = "TxPDO 4-Axis Actual Data";
static const char acName6000_00[] = "Max SubIndex";
static const char acName6000_01[] = "Actual Pos Axis 1";
static const char acName6000_02[] = "Actual Pos Axis 2";
static const char acName6000_03[] = "Actual Pos Axis 3";
static const char acName6000_04[] = "Actual Pos Axis 4";
static const char acName6000_05[] = "Status Word";
static const char acName6000_06[] = "Error Code";

static const char acName7000[] = "RxPDO 4-Axis Target Data";
static const char acName7000_00[] = "Max SubIndex";
static const char acName7000_01[] = "Target Pos Axis 1";
static const char acName7000_02[] = "Target Pos Axis 2";
static const char acName7000_03[] = "Target Pos Axis 3";
static const char acName7000_04[] = "Target Pos Axis 4";
static const char acName7000_05[] = "Direction Axis 1";
static const char acName7000_06[] = "Direction Axis 2";
static const char acName7000_07[] = "Direction Axis 3";
static const char acName7000_08[] = "Direction Axis 4";
static const char acName7000_09[] = "Target Speed Axis 1";
static const char acName7000_0A[] = "Target Speed Axis 2";
static const char acName7000_0B[] = "Target Speed Axis 3";
static const char acName7000_0C[] = "Target Speed Axis 4";
static const char acName7000_0D[] = "Accel Pattern Axis 1";
static const char acName7000_0E[] = "Accel Pattern Axis 2";
static const char acName7000_0F[] = "Accel Pattern Axis 3";
static const char acName7000_10[] = "Accel Pattern Axis 4";
static const char acName7000_11[] = "Control Word";

// -------------------------------------------------------------
// 2. SDO 데이터 구조체 맵핑
// -------------------------------------------------------------
const _objd SDO1000[] = {
  {0x0, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1000, 0x01901389, NULL},
};
const _objd SDO1008[] = {
  {0x0, DTYPE_VISIBLE_STRING, 216, ATYPE_RO, acName1008, 0, "STM32H753 4-Axis Controller"},
};
const _objd SDO1009[] = {
  {0x0, DTYPE_VISIBLE_STRING, 0, ATYPE_RO, acName1009, 0, HW_REV},
};
const _objd SDO100A[] = {
  {0x0, DTYPE_VISIBLE_STRING, 0, ATYPE_RO, acName100A, 0, SW_REV},
};

const _objd SDO1018[] = {
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1018_00, 4, &id_count},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_01, 0x1337, &vendor_id},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_02, 1234, &product_code},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_03, 0, &revision_num},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_04, 0, &serial_num},
};

// =====================================================================
// 0x6000: TxPDO 실제 메모리 연결 (bData는 0으로 둬도 무방)
// =====================================================================
const _objd SDO6000[] = {
  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName6000_00, 6, &txpdo_actual_count},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName6000_01, 0, &Wb.actual_pos[0]},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName6000_02, 0, &Wb.actual_pos[1]},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName6000_03, 0, &Wb.actual_pos[2]},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName6000_04, 0, &Wb.actual_pos[3]},
  {0x05, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6000_05, 0, &Wb.status_word},
  {0x06, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6000_06, 0, &Wb.error_code},
};

// =====================================================================
// 0x7000: RxPDO 실제 메모리 연결
// =====================================================================
const _objd SDO7000[] = {
  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_00, 18, &rxpdo_target_count},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName7000_01, 0, &Rb.target_pos[0]},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName7000_02, 0, &Rb.target_pos[1]},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName7000_03, 0, &Rb.target_pos[2]},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName7000_04, 0, &Rb.target_pos[3]},
  {0x05, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_05, 0, &Rb.direction[0]},
  {0x06, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_06, 0, &Rb.direction[1]},
  {0x07, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_07, 0, &Rb.direction[2]},
  {0x08, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_08, 0, &Rb.direction[3]},
  {0x09, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_09, 0, &Rb.target_speed[0]},
  {0x0A, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_0A, 0, &Rb.target_speed[1]},
  {0x0B, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_0B, 0, &Rb.target_speed[2]},
  {0x0C, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_0C, 0, &Rb.target_speed[3]},
  {0x0D, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_0D, 0, &Rb.accel_pattern[0]},
  {0x0E, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_0E, 0, &Rb.accel_pattern[1]},
  {0x0F, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_0F, 0, &Rb.accel_pattern[2]},
  {0x10, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_10, 0, &Rb.accel_pattern[3]},
  {0x11, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName7000_11, 0, &Rb.control_word},
  {0x12, DTYPE_UNSIGNED16, 16, ATYPE_RW, "Dummy", 0, &Rb.dummy},
};

// =====================================================================
// [핵심 해결] TxPDO 매핑 설정 (0x1A00) -> 6번째, 7번째 더블 바인딩!
// =====================================================================
const _objd SDO1A00[] = {
  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName1A00_00, 6, &txpdo_map_count},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x60000120, &txpdo_mapping_entries[0]},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x60000220, &txpdo_mapping_entries[1]},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x60000320, &txpdo_mapping_entries[2]},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x60000420, &txpdo_mapping_entries[3]},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x60000510, &txpdo_mapping_entries[4]},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x60000610, &txpdo_mapping_entries[5]},
};

// =====================================================================
// [핵심 해결] RxPDO 매핑 설정 (0x1600) -> 6번째, 7번째 더블 바인딩!
// =====================================================================
const _objd SDO1600[] = {
  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName1600_00, 18, &rxpdo_map_count},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000120, &rxpdo_mapping_entries[0]},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000220, &rxpdo_mapping_entries[1]},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000320, &rxpdo_mapping_entries[2]},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000420, &rxpdo_mapping_entries[3]},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000508, &rxpdo_mapping_entries[4]},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000608, &rxpdo_mapping_entries[5]},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000708, &rxpdo_mapping_entries[6]},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000808, &rxpdo_mapping_entries[7]},
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000908, &rxpdo_mapping_entries[8]},
  {0x0A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000A08, &rxpdo_mapping_entries[9]},
  {0x0B, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000B08, &rxpdo_mapping_entries[10]},
  {0x0C, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000C08, &rxpdo_mapping_entries[11]},
  {0x0D, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000D08, &rxpdo_mapping_entries[12]},
  {0x0E, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000E08, &rxpdo_mapping_entries[13]},
  {0x0F, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000F08, &rxpdo_mapping_entries[14]},
  {0x10, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70001008, &rxpdo_mapping_entries[15]},
  {0x11, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70001110, &rxpdo_mapping_entries[16]},
  {0x12, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70001210, &rxpdo_mapping_entries[17]},
};

// =====================================================================
// SyncManager 파라미터 셋업 (0x1C00, 1C12, 1C13)
// =====================================================================
const _objd SDO1C00[] = {
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_00, 4, &sm_comm_type_count},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_01, 1, &sm_comm_types[0]},
  {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_02, 2, &sm_comm_types[1]},
  {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_03, 3, &sm_comm_types[2]},
  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_04, 4, &sm_comm_types[3]},
};

const _objd SDO1C12[] = {
  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName1C12_00, 1, &sm2_assign_count},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName1C12_01, 0x1600, &sm2_pdo_assign},
};
const _objd SDO1C13[] = {
  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName1C13_00, 1, &sm3_assign_count},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName1C13_01, 0x1A00, &sm3_pdo_assign},
};

// -------------------------------------------------------------
// 3. 최종 Object List (SOES 엔진이 읽어가는 메인 배열)
// -------------------------------------------------------------
const _objectlist SDOobjects[] = {
  {0x1000, OTYPE_VAR,    0, 1, acName1000, SDO1000},
  {0x1008, OTYPE_VAR,    0, 1, acName1008, SDO1008},
  {0x1009, OTYPE_VAR,    0, 1, acName1009, SDO1009},
  {0x100A, OTYPE_VAR,    0, 1, acName100A, SDO100A},
  {0x1018, OTYPE_RECORD, 4, 5, acName1018, SDO1018},
  {0x1600, OTYPE_RECORD, 18, 19, acName1600, SDO1600},
  {0x1A00, OTYPE_RECORD, 6, 7, acName1A00, SDO1A00},
  {0x1C00, OTYPE_ARRAY,  4, 5, acName1C00, SDO1C00},
  {0x1C12, OTYPE_ARRAY,  1, 2, acName1C12, SDO1C12},
  {0x1C13, OTYPE_ARRAY,  1, 2, acName1C13, SDO1C13},
  {0x6000, OTYPE_RECORD, 6, 7, acName6000, SDO6000},
  {0x7000, OTYPE_RECORD, 18, 19, acName7000, SDO7000},
  {0xffff, 0xff, 0xff, 0xff, NULL, NULL}
};

//#include "esc_coe.h"
//#include "utypes.h"
//#include <stddef.h>
//
//_Rbuffer Rb;
//_Wbuffer Wb;
//
//#ifndef HW_REV
//#define HW_REV "1.0"
//#endif
//
//#ifndef SW_REV
//#define SW_REV "1.0"
//#endif
//
//// 1. 파일 상단에 SubIndex 0을 위한 전역 변수 선언
//uint8_t rxpdo_map_count = 18;  // 0x1600의 SubIndex 갯수
//uint8_t txpdo_map_count = 6;   // 0x1A00의 SubIndex 갯수
//uint8_t sm2_assign_count = 1;  // 0x1C12에 할당된 PDO 갯수
//uint8_t sm3_assign_count = 1;  // 0x1C13에 할당된 PDO 갯수
//
//// 0x6000, 0x7000용 갯수 변수도 추가해 주는 것이 안전합니다.
//uint8_t rxpdo_target_count = 18;
//uint8_t txpdo_actual_count = 6;
//
//// -------------------------------------------------------------
//// 1. Object Dictionary 이름(String) 정의
//// -------------------------------------------------------------
//static const char acName1000[] = "Device Type";
//static const char acName1008[] = "Device Name";
//static const char acName1009[] = "Hardware Version";
//static const char acName100A[] = "Software Version";
//
//static const char acName1018[] = "Identity Object";
//static const char acName1018_00[] = "Max SubIndex";
//static const char acName1018_01[] = "Vendor ID";
//static const char acName1018_02[] = "Product Code";
//static const char acName1018_03[] = "Revision Number";
//static const char acName1018_04[] = "Serial Number";
//
//// PDO 매핑 항목들의 공통 이름
//static const char acNameMapping[] = "PDO Mapping Entry";
//
//static const char acName1600[] = "RxPDO Mapping";
//static const char acName1600_00[] = "Max SubIndex";
//static const char acName1A00[] = "TxPDO Mapping";
//static const char acName1A00_00[] = "Max SubIndex";
//
//static const char acName1C00[] = "Sync Manager Communication Type";
//static const char acName1C00_00[] = "Max SubIndex";
//static const char acName1C00_01[] = "Communication Type SM0";
//static const char acName1C00_02[] = "Communication Type SM1";
//static const char acName1C00_03[] = "Communication Type SM2";
//static const char acName1C00_04[] = "Communication Type SM3";
//
//static const char acName1C12[] = "Sync Manager 2 PDO Assignment";
//static const char acName1C12_00[] = "Max SubIndex";
//static const char acName1C12_01[] = "PDO Mapping";
//
//static const char acName1C13[] = "Sync Manager 3 PDO Assignment";
//static const char acName1C13_00[] = "Max SubIndex";
//static const char acName1C13_01[] = "PDO Mapping";
//
//// TxPDO (STM32 -> 마스터) 변수 이름
//static const char acName6000[] = "TxPDO 4-Axis Actual Data";
//static const char acName6000_00[] = "Max SubIndex";
//static const char acName6000_01[] = "Actual Pos Axis 1";
//static const char acName6000_02[] = "Actual Pos Axis 2";
//static const char acName6000_03[] = "Actual Pos Axis 3";
//static const char acName6000_04[] = "Actual Pos Axis 4";
//static const char acName6000_05[] = "Status Word";
//static const char acName6000_06[] = "Error Code";
//
//// RxPDO (마스터 -> STM32) 변수 이름
//static const char acName7000[] = "RxPDO 4-Axis Target Data";
//static const char acName7000_00[] = "Max SubIndex";
//static const char acName7000_01[] = "Target Pos Axis 1";
//static const char acName7000_02[] = "Target Pos Axis 2";
//static const char acName7000_03[] = "Target Pos Axis 3";
//static const char acName7000_04[] = "Target Pos Axis 4";
//static const char acName7000_05[] = "Direction Axis 1";
//static const char acName7000_06[] = "Direction Axis 2";
//static const char acName7000_07[] = "Direction Axis 3";
//static const char acName7000_08[] = "Direction Axis 4";
//static const char acName7000_09[] = "Target Speed Axis 1";
//static const char acName7000_0A[] = "Target Speed Axis 2";
//static const char acName7000_0B[] = "Target Speed Axis 3";
//static const char acName7000_0C[] = "Target Speed Axis 4";
//static const char acName7000_0D[] = "Accel Pattern Axis 1";
//static const char acName7000_0E[] = "Accel Pattern Axis 2";
//static const char acName7000_0F[] = "Accel Pattern Axis 3";
//static const char acName7000_10[] = "Accel Pattern Axis 4";
//static const char acName7000_11[] = "Control Word";
//
//
//// -------------------------------------------------------------
//// 2. SDO 데이터 구조체 맵핑
//// -------------------------------------------------------------
//const _objd SDO1000[] = {
//  {0x0, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1000, 0x01901389, NULL},
//};
//const _objd SDO1008[] = {
//  // 디바이스 이름을 H753 4축 제어기에 맞게 변경했습니다.
//  {0x0, DTYPE_VISIBLE_STRING, 216, ATYPE_RO, acName1008, 0, "STM32H753 4-Axis Controller"},
//};
//const _objd SDO1009[] = {
//  {0x0, DTYPE_VISIBLE_STRING, 0, ATYPE_RO, acName1009, 0, HW_REV},
//};
//const _objd SDO100A[] = {
//  {0x0, DTYPE_VISIBLE_STRING, 0, ATYPE_RO, acName100A, 0, SW_REV},
//};
//const _objd SDO1018[] = {
//  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1018_00, 4, NULL},
//  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_01, 0x1337, NULL},
//  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_02, 1234, NULL},
//  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_03, 0, NULL},
//  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_04, 0x00000000, NULL},
//};
//
//// =====================================================================
//// [핵심] 0x6000: TxPDO (슬레이브 -> 마스터) 실제 메모리 연결
//// =====================================================================
//const _objd SDO6000[] = {
//  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName6000_00, 6, &txpdo_actual_count}, // 총 6개 항목
//  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName6000_01, 0, &Wb.actual_pos[0]},
//  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName6000_02, 0, &Wb.actual_pos[1]},
//  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName6000_03, 0, &Wb.actual_pos[2]},
//  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName6000_04, 0, &Wb.actual_pos[3]},
//  {0x05, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6000_05, 0, &Wb.status_word},
//  {0x06, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6000_06, 0, &Wb.error_code},
//};
//
//// =====================================================================
//// [핵심] 0x7000: RxPDO (마스터 -> 슬레이브) 실제 메모리 연결
//// =====================================================================
//const _objd SDO7000[] = {
//  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_00, 18, &rxpdo_target_count}, // 총 18개 항목 (0x12)
//  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName7000_01, 0, &Rb.target_pos[0]},
//  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName7000_02, 0, &Rb.target_pos[1]},
//  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName7000_03, 0, &Rb.target_pos[2]},
//  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName7000_04, 0, &Rb.target_pos[3]},
//  {0x05, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_05, 0, &Rb.direction[0]},
//  {0x06, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_06, 0, &Rb.direction[1]},
//  {0x07, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_07, 0, &Rb.direction[2]},
//  {0x08, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_08, 0, &Rb.direction[3]},
//  {0x09, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_09, 0, &Rb.target_speed[0]},
//  {0x0A, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_0A, 0, &Rb.target_speed[1]},
//  {0x0B, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_0B, 0, &Rb.target_speed[2]},
//  {0x0C, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_0C, 0, &Rb.target_speed[3]},
//  {0x0D, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_0D, 0, &Rb.accel_pattern[0]},
//  {0x0E, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_0E, 0, &Rb.accel_pattern[1]},
//  {0x0F, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_0F, 0, &Rb.accel_pattern[2]},
//  {0x10, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName7000_10, 0, &Rb.accel_pattern[3]},
//  {0x11, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName7000_11, 0, &Rb.control_word},
//  {0x12, DTYPE_UNSIGNED16, 16, ATYPE_RW, "Dummy", 0, &Rb.dummy},
//};
//
//// =====================================================================
//// TxPDO 매핑 설정 (0x1A00) -> 0x6000의 주소와 비트 길이를 등록
//// =====================================================================
//const _objd SDO1A00[] = {
//  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName1A00_00, 6, &txpdo_map_count},
//  // 형식: 0x[Index][SubIndex][BitLength(Hex)]
//  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x60000120, NULL}, // Actual Pos 1 (32bit=0x20)
//  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x60000220, NULL}, // Actual Pos 2
//  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x60000320, NULL}, // Actual Pos 3
//  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x60000420, NULL}, // Actual Pos 4
//  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x60000510, NULL}, // Status Word (16bit=0x10)
//  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x60000610, NULL}, // Error Code (16bit=0x10)
//};
//
//// =====================================================================
//// RxPDO 매핑 설정 (0x1600) -> 0x7000의 주소와 비트 길이를 등록
//// =====================================================================
//const _objd SDO1600[] = {
//  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName1600_00, 18, &rxpdo_map_count},
//  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000120, NULL},
//  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000220, NULL},
//  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000320, NULL},
//  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000420, NULL},
//  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000508, NULL}, // 8bit = 0x08
//  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000608, NULL},
//  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000708, NULL},
//  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000808, NULL},
//  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000908, NULL},
//  {0x0A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000A08, NULL},
//  {0x0B, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000B08, NULL},
//  {0x0C, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000C08, NULL},
//  {0x0D, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000D08, NULL},
//  {0x0E, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000E08, NULL},
//  {0x0F, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70000F08, NULL},
//  {0x10, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70001008, NULL},
//  {0x11, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70001110, NULL}, // 16bit = 0x10
//  {0x12, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x70001210, NULL},
//};
//
//// SyncManager 파라미터 셋업
//const _objd SDO1C00[] = {
//  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_00, 4, NULL},
//  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_01, 1, NULL}, // SM0: Mailbox Out (1)
//  {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_02, 2, NULL}, // SM1: Mailbox In (2)
//  {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_03, 3, NULL}, // SM2: Outputs (3)
//  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_04, 4, NULL}, // SM3: Inputs (4)
//};
//
//uint16_t sm2_pdo_assign = 0x1600;
//uint16_t sm3_pdo_assign = 0x1A00;
//
//const _objd SDO1C12[] = {
//  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName1C12_00, 1, &sm2_assign_count},
//  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName1C12_01, 0x1600, &sm2_pdo_assign}, // RxPDO 매핑 인덱스 등록
//};
//const _objd SDO1C13[] = {
//  {0x00, DTYPE_UNSIGNED8,  8, ATYPE_RW, acName1C13_00, 1, &sm3_assign_count},
//  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName1C13_01, 0x1A00, &sm3_pdo_assign}, // TxPDO 매핑 인덱스 등록
//};
//
//// -------------------------------------------------------------
//// 3. 최종 Object List (SOES 엔진이 읽어가는 메인 배열)
//// -------------------------------------------------------------
//const _objectlist SDOobjects[] = {
//  {0x1000, OTYPE_VAR,    0, 1, acName1000, SDO1000}, // n_sub 대신 n_obj 사용 확인
//  {0x1008, OTYPE_VAR,    0, 1, acName1008, SDO1008},
//  {0x1009, OTYPE_VAR,    0, 1, acName1009, SDO1009},
//  {0x100A, OTYPE_VAR,    0, 1, acName100A, SDO100A},
//  {0x1018, OTYPE_RECORD, 4, 5, acName1018, SDO1018}, // n_obj를 5로 (Sub 0~4)
//  {0x1600, OTYPE_RECORD, 18, 19, acName1600, SDO1600}, // Sub 0~18 이므로 n_obj=19
//  {0x1A00, OTYPE_RECORD, 6, 7, acName1A00, SDO1A00},  // Sub 0~6 이므로 n_obj=7
//  {0x1C00, OTYPE_ARRAY,  4, 5, acName1C00, SDO1C00},
//  {0x1C12, OTYPE_ARRAY,  1, 2, acName1C12, SDO1C12},
//  {0x1C13, OTYPE_ARRAY,  1, 2, acName1C13, SDO1C13},
//  {0x6000, OTYPE_RECORD, 6, 7, acName6000, SDO6000},
//  {0x7000, OTYPE_RECORD, 18, 19, acName7000, SDO7000},
//  {0xffff, 0xff, 0xff, 0xff, NULL, NULL}
//};
