#include "esc_coe.h"
#include "utypes.h"
#include <stddef.h>

_Rbuffer Rb;
_Wbuffer Wb;
_Cbuffer Cb;

#ifndef HW_REV
#define HW_REV "1.0"
#endif
#ifndef SW_REV
#define SW_REV "1.0"
#endif

uint8_t rxpdo_map_count[4] = {8, 8, 8, 8};
uint8_t txpdo_map_count[4] = {3, 3, 3, 3};

uint8_t sm2_assign_count = 4;
uint16_t sm2_pdo_assign[4] = {0x1600, 0x1601, 0x1602, 0x1603};

uint8_t sm3_assign_count = 4;
uint16_t sm3_pdo_assign[4] = {0x1A00, 0x1A01, 0x1A02, 0x1A03};

uint8_t rxpdo_target_count = 32;
uint8_t txpdo_actual_count = 12;
uint8_t config_param_count = 4;

uint8_t sm_comm_type_count = 4;
uint8_t sm_comm_types[4] = {1, 2, 3, 4};

uint8_t id_count = 4;
uint32_t vendor_id = 0x1337;
uint32_t product_code = 0x04D3;
uint32_t revision_num = 1;
uint32_t serial_num = 0;
uint32_t device_type = 0x00000000; // Generic Device

// ⭐️ [수정 완료] 배열 매핑 순서를 Pos -> Spd -> MaxSpd -> Acc -> Dec -> CW -> Pat -> Dum 순서로 변경
uint32_t rxpdo_map_1[8] = {0x21000520, 0x21000920, 0x21000D20, 0x21001120, 0x21001520, 0x21000110, 0x21001908, 0x21001D08};
uint32_t rxpdo_map_2[8] = {0x21000620, 0x21000A20, 0x21000E20, 0x21001220, 0x21001620, 0x21000210, 0x21001A08, 0x21001E08};
uint32_t rxpdo_map_3[8] = {0x21000720, 0x21000B20, 0x21000F20, 0x21001320, 0x21001720, 0x21000310, 0x21001B08, 0x21001F08};
uint32_t rxpdo_map_4[8] = {0x21000820, 0x21000C20, 0x21001020, 0x21001420, 0x21001820, 0x21000410, 0x21001C08, 0x21002008};

uint32_t txpdo_map_1[3] = {0x20000110, 0x20000520, 0x20000910};
uint32_t txpdo_map_2[3] = {0x20000210, 0x20000620, 0x20000A10};
uint32_t txpdo_map_3[3] = {0x20000310, 0x20000720, 0x20000B10};
uint32_t txpdo_map_4[3] = {0x20000410, 0x20000820, 0x20000C10};

static const char acNameMapping[] = "PDO Mapping Entry";
static const char acName1018[] = "Identity Object";
static const char acName1C00[] = "Sync Manager Comm Type";
static const char acName1C12[] = "Sync Manager 2 PDO Assign";
static const char acName1C13[] = "Sync Manager 3 PDO Assign";

const _objd SDO1000[] = {
  {0x00, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Device Type", 0, &device_type}
};

const _objd SDO1018[] = {
  {0x00, DTYPE_UNSIGNED8,   8, ATYPE_RO, "Max SubIndex", 4, &id_count},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Vendor ID", 0, &vendor_id},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Product Code", 0, &product_code},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Revision", 0, &revision_num},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, "Serial", 0, &serial_num},
};

const _objd SDO2000[] = {
  {0x00, DTYPE_UNSIGNED8,   8, ATYPE_RO, "Max SubIndex", 12, &txpdo_actual_count},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, "SW1", 0, &Wb.axis[0].status_word},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, "SW2", 0, &Wb.axis[1].status_word},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RO, "SW3", 0, &Wb.axis[2].status_word},
  {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RO, "SW4", 0, &Wb.axis[3].status_word},
  {0x05, DTYPE_INTEGER32,  32, ATYPE_RO, "ActPos1", 0, &Wb.axis[0].actual_pos},
  {0x06, DTYPE_INTEGER32,  32, ATYPE_RO, "ActPos2", 0, &Wb.axis[1].actual_pos},
  {0x07, DTYPE_INTEGER32,  32, ATYPE_RO, "ActPos3", 0, &Wb.axis[2].actual_pos},
  {0x08, DTYPE_INTEGER32,  32, ATYPE_RO, "ActPos4", 0, &Wb.axis[3].actual_pos},
  {0x09, DTYPE_UNSIGNED16, 16, ATYPE_RO, "Err1", 0, &Wb.axis[0].error_code},
  {0x0A, DTYPE_UNSIGNED16, 16, ATYPE_RO, "Err2", 0, &Wb.axis[1].error_code},
  {0x0B, DTYPE_UNSIGNED16, 16, ATYPE_RO, "Err3", 0, &Wb.axis[2].error_code},
  {0x0C, DTYPE_UNSIGNED16, 16, ATYPE_RO, "Err4", 0, &Wb.axis[3].error_code},
};

const _objd SDO2100[] = {
  {0x00, DTYPE_UNSIGNED8,   8, ATYPE_RO, "Max SubIndex", 32, &rxpdo_target_count},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RW, "CW1", 0, &Rb.axis[0].control_word},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RW, "CW2", 0, &Rb.axis[1].control_word},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, "CW3", 0, &Rb.axis[2].control_word},
  {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RW, "CW4", 0, &Rb.axis[3].control_word},
  {0x05, DTYPE_INTEGER32,  32, ATYPE_RW, "Pos1", 0, &Rb.axis[0].target_pos},
  {0x06, DTYPE_INTEGER32,  32, ATYPE_RW, "Pos2", 0, &Rb.axis[1].target_pos},
  {0x07, DTYPE_INTEGER32,  32, ATYPE_RW, "Pos3", 0, &Rb.axis[2].target_pos},
  {0x08, DTYPE_INTEGER32,  32, ATYPE_RW, "Pos4", 0, &Rb.axis[3].target_pos},
  {0x09, DTYPE_INTEGER32,  32, ATYPE_RW, "Spd1", 0, &Rb.axis[0].target_speed},
  {0x0A, DTYPE_INTEGER32,  32, ATYPE_RW, "Spd2", 0, &Rb.axis[1].target_speed},
  {0x0B, DTYPE_INTEGER32,  32, ATYPE_RW, "Spd3", 0, &Rb.axis[2].target_speed},
  {0x0C, DTYPE_INTEGER32,  32, ATYPE_RW, "Spd4", 0, &Rb.axis[3].target_speed},
  {0x0D, DTYPE_UNSIGNED32, 32, ATYPE_RW, "MaxSpd1", 0, &Rb.axis[0].max_speed},
  {0x0E, DTYPE_UNSIGNED32, 32, ATYPE_RW, "MaxSpd2", 0, &Rb.axis[1].max_speed},
  {0x0F, DTYPE_UNSIGNED32, 32, ATYPE_RW, "MaxSpd3", 0, &Rb.axis[2].max_speed},
  {0x10, DTYPE_UNSIGNED32, 32, ATYPE_RW, "MaxSpd4", 0, &Rb.axis[3].max_speed},
  {0x11, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Acc1", 0, &Rb.axis[0].accel_time},
  {0x12, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Acc2", 0, &Rb.axis[1].accel_time},
  {0x13, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Acc3", 0, &Rb.axis[2].accel_time},
  {0x14, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Acc4", 0, &Rb.axis[3].accel_time},
  {0x15, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Dec1", 0, &Rb.axis[0].decel_time},
  {0x16, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Dec2", 0, &Rb.axis[1].decel_time},
  {0x17, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Dec3", 0, &Rb.axis[2].decel_time},
  {0x18, DTYPE_UNSIGNED32, 32, ATYPE_RW, "Dec4", 0, &Rb.axis[3].decel_time},
  {0x19, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Pat1", 0, &Rb.axis[0].speed_pattern},
  {0x1A, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Pat2", 0, &Rb.axis[1].speed_pattern},
  {0x1B, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Pat3", 0, &Rb.axis[2].speed_pattern},
  {0x1C, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Pat4", 0, &Rb.axis[3].speed_pattern},
  {0x1D, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Dum1", 0, &Rb.axis[0].dummy},
  {0x1E, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Dum2", 0, &Rb.axis[1].dummy},
  {0x1F, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Dum3", 0, &Rb.axis[2].dummy},
  {0x20, DTYPE_UNSIGNED8,   8, ATYPE_RW, "Dum4", 0, &Rb.axis[3].dummy},
};

// --- RxPDO Mapping Objects (0x1600 - 0x1603) ---
// ⭐️ [수정 완료] SDO 딕셔너리의 기본 매핑값(6번째 인자)도 새로운 순서에 맞게 업데이트!
const _objd SDO1600[] = {
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RW, "Count", 8, &rxpdo_map_count[0]},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21000520, &rxpdo_map_1[0]},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21000920, &rxpdo_map_1[1]},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21000D20, &rxpdo_map_1[2]},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21001120, &rxpdo_map_1[3]},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21001520, &rxpdo_map_1[4]},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21000110, &rxpdo_map_1[5]},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21001908, &rxpdo_map_1[6]},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21001D08, &rxpdo_map_1[7]}
};

const _objd SDO1601[] = {
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RW, "Count", 8, &rxpdo_map_count[1]},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21000620, &rxpdo_map_2[0]},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21000A20, &rxpdo_map_2[1]},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21000E20, &rxpdo_map_2[2]},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21001220, &rxpdo_map_2[3]},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21001620, &rxpdo_map_2[4]},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21000210, &rxpdo_map_2[5]},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21001A08, &rxpdo_map_2[6]},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21001E08, &rxpdo_map_2[7]}
};

const _objd SDO1602[] = {
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RW, "Count", 8, &rxpdo_map_count[2]},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21000720, &rxpdo_map_3[0]},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21000B20, &rxpdo_map_3[1]},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21000F20, &rxpdo_map_3[2]},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21001320, &rxpdo_map_3[3]},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21001720, &rxpdo_map_3[4]},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21000310, &rxpdo_map_3[5]},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21001B08, &rxpdo_map_3[6]},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21001F08, &rxpdo_map_3[7]}
};

const _objd SDO1603[] = {
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RW, "Count", 8, &rxpdo_map_count[3]},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21000820, &rxpdo_map_4[0]},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21000C20, &rxpdo_map_4[1]},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21001020, &rxpdo_map_4[2]},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21001420, &rxpdo_map_4[3]},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21001820, &rxpdo_map_4[4]},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21000410, &rxpdo_map_4[5]},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21001C08, &rxpdo_map_4[6]},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x21002008, &rxpdo_map_4[7]}
};

// --- TxPDO Mapping Objects (0x1A00 - 0x1A03) ---
const _objd SDO1A00[] = {
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RW, "Count", 3, &txpdo_map_count[0]},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x20000110, &txpdo_map_1[0]},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x20000520, &txpdo_map_1[1]},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x20000910, &txpdo_map_1[2]}
};

const _objd SDO1A01[] = {
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RW, "Count", 3, &txpdo_map_count[1]},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x20000210, &txpdo_map_2[0]},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x20000620, &txpdo_map_2[1]},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x20000A10, &txpdo_map_2[2]}
};

const _objd SDO1A02[] = {
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RW, "Count", 3, &txpdo_map_count[2]},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x20000310, &txpdo_map_3[0]},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x20000720, &txpdo_map_3[1]},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x20000B10, &txpdo_map_3[2]}
};

const _objd SDO1A03[] = {
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RW, "Count", 3, &txpdo_map_count[3]},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x20000410, &txpdo_map_4[0]},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x20000820, &txpdo_map_4[1]},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acNameMapping, 0x20000C10, &txpdo_map_4[2]}
};

const _objd SDO1C00[] = {
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, "Count", 4, &sm_comm_type_count},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RO, "Type0", 1, &sm_comm_types[0]}, // Mailbox Out
  {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RO, "Type1", 2, &sm_comm_types[1]}, // Mailbox In
  {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RO, "Type2", 3, &sm_comm_types[2]}, // Process Data Out (Outputs)
  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RO, "Type3", 4, &sm_comm_types[3]}  // Process Data In (Inputs)
};

const _objd SDO1C12[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RW, "Count", 4, &sm2_assign_count},
    {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RW, "P1", 0x1600, &sm2_pdo_assign[0]},
    {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RW, "P2", 0x1601, &sm2_pdo_assign[1]},
    {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, "P3", 0x1602, &sm2_pdo_assign[2]},
    {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RW, "P4", 0x1603, &sm2_pdo_assign[3]}
};

const _objd SDO1C13[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RW, "Count", 4, &sm3_assign_count},
    {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RW, "P1", 0x1A00, &sm3_pdo_assign[0]},
    {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RW, "P2", 0x1A01, &sm3_pdo_assign[1]},
    {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, "P3", 0x1A02, &sm3_pdo_assign[2]},
    {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RW, "P4", 0x1A03, &sm3_pdo_assign[3]}
};

const _objectlist SDOobjects[] = {
  {0x1000, OTYPE_VAR,     1,  1, "Device Type", SDO1000},
  {0x1018, OTYPE_RECORD,  4,  5, acName1018, SDO1018},
  {0x1600, OTYPE_RECORD,  8,  9, "Rx1", SDO1600},
  {0x1601, OTYPE_RECORD,  8,  9, "Rx2", SDO1601},
  {0x1602, OTYPE_RECORD,  8,  9, "Rx3", SDO1602},
  {0x1603, OTYPE_RECORD,  8,  9, "Rx4", SDO1603},
  {0x1A00, OTYPE_RECORD,  3,  4, "Tx1", SDO1A00},
  {0x1A01, OTYPE_RECORD,  3,  4, "Tx2", SDO1A01},
  {0x1A02, OTYPE_RECORD,  3,  4, "Tx3", SDO1A02},
  {0x1A03, OTYPE_RECORD,  3,  4, "Tx4", SDO1A03},
  {0x1C00, OTYPE_ARRAY,   4,  5, acName1C00, SDO1C00},
  {0x1C12, OTYPE_ARRAY,   4,  5, acName1C12, SDO1C12},
  {0x1C13, OTYPE_ARRAY,   4,  5, acName1C13, SDO1C13},
  {0x2000, OTYPE_RECORD, 12, 13, "Tx Data", SDO2000},
  {0x2100, OTYPE_RECORD, 32, 33, "Rx Data", SDO2100},
  {0xffff, 0xff, 0xff, 0xff, NULL, NULL}
};
