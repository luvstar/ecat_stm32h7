#include "esc.h"             // SOES 코어 헤더 (ESC_read, ESC_write 선언 포함)
#include "stm32h7xx_hal.h"   // STM32H7 시리즈 HAL 라이브러리
#include "string.h"

// 큐브IDE에서 설정한 SPI 핸들
extern SPI_HandleTypeDef hspi1;

#define LAN9252_CS_PORT GPIOE
#define LAN9252_CS_PIN  GPIO_PIN_2
#define CS_LOW()  HAL_GPIO_WritePin(LAN9252_CS_PORT, LAN9252_CS_PIN, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(LAN9252_CS_PORT, LAN9252_CS_PIN, GPIO_PIN_SET)

// =========================================================================
// 2. 증명된 원시 통신 함수 (칩 ID 읽기를 성공시킨 코드 완벽 적용)
// =========================================================================
void SPI_Write(uint16_t address, void *buf, uint16_t len) {
    uint8_t header[3];
    header[0] = 0x02;
    header[1] = (address >> 8) & 0xFF;
    header[2] = address & 0xFF;

    __NOP();
    __NOP();

    CS_LOW();
    HAL_SPI_Transmit(&hspi1, header, 3, HAL_MAX_DELAY);

    // buf를 8비트 포인터로 잠시 변환해서 전송
    HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, len, HAL_MAX_DELAY);

    CS_HIGH();
}

void SPI_Read(uint16_t address, void *buf, uint16_t len) {
    uint8_t header[4];
    header[0] = 0x0B;
    header[1] = (address >> 8) & 0xFF;
    header[2] = address & 0xFF;
    header[3] = 0x00;

    __NOP();
    __NOP();

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, header, 4, HAL_MAX_DELAY);

    // buf를 8비트 포인터로 잠시 변환해서 수신
    HAL_SPI_Receive(&hspi1, (uint8_t *)buf, len, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
}

// =========================================================================
// 3. LAN9252 전용 하드웨어 초기화 (증명된 SPI_Read/Write 사용)
// =========================================================================
uint8_t LAN9252_HW_Init(void) {
    uint32_t data = 0;
    uint32_t intMask = 0;

    // 0x0064 읽기: 성공했던 통신 코드를 타기 때문에 여기서 0x87654321이 무조건 찍힙니다.
    do {
        SPI_Read(0x0064, &data, 4);
    } while (data != 0x87654321);

    // 인터럽트 끄기
    data = 0x00000000;
    SPI_Write(0x005C, &data, 4);

    // AL Event Mask 초기화
    do {
        data = 0x00000093;
        SPI_Write(0x0204, &data, 4);
        SPI_Read(0x0204, &intMask, 4);
    } while (intMask != 0x00000093);

    data = 0x00000000;
    SPI_Write(0x0204, &data, 4);

    // IRQ 설정
    data = 0x00000101;
    SPI_Write(0x0054, &data, 4);

    // 글로벌 인터럽트 켜기
    data = 0x00000001;
    SPI_Write(0x005C, &data, 4);

    // 상태 지우기
    SPI_Read(0x0058, &data, 4);

    return 1;
}

// =========================================================================
// 4. SOES 이더캣 스택용 함수 (Microchip 간접 접근 방식 + 증명된 통신 코드)
// =========================================================================
void ESC_read(uint16_t address, void *buf, uint16_t len) {
    if (address >= 0x1000) {
        // [Process RAM 공간] 빠르고 안정적인 다이렉트 통신
        SPI_Read(address, buf, len);
    } else {
        // [제어 레지스터 공간] LAN9252의 32-bit 제한을 피하기 위한 우회(Indirect)
        uint8_t *pData = (uint8_t *)buf;
        uint16_t bytes_left = len;
        uint16_t current_addr = address;

        while (bytes_left > 0) {
            uint8_t validDataLen = (bytes_left > 4) ? 4 : bytes_left;
            if (current_addr & 1) validDataLen = 1;
            else if (current_addr & 2) validDataLen = (validDataLen >= 2) ? 2 : 1;
            else if (validDataLen < 4) validDataLen = (validDataLen >= 2) ? 2 : 1;

            uint32_t cmd = current_addr | ((uint32_t)validDataLen << 16) | 0xC0000000;
            SPI_Write(0x0304, &cmd, 4);

            uint32_t data = 0;
            SPI_Read(0x0300, &data, 4);

            for (int j = 0; j < validDataLen; j++) {
                *pData++ = (data >> (j * 8)) & 0xFF;
            }
            current_addr += validDataLen;
            bytes_left -= validDataLen;
        }
    }
}

void ESC_write(uint16_t address, void *buf, uint16_t len) {
    if (address >= 0x1000) {
        // [Process RAM 공간] 빠르고 안정적인 다이렉트 통신
        SPI_Write(address, buf, len);
    } else {
        // [제어 레지스터 공간] 우회(Indirect) 접근
        uint8_t *pData = (uint8_t *)buf;
        uint16_t bytes_left = len;
        uint16_t current_addr = address;

        while (bytes_left > 0) {
            uint8_t validDataLen = (bytes_left > 4) ? 4 : bytes_left;
            if (current_addr & 1) validDataLen = 1;
            else if (current_addr & 2) validDataLen = (validDataLen >= 2) ? 2 : 1;
            else if (validDataLen < 4) validDataLen = (validDataLen >= 2) ? 2 : 1;

            uint32_t data = 0;
            for (int j = 0; j < validDataLen; j++) {
                data |= ((uint32_t)(*pData++) << (j * 8));
            }

            SPI_Write(0x0300, &data, 4);

            uint32_t cmd = current_addr | ((uint32_t)validDataLen << 16) | 0x80000000;
            SPI_Write(0x0304, &cmd, 4);

            current_addr += validDataLen;
            bytes_left -= validDataLen;
        }
    }
}
void ESC_init(const esc_cfg_t * cfg) {
    // 하드웨어 초기화 딜레이 (필요시 사용)
    HAL_Delay(10);
}
