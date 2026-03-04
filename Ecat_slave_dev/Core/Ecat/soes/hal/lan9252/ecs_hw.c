#include "esc.h"             // SOES 코어 헤더 (ESC_read, ESC_write 선언 포함)
#include "stm32h7xx_hal.h"   // STM32H7 시리즈 HAL 라이브러리
#include "string.h"

// 큐브IDE에서 설정한 SPI 핸들
extern SPI_HandleTypeDef hspi1;

#define LAN9252_CS_PORT GPIOE
#define LAN9252_CS_PIN  GPIO_PIN_2
#define CS_LOW()  HAL_GPIO_WritePin(LAN9252_CS_PORT, LAN9252_CS_PIN, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(LAN9252_CS_PORT, LAN9252_CS_PIN, GPIO_PIN_SET)

// LAN9252 명령어 매크로
#define ESC_CMD_SERIAL_WRITE     0x02
#define ESC_CMD_SERIAL_READ      0x03
#define ESC_CMD_FAST_READ        0x0B
#define ESC_CMD_FAST_READ_DUMMY  1

// 주요 레지스터 주소
#define ESC_PRAM_RD_FIFO_REG     0x000
#define ESC_PRAM_WR_FIFO_REG     0x020
#define ESC_PRAM_RD_ADDR_LEN_REG 0x308
#define ESC_PRAM_RD_CMD_REG      0x30C
#define ESC_PRAM_WR_ADDR_LEN_REG 0x310
#define ESC_PRAM_WR_CMD_REG      0x314

#define ESC_CSR_DATA_REG         0x300
#define ESC_CSR_CMD_REG          0x304
#define ESC_RESET_CTRL_REG       0x1F8

// 비트 연산 매크로
#define BIT(x)                   (1UL << (x))
#define ESC_PRAM_CMD_BUSY        BIT(31)
#define ESC_PRAM_CMD_ABORT       BIT(30)
#define ESC_PRAM_CMD_CNT(x)      (((x) >> 8) & 0x1F)
#define ESC_PRAM_CMD_AVAIL       BIT(0)
#define ESC_PRAM_SIZE(x)         ((uint32_t)(x) << 16)
#define ESC_PRAM_ADDR(x)         ((uint32_t)(x) << 0)

#define ESC_CSR_CMD_BUSY         BIT(31)
#define ESC_CSR_CMD_READ         (BIT(31) | BIT(30))
#define ESC_CSR_CMD_WRITE        BIT(31)
#define ESC_CSR_CMD_SIZE(x)      ((uint32_t)(x) << 16)
#define ESC_RESET_CTRL_RST       BIT(6)

// -------------------------------------------------------------------
// 1. STM32용 핀 제어 매크로 (질문자님 환경에 맞게 PE2 사용)
// -------------------------------------------------------------------
#define spi_select()   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET)
#define spi_unselect() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET)

// -------------------------------------------------------------------
// 2. STM32용 하위 통신 함수 (Linux의 write/read 대체)
// -------------------------------------------------------------------
static void write(uint8_t *data, uint16_t len) {
    HAL_SPI_Transmit(&hspi1, data, len, HAL_MAX_DELAY);
}

static void read(uint8_t *buf, uint16_t len) {
    HAL_SPI_Receive(&hspi1, buf, len, HAL_MAX_DELAY);
}

// -------------------------------------------------------------------
// 아래부터는 SOES 원본 로직 100% 동일
// -------------------------------------------------------------------

/* lan9252 single write */
//static void lan9252_write_32 (uint16_t address, uint32_t val)
//{
//    uint8_t data[7];
//    data[0] = ESC_CMD_SERIAL_WRITE;
//    data[1] = ((address >> 8) & 0xFF);
//    data[2] = (address & 0xFF);
//    data[3] = (val & 0xFF);
//    data[4] = ((val >> 8) & 0xFF);
//    data[5] = ((val >> 16) & 0xFF);
//    data[6] = ((val >> 24) & 0xFF);
//
//    spi_select();
//    write(data, 7);
//    spi_unselect();
//}
static void lan9252_write_32 (uint16_t address, uint32_t val)
{
    uint8_t data[7];
    data[0] = ESC_CMD_SERIAL_WRITE; // 0x02
    data[1] = ((address >> 8) & 0xFF);
    data[2] = (address & 0xFF);
    data[3] = (val & 0xFF);
    data[4] = ((val >> 8) & 0xFF);
    data[5] = ((val >> 16) & 0xFF);
    data[6] = ((val >> 24) & 0xFF);

    spi_select();
    HAL_SPI_Transmit(&hspi1, data, 7, HAL_MAX_DELAY);
    spi_unselect();
}

/* lan9252 single read */
//static uint32_t lan9252_read_32 (uint32_t address)
//{
//   uint8_t data[4];
//   uint8_t result[4];
//
//   data[0] = ESC_CMD_FAST_READ;
//   data[1] = ((address >> 8) & 0xFF);
//   data[2] = (address & 0xFF);
//   data[3] = 0x00; // Dummy
//
//   spi_select();
//   write(data, 4);
//   read(result, 4);
//   spi_unselect();
//
//   return ((result[3] << 24) | (result[2] << 16) | (result[1] << 8) | result[0]);
//}
static uint32_t lan9252_read_32 (uint32_t address)
{
   uint8_t tx_buf[8] = {0x0B, (uint8_t)(address >> 8), (uint8_t)address, 0x00, 0, 0, 0, 0};
   uint8_t rx_buf[8] = {0};

   spi_select();
   // 명령어 4바이트 + 데이터 4바이트를 한 번에 교환
   HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 8, HAL_MAX_DELAY);
   spi_unselect();

   // rx_buf[4]~[7]에 실제 데이터가 들어있음 (리틀 엔디언 방식 조립)
   return ((uint32_t)rx_buf[7] << 24) | ((uint32_t)rx_buf[6] << 16) |
          ((uint32_t)rx_buf[5] << 8)  | (uint32_t)rx_buf[4];
}

/* ESC read CSR function */
static void ESC_read_csr (uint16_t address, void *buf, uint16_t len)
{
   uint32_t value;

   value = (ESC_CSR_CMD_READ | ESC_CSR_CMD_SIZE(len) | address);
   lan9252_write_32(ESC_CSR_CMD_REG, value);

   do {
      value = lan9252_read_32(ESC_CSR_CMD_REG);
   } while(value & ESC_CSR_CMD_BUSY);

   value = lan9252_read_32(ESC_CSR_DATA_REG);
   memcpy(buf, (uint8_t *)&value, len);
}

/* ESC write CSR function */
static void ESC_write_csr (uint16_t address, void *buf, uint16_t len)
{
   uint32_t value = 0;

   memcpy((uint8_t*)&value, buf, len);
   lan9252_write_32(ESC_CSR_DATA_REG, value);

   value = (ESC_CSR_CMD_WRITE | ESC_CSR_CMD_SIZE(len) | address);
   lan9252_write_32(ESC_CSR_CMD_REG, value);

   do {
      value = lan9252_read_32(ESC_CSR_CMD_REG);
   } while(value & ESC_CSR_CMD_BUSY);
}

/* ESC read process data ram function */
static void ESC_read_pram (uint16_t address, void *buf, uint16_t len)
{
   uint32_t value;
   uint8_t * temp_buf = buf;
   uint16_t byte_offset = 0;
   uint8_t fifo_cnt, first_byte_position, temp_len, data[4];

   value = ESC_PRAM_CMD_ABORT;
   lan9252_write_32(ESC_PRAM_RD_CMD_REG, value);

   do {
      value = lan9252_read_32(ESC_PRAM_RD_CMD_REG);
   } while(value & ESC_PRAM_CMD_BUSY);

   value = ESC_PRAM_SIZE(len) | ESC_PRAM_ADDR(address);
   lan9252_write_32(ESC_PRAM_RD_ADDR_LEN_REG, value);

   value = ESC_PRAM_CMD_BUSY;
   lan9252_write_32(ESC_PRAM_RD_CMD_REG, value);

   do {
      value = lan9252_read_32(ESC_PRAM_RD_CMD_REG);
   } while((value & ESC_PRAM_CMD_AVAIL) == 0);

   fifo_cnt = ESC_PRAM_CMD_CNT(value);
   value = lan9252_read_32(ESC_PRAM_RD_FIFO_REG);
   fifo_cnt--;

   first_byte_position = (address & 0x03);
   temp_len = ((4 - first_byte_position) > len) ? len : (4 - first_byte_position);

   memcpy(temp_buf, ((uint8_t *)&value + first_byte_position), temp_len);
   len -= temp_len;
   byte_offset += temp_len;

   spi_select();
   data[0] = ESC_CMD_FAST_READ;
   data[1] = ((ESC_PRAM_RD_FIFO_REG >> 8) & 0xFF);
   data[2] = (ESC_PRAM_RD_FIFO_REG & 0xFF);
   data[3] = 0x00;
   write(data, 4);

   while(len > 0) {
      temp_len = (len > 4) ? 4: len;
      read((temp_buf + byte_offset), 4);
      fifo_cnt--;
      len -= temp_len;
      byte_offset += temp_len;
   }
   spi_unselect();
}

/* ESC write process data ram function */
static void ESC_write_pram (uint16_t address, void *buf, uint16_t len)
{
   uint32_t value;
   uint8_t * temp_buf = buf;
   uint16_t byte_offset = 0;
   uint8_t fifo_cnt, first_byte_position, temp_len, data[3];

   value = ESC_PRAM_CMD_ABORT;
   lan9252_write_32(ESC_PRAM_WR_CMD_REG, value);

   do {
      value = lan9252_read_32(ESC_PRAM_WR_CMD_REG);
   } while(value & ESC_PRAM_CMD_BUSY);

   value = ESC_PRAM_SIZE(len) | ESC_PRAM_ADDR(address);
   lan9252_write_32(ESC_PRAM_WR_ADDR_LEN_REG, value);

   value = ESC_PRAM_CMD_BUSY;
   lan9252_write_32(ESC_PRAM_WR_CMD_REG, value);

   do {
      value = lan9252_read_32(ESC_PRAM_WR_CMD_REG);
   } while((value & ESC_PRAM_CMD_AVAIL) == 0);

   fifo_cnt = ESC_PRAM_CMD_CNT(value);
   first_byte_position = (address & 0x03);
   temp_len = ((4 - first_byte_position) > len) ? len : (4 - first_byte_position);

   value = 0;
   memcpy(((uint8_t *)&value + first_byte_position), temp_buf, temp_len);
   lan9252_write_32(ESC_PRAM_WR_FIFO_REG, value);

   len -= temp_len;
   byte_offset += temp_len;
   fifo_cnt--;

   spi_select();
   data[0] = ESC_CMD_SERIAL_WRITE;
   data[1] = ((ESC_PRAM_WR_FIFO_REG >> 8) & 0xFF);
   data[2] = (ESC_PRAM_WR_FIFO_REG & 0xFF);
   write(data, 3);

   while(len > 0) {
      temp_len = (len > 4) ? 4 : len;
      value = 0;
      memcpy((uint8_t *)&value, (temp_buf + byte_offset), temp_len);
      write((uint8_t *)&value, 4);
      fifo_cnt--;
      len -= temp_len;
      byte_offset += temp_len;
   }
   spi_unselect();
}

/** ESC read function used by the Slave stack. */
void ESC_read (uint16_t address, void *buf, uint16_t len)
{
   if (address >= 0x1000) {
      ESC_read_pram(address, buf, len);
   } else {
      uint16_t size;
      uint8_t *temp_buf = (uint8_t *)buf;

      while(len > 0) {
         size = (len > 4) ? 4 : len;
         if(address & BIT(0)) size = 1;
         else if (address & BIT(1)) size = (size & BIT(0)) ? 1 : 2;
         else if (size == 3) size = 1;

         ESC_read_csr(address, temp_buf, size);
         len -= size;
         temp_buf += size;
         address += size;
      }
   }
   // ALevent 갱신 로직
   ESC_read_csr(0x0220, (void *)&ESCvar.ALevent, 2);
   ESCvar.ALevent = etohs(ESCvar.ALevent);
}

/** ESC write function used by the Slave stack. */
void ESC_write (uint16_t address, void *buf, uint16_t len)
{
   if (address >= 0x1000) {
      ESC_write_pram(address, buf, len);
   } else {
      uint16_t size;
      uint8_t *temp_buf = (uint8_t *)buf;

      while(len > 0) {
         size = (len > 4) ? 4 : len;
         if(address & BIT(0)) size = 1;
         else if (address & BIT(1)) size = (size & BIT(0)) ? 1 : 2;
         else if (size == 3) size = 1;

         ESC_write_csr(address, temp_buf, size);
         len -= size;
         temp_buf += size;
         address += size;
      }
   }
   ESC_read_csr(0x0220, (void *)&ESCvar.ALevent, 2);
   ESCvar.ALevent = etohs(ESCvar.ALevent);
}

void ESC_init(const esc_cfg_t * config)
{
   uint32_t value = 0;

   // 칩 리셋 명령 전송
   lan9252_write_32(ESC_RESET_CTRL_REG, ESC_RESET_CTRL_RST);

   // 리셋이 완료될 때까지 대기
   do {
      value = lan9252_read_32(ESC_RESET_CTRL_REG);
   } while(value & ESC_RESET_CTRL_RST);

   HAL_Delay(500); // 안정화 대기
}


