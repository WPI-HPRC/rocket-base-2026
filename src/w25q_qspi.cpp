#include "w25q_qspi.h"

static QSPI_HandleTypeDef hqspi;

#define CMD_WRITE_ENABLE      0x06
#define CMD_READ_SR1          0x05
#define CMD_READ_SR2          0x35
#define CMD_WRITE_SR2         0x31
#define CMD_SECTOR_ERASE_4K   0x20
#define CMD_CHIP_ERASE        0xC7
#define CMD_PAGE_PROGRAM      0x02
#define CMD_FAST_READ_QUAD    0xEB
#define CMD_READ_JEDEC_ID     0x9F

#define SR1_BUSY_MASK  0x01
#define SR2_QE_MASK    0x02

#define PAGE_SIZE   256UL

// ---------- GPIO / peripheral init ----------

static void QSPI_GPIO_Init(void) {
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {0};
  gpio.Mode  = GPIO_MODE_AF_PP;
  gpio.Pull  = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  gpio.Alternate = GPIO_AF9_QUADSPI;                 // CLK, IO2, IO3
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_10;
  HAL_GPIO_Init(GPIOF, &gpio);

  gpio.Alternate = GPIO_AF10_QUADSPI;                // IO0, IO1
  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOF, &gpio);

  gpio.Alternate = GPIO_AF10_QUADSPI;                // NCS
  gpio.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOG, &gpio);
}

static bool QSPI_PeriphInit(void) {
  __HAL_RCC_QSPI_CLK_ENABLE();
  __HAL_RCC_QSPI_FORCE_RESET();
  __HAL_RCC_QSPI_RELEASE_RESET();

  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler     = 1;   // tune vs your AHB clock / flash max freq (104MHz for W25Q128JV std, less for fast-read w/ dummy cycles)
  hqspi.Init.FifoThreshold      = 4;
  hqspi.Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize          = 23;  // 2^(23+1) = 16MB
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_6_CYCLE;
  hqspi.Init.ClockMode          = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID            = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash          = QSPI_DUALFLASH_DISABLE;

  return HAL_QSPI_Init(&hqspi) == HAL_OK;
}

// ---------- low-level command helpers ----------

static bool WriteEnable(void) {
  QSPI_CommandTypeDef cmd = {0};
  cmd.Instruction     = CMD_WRITE_ENABLE;
  cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  cmd.AddressMode     = QSPI_ADDRESS_NONE;
  cmd.DataMode        = QSPI_DATA_NONE;
  return HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
}

static bool ReadReg(uint8_t instruction, uint8_t *val) {
  QSPI_CommandTypeDef cmd = {0};
  cmd.Instruction     = instruction;
  cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  cmd.AddressMode     = QSPI_ADDRESS_NONE;
  cmd.DataMode        = QSPI_DATA_1_LINE;
  cmd.NbData          = 1;
  if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;
  return HAL_QSPI_Receive(&hqspi, val, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
}

static bool WriteReg(uint8_t instruction, uint8_t val) {
  QSPI_CommandTypeDef cmd = {0};
  cmd.Instruction     = instruction;
  cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  cmd.AddressMode     = QSPI_ADDRESS_NONE;
  cmd.DataMode        = QSPI_DATA_1_LINE;
  cmd.NbData          = 1;
  if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;
  return HAL_QSPI_Transmit(&hqspi, &val, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
}

static bool WaitBusy(uint32_t timeoutMs) {
  QSPI_CommandTypeDef cmd = {0};
  cmd.Instruction     = CMD_READ_SR1;
  cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  cmd.DataMode        = QSPI_DATA_1_LINE;
  cmd.NbData          = 1;

  QSPI_AutoPollingTypeDef cfg = {0};
  cfg.Match           = 0x00;
  cfg.Mask            = SR1_BUSY_MASK;
  cfg.MatchMode       = QSPI_MATCH_MODE_AND;
  cfg.StatusBytesSize = 1;
  cfg.Interval        = 0x10;
  cfg.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  return HAL_QSPI_AutoPolling(&hqspi, &cmd, &cfg, timeoutMs) == HAL_OK;
}

static bool EnableQuadMode(void) {
  uint8_t sr2 = 0;
  if (!ReadReg(CMD_READ_SR2, &sr2)) return false;
  if (sr2 & SR2_QE_MASK) return true;   // already enabled

  if (!WriteEnable()) return false;
  if (!WriteReg(CMD_WRITE_SR2, sr2 | SR2_QE_MASK)) return false;
  return WaitBusy(100);
}

// ---------- public API ----------

bool W25Q_Init(void) {
  QSPI_GPIO_Init();
  if (!QSPI_PeriphInit()) return false;
  return EnableQuadMode();
}

bool W25Q_ChipErase(void) {
  if (!WriteEnable()) return false;

  QSPI_CommandTypeDef cmd = {0};
  cmd.Instruction     = CMD_CHIP_ERASE;
  cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  cmd.AddressMode     = QSPI_ADDRESS_NONE;
  cmd.DataMode        = QSPI_DATA_NONE;
  if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

  return WaitBusy(200000); // datasheet max ~200s for chip erase
}

bool W25Q_SectorErase4K(uint32_t addr) {
  if (!WriteEnable()) return false;

  QSPI_CommandTypeDef cmd = {0};
  cmd.Instruction     = CMD_SECTOR_ERASE_4K;
  cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  cmd.AddressMode     = QSPI_ADDRESS_1_LINE;
  cmd.AddressSize     = QSPI_ADDRESS_24_BITS;
  cmd.Address         = addr;
  cmd.DataMode        = QSPI_DATA_NONE;
  if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

  return WaitBusy(1000); // datasheet max ~400ms
}

static bool PageProgram(uint32_t addr, const uint8_t *data, uint32_t len) {
  if (!WriteEnable()) return false;

  QSPI_CommandTypeDef cmd = {0};
  cmd.Instruction     = CMD_PAGE_PROGRAM;
  cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  cmd.AddressMode     = QSPI_ADDRESS_1_LINE;
  cmd.AddressSize     = QSPI_ADDRESS_24_BITS;
  cmd.Address         = addr;
  cmd.DataMode        = QSPI_DATA_1_LINE;
  cmd.NbData          = len;
  if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;
  if (HAL_QSPI_Transmit(&hqspi, (uint8_t*)data, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

  return WaitBusy(50); // datasheet max ~3ms
}

bool W25Q_Write(uint32_t addr, const uint8_t *data, uint32_t len) {
  uint32_t written = 0;
  while (written < len) {
    uint32_t curAddr    = addr + written;
    uint32_t pageOffset = curAddr % PAGE_SIZE;
    uint32_t chunk      = PAGE_SIZE - pageOffset;
    if (chunk > (len - written)) chunk = len - written;

    if (!PageProgram(curAddr, data + written, chunk)) return false;
    written += chunk;
  }
  return true;
}

bool W25Q_Read(uint32_t addr, uint8_t *data, uint32_t len) {
  QSPI_CommandTypeDef cmd = {0};
  cmd.Instruction         = CMD_FAST_READ_QUAD;
  cmd.InstructionMode     = QSPI_INSTRUCTION_1_LINE;
  cmd.AddressMode         = QSPI_ADDRESS_4_LINES;
  cmd.AddressSize         = QSPI_ADDRESS_24_BITS;
  cmd.Address             = addr;
  cmd.AlternateByteMode   = QSPI_ALTERNATE_BYTES_4_LINES;
  cmd.AlternateBytesSize  = QSPI_ALTERNATE_BYTES_8_BITS;
  cmd.AlternateBytes      = 0xFF;  // disable continuous-read mode
  cmd.DataMode            = QSPI_DATA_4_LINES;
  cmd.DummyCycles         = 4;     // W25Q128JV: 4 dummy cycles at default (non-high-perf) SCLK for 0xEB
  cmd.NbData              = len;

  if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;
  return HAL_QSPI_Receive(&hqspi, data, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
}

bool W25Q_ReadJedecID(uint8_t *mfgId, uint8_t *memType, uint8_t *capacity) {
  QSPI_CommandTypeDef cmd = {0};
  cmd.Instruction     = CMD_READ_JEDEC_ID;
  cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  cmd.AddressMode     = QSPI_ADDRESS_NONE;
  cmd.DataMode        = QSPI_DATA_1_LINE;
  cmd.NbData          = 3;
  if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;

  uint8_t buf[3];
  if (HAL_QSPI_Receive(&hqspi, buf, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return false;
  *mfgId = buf[0]; *memType = buf[1]; *capacity = buf[2];
  return true;
}