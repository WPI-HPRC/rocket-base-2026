#pragma once
#include "w25q_qspi.h"

class FlashLog {
public:
  static const uint32_t QSPI_FLASH_SIZE = 256*65536; // W25Q128JV

  bool begin(bool erase = false) {
    if (!W25Q_Init()) return false;
    // Serial.println("inited");

    uint8_t mfg, type, cap;
    if (!W25Q_ReadJedecID(&mfg, &type, &cap)) return false;
    // Serial.println("read id");
    // Serial.println(mfg);
    // if (mfg != 0xEF) return false; // 0xEF = Winbond; sanity check wiring/init before erasing
    Serial.println("Id matched");

    if(erase){
        if (!W25Q_ChipErase()) return false;
    }
    _writePtr = 0;
    return true;
  }

  bool append(const uint8_t *data, uint32_t len) {
    uint32_t totalLen = 4 + len;
    if (_writePtr + totalLen > QSPI_FLASH_SIZE) return false;

    if (!W25Q_Write(_writePtr, (uint8_t*)&len, 4)) return false;
    _writePtr += 4;
    if (!W25Q_Write(_writePtr, data, len)) return false;
    _writePtr += len;
    return true;
  }

  bool readAt(uint32_t addr, uint32_t *outLen, uint8_t *buf, uint32_t bufCap) {
    if (!W25Q_Read(addr, (uint8_t*)outLen, 4)) return false;
    if (*outLen > bufCap) return false;
    return W25Q_Read(addr + 4, buf, *outLen);
  }

  uint32_t bytesUsed() const { return _writePtr; }
  bool isFull() const { return _writePtr >= QSPI_FLASH_SIZE; }

private:
  uint32_t _writePtr = 0;
};