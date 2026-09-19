#pragma once
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

bool W25Q_Init(void);
bool W25Q_ChipErase(void);
bool W25Q_SectorErase4K(uint32_t addr);
bool W25Q_Write(uint32_t addr, const uint8_t *data, uint32_t len); // handles page-boundary splitting
bool W25Q_Read(uint32_t addr, uint8_t *data, uint32_t len);
bool W25Q_ReadJedecID(uint8_t *mfgId, uint8_t *memType, uint8_t *capacity);