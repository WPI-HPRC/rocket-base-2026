#pragma once

#include <STM32SD.h>
#include <Servo.h>
#include "boilerplate/Sensors/Impl/ASM330.h"
#include "boilerplate/Sensors/Impl/LIS2MDLTR.h"
#include "boilerplate/Sensors/Impl/LIV3F.h"
#include "boilerplate/Sensors/Impl/LPS22.h"
#include "boilerplate/Sensors/Impl/LSM6.h"
#include "boilerplate/qmekf-lib/include/split_mekf.h"

#include "config.h"

#include "LoRaE22.h"

struct ASM330Data;
struct LPS22Data;
struct ICMData;
struct MAX10SData;
struct INA219Data;

#define LOG_FILE_BUFFER_SIZE 16000

struct Context {
    File logFile;
    char logFileBuffer[LOG_FILE_BUFFER_SIZE];
    size_t logFileBufferEnd = 0;
    File debugLogFile;
    File fixedRateLogFile;
    bool sdInitialized;
    bool ekfLooping;

    ASM330 asm330;
    LSM6 lsm;
    LPS22 baro;
    LIS2MDL mag;
    LIV3F gps;

    LoRaE22 radio;
    
    SplitStateEstimator estimator;
};
