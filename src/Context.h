#pragma once

#include <STM32SD.h>
#include <Servo.h>

#ifdef __has_include
#if __has_include("states/States.h")
#include "State.h"
#include "states/States.h"
#else
#include "template_states/States.h"
#define TEMPLATE_STATES_OVERRIDE
#include "State.h"
#endif
#else
#warning No __has_include, falling back to template_states
#include "template_states/States.h"
#endif

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
    File debugLogFile;
    File ekfLogFile;

    uint8_t logFileIdx;
    char logFileBuffer[LOG_FILE_BUFFER_SIZE];
    size_t logFileBufferEnd = 0;

    bool sdInitialized;
    bool ekfLooping;

    StateID currentState;

    ASM330 asm330;
    LSM6 lsm;
    LPS22 baro;
    LIS2MDL mag;
    LIV3F gps;

    LoRaE22 radio;
    
    SplitStateEstimator estimator;
};
