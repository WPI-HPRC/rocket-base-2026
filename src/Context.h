#pragma once

#include <SdFat.h>
#include "boilerplate/Sensors/Impl/ASM330.h"
#include "boilerplate/Sensors/Impl/ICM20948.h"
//#include "boilerplate/Sensors/Impl/INA219.h"
#include "boilerplate/Sensors/Impl/LPS22.h"
#include "boilerplate/Sensors/Impl/MAX10S.h"
#include "boilerplate/Servo.h"
#include "boilerplate/qmekf-lib/qmekf.h"

struct ASM330Data;
struct LPS22Data;
struct ICMData;
struct MAX10SData;
struct INA219Data;

struct Context {
    File logFile;
    File errorLogFile;
    File fixedRateLogFile;
    SdFs sd;
    bool sdInitialized;
    bool ekfLooping;

    ASM330 accel;
    LPS22 baro;
    ICM20948 mag;
    MAX10S gps;
    //INA219 curr;
    
    Servo airBrakes;
    StateEstimator estimator;
};
