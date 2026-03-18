#pragma once

#include <stdint.h>
//#include "Sensors/ASM330.h"
//#include "Sensors/LSP22.h"
//#include "Sensors/ICM20948.h"
//#include "Sensors/MAX10S.h"
//#include "Sensors/INA219.h"
#include "Context.h"

enum SensorType {
    ASM330_TAG = 0,
    LPS22_TAG = 1,
    ICM20948_TAG = 2,
    MAX10S_TAG = 3,
    INA219_TAG = 4,
    EKF_STATE_TAG = 5,
    EKF_P_TAG = 6
};

struct ekfState {
    float w;
    float i;
    float j;
    float k;
    float velX;
    float velY;
    float velZ;
    float posX;
    float posY;
    float posZ;
    float gyroBX;
    float gyroBY;
    float gyroBZ;
    float accelBX;
    float accelBY;
    float accelBZ;
    float magBX;
    float magBY;
    float magBZ;
    float baroB;
};

struct ekfP {
    float P0;
    float P1;
    float P3;
    float P4;
    float P5;
    float P6;
    float P7;
    float P8;
    float P9;
    float P10;
    float P11;
    float P12;
    float P13;
    float P14;
    float P15;
    float P16;
    float P17;
    float P18;
};

union LogSensorData {
    ASM330Data asm330;
    LPS22Data lps22;
    ICMData icm20948;
    MAX10SData max10s;
    INA219Data ina219;
    ekfState ekf_state;
    ekfP ekf_p;
};

#pragma pack(push, 1)
struct Packet {
    uint8_t id; // NOTE: should identify the sensor and the length of this packet
    uint32_t timeStamp;
    LogSensorData data;
};
#pragma pack(pop)

bool initializeLogging(Context *ctx);

void loggingLoop(Context *ctx);

void writePacket(File *logFile, uint32_t timestamp, LogSensorData data, SensorType type);
