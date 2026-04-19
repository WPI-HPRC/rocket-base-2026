#include <stdint.h>
#include "logging.h"
#include "ff.h"

size_t dataLengths[] = {
    sizeof(ASM330Data),
    sizeof(LIS2MDLData),
    sizeof(LIV3FData),
    sizeof(LPS22Data),
    sizeof(LSM6Data),
    sizeof(ekfAttState),
    sizeof(ekfPVState),
    sizeof(AttekfP),
    sizeof(PVekfP),
};

bool initializeLogging(Context *ctx) {
    Serial.print("Initailizing SD... ");

    if (SD.begin()) { 
        // TODO: Define these values
        int fileIdx = 0;
        char filename[100];
        char errorFilename[100];
        char fixedRateLogFilename[100];
        while (fileIdx < 100)
        {
            sprintf(filename, "flightData%d.bin", fileIdx);
            sprintf(errorFilename, "stateTransition%d.txt", fileIdx);
            sprintf(fixedRateLogFilename, "fixedRateLog%d.bin", fileIdx);
            fileIdx++;

            Serial.printf("Trying files `%s/%s`\n", filename, errorFilename);
            if (!SD.exists(filename))
            {
                ctx->logFile = SD.open(filename, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
                ctx->errorLogFile = SD.open(errorFilename, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
                ctx->fixedRateLogFile = SD.open(fixedRateLogFilename, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
                break;
            }
        }
        return true;
    }
    else
    {
        // NOTE: SD initialization failed
        // Do something about that probably
        Serial.println("FAILED");
        return false;
    }
}

void loggingLoop(Context *ctx) {
    static long lastTimeFlushedFiles = 0;
    static long lastTimeLoggedFixedRate = 0;

    if (millis() - lastTimeFlushedFiles >= 2000) {
        lastTimeFlushedFiles = millis();
        ctx->errorLogFile.flush();
        ctx->logFile.flush();
        ctx->fixedRateLogFile.flush();
    }

    if(millis() - lastTimeLoggedFixedRate >= 50) {
        lastTimeLoggedFixedRate = millis();
        BLA::Matrix<13, 1> attState = ctx->estimator.getAttState();

        LogSensorData ekfAttStateData = {
            .ekfAtt_state = {
                .w = attState(0, 0),
                .i = attState(1, 0),
                .j = attState(2, 0),
                .k = attState(3, 0),
                .gyroBX = attState(4, 0),
                .gyroBY = attState(5, 0),
                .gyroBZ = attState(6, 0),
                .accelBX = attState(7, 0),
                .accelBY = attState(8, 0),
                .accelBZ = attState(9, 0),
                .magBX = attState(10, 0),
                .magBY = attState(11, 0),
                .magBZ = attState(12, 0),
            }
        };
        writePacket(&ctx->fixedRateLogFile, lastTimeLoggedFixedRate, &ekfAttStateData, EKF_ATT_STATE_TAG);

        BLA::Matrix<10, 1> PVState = ctx->estimator.getPVState();
        LogSensorData ekfPVStateData = {
            .ekfPV_state = {
                .velX = PVState(0, 0),
                .velY = PVState(1, 0),
                .velZ = PVState(2, 0),
                .posX = PVState(3, 0),
                .posY = PVState(4, 0),
                .posZ = PVState(5, 0),
                .accelBX = PVState(6, 0),
                .accelBY = PVState(7, 0),
                .accelBZ = PVState(8, 0),
            }
        };
        writePacket(&ctx->fixedRateLogFile, lastTimeLoggedFixedRate, &ekfPVStateData, EKF_PV_STATE_TAG);

        BLA::Matrix<12, 1> AttekfP = ctx->estimator.getAttPDiag();
        LogSensorData ekfAttPData = {
            .Attekf_p = {
                .P0 = AttekfP(0, 0),
                .P1 = AttekfP(1, 0),
                .P2 = AttekfP(2, 0),
                .P3 = AttekfP(3, 0),
                .P4 = AttekfP(4, 0),
                .P5 = AttekfP(5, 0),
                .P6 = AttekfP(6, 0),
                .P7 = AttekfP(7, 0),
                .P8 = AttekfP(8, 0),
                .P9 = AttekfP(9, 0),
                .P10 = AttekfP(10, 0),
                .P11 = AttekfP(11, 0),
            }
        };
        writePacket(&ctx->fixedRateLogFile, lastTimeLoggedFixedRate, &ekfAttPData, EKF_ATT_P_TAG);

        BLA::Matrix<10, 1> PVekfP = ctx->estimator.getPVPDiag();
        LogSensorData ekfPVPData = {
            .PVekf_p = {
                .P0 = PVekfP(0, 0),
                .P1 = PVekfP(1, 0),
                .P2 = PVekfP(2, 0),
                .P3 = PVekfP(3, 0),
                .P4 = PVekfP(4, 0),
                .P5 = PVekfP(5, 0),
                .P6 = PVekfP(6, 0),
                .P7 = PVekfP(7, 0),
                .P8 = PVekfP(8, 0),
                .P9 = PVekfP(9, 0),
            }
        };
        writePacket(&ctx->fixedRateLogFile, lastTimeLoggedFixedRate, &ekfPVPData, EKF_PV_P_TAG);

        const auto &accel_desc = ctx->asm330.get_descriptor();
        LogSensorData accel = {
            .asm330 = accel_desc.data
        };
        writePacket(&ctx->fixedRateLogFile, lastTimeLoggedFixedRate, &accel, ASM330_TAG);

        const auto &baro_desc = ctx->baro.get_descriptor();
        LogSensorData baro = {
            .lps22 = baro_desc.data
        };
        writePacket(&ctx->fixedRateLogFile, lastTimeLoggedFixedRate, &baro, LPS22_TAG);
        
        const auto &mag_desc = ctx->mag.get_descriptor();
        LogSensorData mag = {
            .lis2m = mag_desc.data
        };
        writePacket(&ctx->fixedRateLogFile, lastTimeLoggedFixedRate, &mag, LIS2MDLTR_TAG);

        const auto &gps_desc = ctx->gps.get_descriptor();
        LogSensorData gps = {
            .liv3f = gps_desc.data
        };
        writePacket(&ctx->fixedRateLogFile, lastTimeLoggedFixedRate, &gps, LIV3F_TAG);
    }

    static long lastAccelDataAt = 0;
    const auto &accel_desc = ctx->asm330.get_descriptor();
    if (accel_desc.getLastUpdated() > lastAccelDataAt) {
        lastAccelDataAt = accel_desc.getLastUpdated();
        LogSensorData d = {
            .asm330 = accel_desc.data
        };
        writePacket(&ctx->logFile, lastAccelDataAt, &d, ASM330_TAG);
    }

    static long lastBaroDataAt = 0;
    const auto &baro_desc = ctx->baro.get_descriptor();
    if (baro_desc.getLastUpdated() > lastBaroDataAt) {
        lastBaroDataAt = baro_desc.getLastUpdated();
        LogSensorData d = {
            .lps22 = baro_desc.data
        };
        writePacket(&ctx->logFile, lastBaroDataAt, &d, LPS22_TAG);
    }

    static long lastMagDataAt = 0;
    const auto &mag_desc = ctx->mag.get_descriptor();
    if (mag_desc.getLastUpdated() > lastMagDataAt) {
        lastMagDataAt = mag_desc.getLastUpdated();
        LogSensorData d = {
            .lis2m = mag_desc.data
        };
        writePacket(&ctx->logFile, lastMagDataAt, &d, LIS2MDLTR_TAG);
    }

    static long lastGpsDataAt = 0;
    const auto &gps_desc = ctx->gps.get_descriptor();
    if (gps_desc.getLastUpdated() > lastGpsDataAt) {
        lastGpsDataAt = gps_desc.getLastUpdated();
        LogSensorData d = {
            .liv3f = gps_desc.data
        };
        writePacket(&ctx->logFile, lastGpsDataAt, &d, LIV3F_TAG);
    }
}

void writePacket(File *logFile, uint32_t timestamp, LogSensorData *data, SensorType type) {
    Packet packetToWrite = { type, timestamp, *data};

    size_t length = sizeof(uint8_t) + sizeof(uint32_t) + dataLengths[type];

    logFile->write((const uint8_t *)&packetToWrite, length);
}
