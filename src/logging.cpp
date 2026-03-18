#include <SdFat.h>
#include <stdint.h>
#include "logging.h"
//#include "Sensors/ASM330.h"
#include "config.h"

size_t dataLengths[] = {
    sizeof(ASM330Data),
    sizeof(LPS22Data), // Length of s2 data in Bytes
    sizeof(ICMData),
    sizeof(MAX10SData),
    sizeof(INA219Data),
    sizeof(ekfState),
};

bool initializeLogging(Context *ctx) {
    Serial.print("Initailizing SD... ");
    SPI.setSCLK(SD_SCLK);
    SPI.setMISO(SD_MISO);
    SPI.setMOSI(SD_MOSI);
    SPI.begin();

    if (ctx->sd.begin(SD_CS, SD_SPI_SPEED)) { 
        // TODO: Define these values
        int fileIdx = 0;
        char filename[100];
        char errorFilename[100];
        char fixedRateLogFilename[100];
        while (fileIdx < 100)
        {
            sprintf(filename, "flightData%d.csv", fileIdx);
            sprintf(errorFilename, "errorLog%d.txt", fileIdx++);
            sprintf(fixedRateLogFilename, "fixedRateLog%d.csv", fileIdx);

            Serial.printf("Trying files `%s/%s`\n", filename, errorFilename);
            if (!ctx->sd.exists(filename))
            {
                ctx->logFile = ctx->sd.open(filename, O_RDWR | O_CREAT | O_TRUNC);
                ctx->errorLogFile = ctx->sd.open(errorFilename, O_RDWR | O_CREAT | O_TRUNC);
                ctx->fixedRateLogFile = ctx->sd.open(fixedRateLogFilename, O_RDWR | O_CREAT | O_TRUNC);
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
        BLA::Matrix<20, 1> ekfState = ctx->estimator.getState();

        LogSensorData ekfStateData = {
            .ekf_state = {
                .w = ekfState(0, 0),
                .i = ekfState(1, 0),
                .j = ekfState(2, 0),
                .k = ekfState(3, 0),
                .velX = ekfState(4, 0),
                .velY = ekfState(5, 0),
                .velZ = ekfState(6, 0),
                .posX = ekfState(7, 0),
                .posY = ekfState(8, 0),
                .posZ = ekfState(9, 0),
                .gyroBX = ekfState(10, 0),
                .gyroBY = ekfState(11, 0),
                .gyroBZ = ekfState(12, 0),
                .accelBX = ekfState(13, 0),
                .accelBY = ekfState(14, 0),
                .accelBZ = ekfState(15, 0),
                .magBX = ekfState(16, 0),
                .magBY = ekfState(17, 0),
                .magBZ = ekfState(18, 0),
                .baroB = ekfState(19, 0),
            }
        };
        writePacket(&ctx->fixedRateLogFile, lastTimeLoggedFixedRate, ekfStateData, EKF_STATE_TAG);

        BLA::Matrix<19, 1> ekfP = ctx->estimator.getPDiag();
        LogSensorData ekfPData = {
            .ekf_p = {
                .P0 = ekfP(0, 0),
                .P1 = ekfP(1, 0),
                .P3 = ekfP(2, 0),
                .P4 = ekfP(3, 0),
                .P5 = ekfP(4, 0),
                .P6 = ekfP(5, 0),
                .P7 = ekfP(6, 0),
                .P8 = ekfP(7, 0),
                .P9 = ekfP(8, 0),
                .P10 = ekfP(9, 0),
                .P11 = ekfP(10, 0),
                .P12 = ekfP(11, 0),
                .P13 = ekfP(12, 0),
                .P14 = ekfP(13, 0),
                .P15 = ekfP(14, 0),
                .P16 = ekfP(15, 0),
                .P17 = ekfP(16, 0),
                .P18 = ekfP(17, 0),
            }
        };
        writePacket(&ctx->fixedRateLogFile, lastTimeLoggedFixedRate, ekfPData, EKF_P_TAG);

        const auto &accel_desc = ctx->accel.get_descriptor();
        LogSensorData accel = {
            .asm330 = accel_desc.data
        };
        writePacket(&ctx->fixedRateLogFile, lastTimeLoggedFixedRate, accel, ASM330_TAG);

        const auto &baro_desc = ctx->baro.get_descriptor();
        LogSensorData baro = {
            .lps22 = baro_desc.data
        };
        writePacket(&ctx->fixedRateLogFile, lastTimeLoggedFixedRate, baro, LPS22_TAG);
        
        const auto &mag_desc = ctx->mag.get_descriptor();
        LogSensorData mag = {
            .icm20948 = mag_desc.data
        };
        writePacket(&ctx->fixedRateLogFile, lastTimeLoggedFixedRate, mag, ICM20948_TAG);

        const auto &gps_desc = ctx->gps.get_descriptor();
        LogSensorData gps = {
            .max10s = gps_desc.data
        };
        writePacket(&ctx->fixedRateLogFile, lastTimeLoggedFixedRate, gps, MAX10S_TAG);

        const auto &curr_desc = ctx->curr.get_descriptor();
        LogSensorData ina = {
            .ina219 = curr_desc.data
        };
        writePacket(&ctx->fixedRateLogFile, lastTimeLoggedFixedRate, ina, INA219_TAG);
    }

    static long lastAccelDataAt = 0;
    const auto &accel_desc = ctx->accel.get_descriptor();
    if (accel_desc.getLastUpdated() > lastAccelDataAt) {
        lastAccelDataAt = accel_desc.getLastUpdated();
        LogSensorData d = {
            .asm330 = accel_desc.data
        };
        writePacket(&ctx->logFile, lastAccelDataAt, d, ASM330_TAG);
    }

    static long lastBaroDataAt = 0;
    const auto &baro_desc = ctx->baro.get_descriptor();
    if (baro_desc.getLastUpdated() > lastBaroDataAt) {
        lastBaroDataAt = baro_desc.getLastUpdated();
        LogSensorData d = {
            .lps22 = baro_desc.data
        };
        writePacket(&ctx->logFile, lastBaroDataAt, d, LPS22_TAG);
    }

    static long lastMagDataAt = 0;
    const auto &mag_desc = ctx->mag.get_descriptor();
    if (mag_desc.getLastUpdated() > lastMagDataAt) {
        lastMagDataAt = mag_desc.getLastUpdated();
        LogSensorData d = {
            .icm20948 = mag_desc.data
        };
        writePacket(&ctx->logFile, lastMagDataAt, d, ICM20948_TAG);
    }

    static long lastGpsDataAt = 0;
    const auto &gps_desc = ctx->gps.get_descriptor();
    if (gps_desc.getLastUpdated() > lastGpsDataAt) {
        lastGpsDataAt = gps_desc.getLastUpdated();
        LogSensorData d = {
            .max10s = gps_desc.data
        };
        writePacket(&ctx->logFile, lastGpsDataAt, d, MAX10S_TAG);
    }

    static long lastCurrentDataAt = 0;
    const auto &curr_desc = ctx->curr.get_descriptor();
    if (curr_desc.getLastUpdated() > lastCurrentDataAt) {
        lastCurrentDataAt = curr_desc.getLastUpdated();
        LogSensorData d = {
            .ina219 = curr_desc.data
        };
        writePacket(&ctx->logFile, lastCurrentDataAt, d, INA219_TAG);
    }
}

void writePacket(File *logFile, uint32_t timestamp, LogSensorData data, SensorType type) {
    Packet packetToWrite = { type, timestamp, data};

    size_t length = sizeof(uint8_t) + sizeof(uint32_t) + dataLengths[type];

    logFile->write(&packetToWrite, length);
}