#include "ff.h"
#include "logging.h"
#include <stdint.h>

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
            sprintf(filename, "flightData%d.csv", fileIdx);
            sprintf(errorFilename, "errorLog%d.txt", fileIdx);
            sprintf(fixedRateLogFilename, "ekflog%d.csv", fileIdx);
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
    static long lastTimeLoggedEKF = 0;

    if (millis() - lastTimeFlushedFiles >= 2000) {
        lastTimeFlushedFiles = millis();
        ctx->errorLogFile.flush();
        ctx->logFile.flush();
        ctx->fixedRateLogFile.flush();
    }

    if(millis() - lastTimeLoggedEKF >= 50) {
        lastTimeLoggedEKF = millis();
        // BLA::Matrix<10, 1> ekfState = ctx->estimator.getPVState();

        // LogSensorData ekfStateData = {
        //     .ekf_state = {
        //         .w = ekfState(0, 0),
        //         .i = ekfState(1, 0),
        //         .j = ekfState(2, 0),
        //         .k = ekfState(3, 0),
        //         .velX = ekfState(4, 0),
        //         .velY = ekfState(5, 0),
        //         .velZ = ekfState(6, 0),
        //         .posX = ekfState(7, 0),
        //         .posY = ekfState(8, 0),
        //         .posZ = ekfState(9, 0),
        //         .gyroBX = ekfState(10, 0),
        //         .gyroBY = ekfState(11, 0),
        //         .gyroBZ = ekfState(12, 0),
        //         .accelBX = ekfState(13, 0),
        //         .accelBY = ekfState(14, 0),
        //         .accelBZ = ekfState(15, 0),
        //         .magBX = ekfState(16, 0),
        //         .magBY = ekfState(17, 0),
        //         .magBZ = ekfState(18, 0),
        //         .baroB = ekfState(19, 0),
        //     }
        // };
        // writePacket(&ctx->fixedRateLogFile, lastTimeLoggedFixedRate, &ekfStateData, EKF_STATE_TAG);

        // BLA::Matrix<10, 1> ekfP = ctx->estimator.getPVPDiag();
        // LogSensorData ekfPData = {
        //     .ekf_p = {
        //         .P0 = ekfP(0, 0),
        //         .P1 = ekfP(1, 0),
        //         .P3 = ekfP(2, 0),
        //         .P4 = ekfP(3, 0),
        //         .P5 = ekfP(4, 0),
        //         .P6 = ekfP(5, 0),
        //         .P7 = ekfP(6, 0),
        //         .P8 = ekfP(7, 0),
        //         .P9 = ekfP(8, 0),
        //         .P10 = ekfP(9, 0),
        //         .P11 = ekfP(10, 0),
        //         .P12 = ekfP(11, 0),
        //         .P13 = ekfP(12, 0),
        //         .P14 = ekfP(13, 0),
        //         .P15 = ekfP(14, 0),
        //         .P16 = ekfP(15, 0),
        //         .P17 = ekfP(16, 0),
        //         .P18 = ekfP(17, 0),
        //     }
        // };
        // writePacket(&ctx->fixedRateLogFile, lastTimeLoggedFixedRate, &ekfPData, EKF_P_TAG);
    }

    static flatbuffers::FlatBufferBuilder builder;

    static long lastAccelDataAt = 0;
    const auto &accel_desc = ctx->asm330.get_descriptor();
    if (accel_desc.getLastUpdated() > lastAccelDataAt) {
        lastAccelDataAt = accel_desc.getLastUpdated();
        hprc::ASM330Data d(accel_desc.data.accel0, accel_desc.data.accel1,
                           accel_desc.data.accel2, accel_desc.data.gyr0,
                           accel_desc.data.gyr1, accel_desc.data.gyr2);

        logSensorData(&builder, &ctx->logFile, hprc::SensorData_ASM330,(hprc::SensorData *const) &d);
    }

    static long lastBaroDataAt = 0;
    const auto &baro_desc = ctx->baro.get_descriptor();
    if (baro_desc.getLastUpdated() > lastBaroDataAt) {
        lastBaroDataAt = baro_desc.getLastUpdated();
        hprc::LPS22Data d(baro_desc.data.pressure, baro_desc.data.temp);

        logSensorData(&builder, &ctx->logFile, hprc::SensorData_LPS22, (hprc::SensorData *const) &d);
    }

    static long lastMagDataAt = 0;
    const auto &mag_desc = ctx->mag.get_descriptor();
    if (mag_desc.getLastUpdated() > lastMagDataAt) {
        lastMagDataAt = mag_desc.getLastUpdated();
        hprc::LIS2MDLData d(mag_desc.data.mag0, mag_desc.data.mag1,
                            mag_desc.data.mag2);

        logSensorData(&builder, &ctx->logFile, hprc::SensorData_LIS2MDL, (hprc::SensorData *const) &d);
    }

    static long lastGpsDataAt = 0;
    const auto &gps_desc = ctx->gps.get_descriptor();
    if (gps_desc.getLastUpdated() > lastGpsDataAt) {
        lastGpsDataAt = gps_desc.getLastUpdated();
        hprc::LIV3FData d(gps_desc.data.lat, gps_desc.data.lon,
                          gps_desc.data.alt, gps_desc.data.satellites,
                          gps_desc.data.epochTime);
        
        logSensorData(&builder, &ctx->logFile, hprc::SensorData_LIV3F, (hprc::SensorData *const) &d);
    }
}

void logSensorData(flatbuffers::FlatBufferBuilder *builder, File *logFile, hprc::SensorData data_type, hprc::SensorData *const &d) {
    builder->Clear();

    auto sensorData = builder->CreateStruct(d);

    auto packet = hprc::CreateSDPacket(*builder, millis(), data_type, sensorData.Union());

    builder->FinishSizePrefixed(packet);

    logFile->write(builder->GetBufferPointer(), builder->GetSize());
}