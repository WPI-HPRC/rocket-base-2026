#include "Context.h"
#include "SdData_generated.h"
#include "Sensors_generated.h"
#include "ff.h"
#include "logging.h"
#include <stdint.h>

bool initializeLogging(Context *ctx) {
  Serial.print("Initailizing SD... ");

  if (SD.begin()) {
    int fileIdx = 0;
    char filename[256];
    char debugFilename[256];
    char ekfLogFilename[256];

    SD.mkdir(LOG_FOLDER_NAME);

    {
      File prev_log_idx_file;
      char idx_string[16];
      if (SD.exists(LOG_FOLDER_NAME "/.prev_log_idx")) {
        prev_log_idx_file = SD.open(LOG_FOLDER_NAME "/.prev_log_idx", FA_WRITE | FA_READ);
        fileIdx = prev_log_idx_file.parseInt() + 1;
        if (fileIdx > 100) {
          fileIdx = 0;
        }
      } else {
        prev_log_idx_file = SD.open(LOG_FOLDER_NAME "/.prev_log_idx", FA_CREATE_ALWAYS | FA_WRITE);
      }
      prev_log_idx_file.seek(0);
      itoa(fileIdx, idx_string, 10);
      idx_string[3] = 0;
      prev_log_idx_file.write(idx_string, 4);
      prev_log_idx_file.close();
    }
    
    sprintf(filename, LOG_FOLDER_NAME "/flightData%d.fb.bin", fileIdx);
    sprintf(debugFilename, LOG_FOLDER_NAME "/debugLog%d.txt", fileIdx);
    sprintf(ekfLogFilename, LOG_FOLDER_NAME "/ekflog%d.bin", fileIdx);

    Serial.printf("Main log file `%s`\n", filename);
    ctx->logFile = SD.open(filename, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
    ctx->debugLogFile =
        SD.open(debugFilename, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
    ctx->ekfLogFile = SD.open(ekfLogFilename,
                                      FA_CREATE_ALWAYS | FA_WRITE | FA_READ);

    ctx->logFileIdx = fileIdx;
    return true;
  } else {
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
    ctx->debugLogFile.flush();
    ctx->logFile.flush();
    ctx->ekfLogFile.flush();
  }

  if (millis() - lastTimeLoggedEKF >= 50) {
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
    // writePacket(&ctx->fixedRateLogFile, lastTimeLoggedFixedRate,
    // &ekfStateData, EKF_STATE_TAG);

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
    // writePacket(&ctx->fixedRateLogFile, lastTimeLoggedFixedRate, &ekfPData,
    // EKF_P_TAG);
  }

  static flatbuffers::FlatBufferBuilder builder;

  bool hasData = false;
  hprc::SensorsBuilder sensorBuilder(builder);
  uint32_t earliestTimestamp = UINT32_MAX;

  static uint32_t lastASM330DataAt = 0;
  const auto &asm330_desc = ctx->asm330.get_descriptor();
  if (asm330_desc.getLastUpdated() > lastASM330DataAt) {
    lastASM330DataAt = asm330_desc.getLastUpdated();
    earliestTimestamp = std::min(earliestTimestamp, lastASM330DataAt);

    hprc::ASM330Data asm330Data(asm330_desc.data.accel0,
                                asm330_desc.data.accel1,
                                asm330_desc.data.accel2, asm330_desc.data.gyr0,
                                asm330_desc.data.gyr1, asm330_desc.data.gyr2);
    sensorBuilder.add_asm330(&asm330Data);

    hasData = true;
  }

  static uint32_t lastLSM6DataAt = 0;
  const auto &lsm6_desc = ctx->lsm.get_descriptor();
  if (lsm6_desc.getLastUpdated() > lastLSM6DataAt) {
    lastLSM6DataAt = lsm6_desc.getLastUpdated();
    earliestTimestamp = std::min(earliestTimestamp, lastLSM6DataAt);

    hprc::LSM6Data lsm6Data(lsm6_desc.data.accel0, lsm6_desc.data.accel1,
                            lsm6_desc.data.accel2, lsm6_desc.data.gyr0,
                            lsm6_desc.data.gyr1, lsm6_desc.data.gyr2);
    sensorBuilder.add_lsm6(&lsm6Data);

    hasData = true;
  }

  static uint32_t lastBaroDataAt = 0;
  const auto &baro_desc = ctx->baro.get_descriptor();
  if (baro_desc.getLastUpdated() > lastBaroDataAt) {
    lastBaroDataAt = baro_desc.getLastUpdated();
    earliestTimestamp = std::min(earliestTimestamp, lastBaroDataAt);

    hprc::LPS22Data baroData(baro_desc.data.pressure, baro_desc.data.temp);
    sensorBuilder.add_lps22(&baroData);

    hasData = true;
  }

  static uint32_t lastMagDataAt = 0;
  const auto &mag_desc = ctx->mag.get_descriptor();
  if (mag_desc.getLastUpdated() > lastMagDataAt) {
    lastMagDataAt = mag_desc.getLastUpdated();
    earliestTimestamp = std::min(earliestTimestamp, lastMagDataAt);

    hprc::LIS2MDLData magData(mag_desc.data.mag0, mag_desc.data.mag1,
                              mag_desc.data.mag2);
    sensorBuilder.add_lis2mdl(&magData);

    hasData = true;
  }

  static uint32_t lastGpsDataAt = 0;
  const auto &gps_desc = ctx->gps.get_descriptor();
  if (gps_desc.getLastUpdated() > lastGpsDataAt) {
    lastGpsDataAt = gps_desc.getLastUpdated();
    earliestTimestamp = std::min(earliestTimestamp, lastGpsDataAt);

    hprc::LIV3FData gpsData(gps_desc.data.lat, gps_desc.data.lon,
                            gps_desc.data.alt, gps_desc.data.satellites,
                            gps_desc.data.epochTime);
    sensorBuilder.add_liv3f(&gpsData);

    hasData = true;
  }

  if (hasData) {
    auto packet = hprc::CreateSDPacket(builder, earliestTimestamp,
                                       sensorBuilder.Finish());

    builder.FinishSizePrefixed(packet);

    if (ctx->logFileBufferEnd + builder.GetSize() >= LOG_FILE_BUFFER_SIZE) {
      ctx->logFile.write(ctx->logFileBuffer, ctx->logFileBufferEnd);
      ctx->logFile.flush();
      ctx->logFileBufferEnd = 0;
    }
    memcpy(ctx->logFileBuffer + ctx->logFileBufferEnd,
           builder.GetBufferPointer(), builder.GetSize());
    ctx->logFileBufferEnd += builder.GetSize();

    builder.Clear();
  }
}
