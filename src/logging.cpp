#include "Context.h"
#include "SdData_generated.h"
#include "Sensors_generated.h"
#include "logging.h"
#include <stdint.h>

#if SPI_DRIVER_SELECT != 2
#error "Build with -DSPI_DRIVER_SELECT=2 to enable SdFat's soft SPI driver"
#endif

// The SD slot is wired for 4-bit SDMMC. In SPI mode DAT3 is CS, CMD is MOSI and
// DAT0 is MISO. No hardware SPI peripheral maps to those pins, so the bus is
// bit-banged.
static SoftSpiDriver<PIN_SD_DAT0, PIN_SD_CMD, PIN_SD_SCK> sdSpi;
static SdFs sd;

static constexpr char PREV_LOG_IDX_PATH[] = LOG_FOLDER_NAME "/.prev_log_idx";
static constexpr int MAX_LOG_FILE_IDX = 100;
static constexpr oflag_t CREATE_LOG_FLAGS = O_RDWR | O_CREAT | O_TRUNC;

// Reads the index used by the previous boot and stores the next one.
static int nextLogFileIdx() {
  FsFile idxFile;
  if (!idxFile.open(PREV_LOG_IDX_PATH, O_RDWR | O_CREAT)) {
    Serial.printf("Failed to open `%s`\n", PREV_LOG_IDX_PATH);
    return 0;
  }

  int fileIdx = idxFile.available() ? idxFile.parseInt() + 1 : 0;
  if (fileIdx > MAX_LOG_FILE_IDX) {
    fileIdx = 0;
  }

  idxFile.truncate(0);
  idxFile.print(fileIdx);
  idxFile.close();
  return fileIdx;
}

static bool openLogFile(FsFile &file, const char *path) {
  if (file.open(path, CREATE_LOG_FLAGS)) {
    return true;
  }
  Serial.printf("Failed to open `%s`\n", path);
  return false;
}

bool initializeLogging(Context *ctx) {
  Serial.print("Initializing SD... ");

  // Soft SPI runs at one fixed speed, so the clock argument is ignored.
  if (!sd.begin(SdSpiConfig(PIN_SD_DAT3, DEDICATED_SPI, SD_SCK_MHZ(0), &sdSpi))) {
    Serial.println("FAILED");
    sd.printSdError(&Serial);
    return false;
  }
  Serial.println("OK");

  if (!sd.exists(LOG_FOLDER_NAME)) {
    sd.mkdir(LOG_FOLDER_NAME);
  }

  int fileIdx = nextLogFileIdx();
  char filename[64];
  char debugFilename[64];
  char ekfLogFilename[64];
  sprintf(filename, LOG_FOLDER_NAME "/flightData%d.fb.bin", fileIdx);
  sprintf(debugFilename, LOG_FOLDER_NAME "/debugLog%d.txt", fileIdx);
  sprintf(ekfLogFilename, LOG_FOLDER_NAME "/ekflog%d.bin", fileIdx);

  Serial.printf("Main log file `%s`\n", filename);
  bool opened = openLogFile(ctx->logFile, filename);
  opened &= openLogFile(ctx->debugLogFile, debugFilename);
  opened &= openLogFile(ctx->ekfLogFile, ekfLogFilename);

  ctx->logFileIdx = fileIdx;
  return opened;
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

    hprc::MAX10SData gpsData(gps_desc.data.lat, gps_desc.data.lon,
                            gps_desc.data.altMSL, gps_desc.data.satellites,
                            gps_desc.data.epochTime);
    sensorBuilder.add_max10s(&gpsData);

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
