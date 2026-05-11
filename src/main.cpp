#include "Context.h"
#include "boilerplate/Sensors/Impl/LIV3F.h"
#include <Arduino.h>
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
#include "boilerplate/Sensors/SensorManager/SensorManager.h"

#include "logging.h"

#include <HardwareSerial.h>
#include <SPI.h>
#include <Wire.h>

SPIClass SENSORS_SPI(SENSORS_SPI_MOSI, SENSORS_SPI_MISO, SENSORS_SPI_SCK);
TwoWire GPS_I2C(GPS_I2C_SDA, GPS_I2C_SCL);
HardwareSerial GPS_SERIAL(GPS_SERIAL_RX, GPS_SERIAL_TX);
TwoWire CONNECTOR_I2C(CONNECTOR_I2C_SDA, CONNECTOR_I2C_SCL);
SPIClass CAMERA_SPI(CAMERA_MOSI, CAMERA_MISO, CAMERA_SCK);
HardwareSerial RADIO_SERIAL(RADIO_SERIAL_RX, RADIO_SERIAL_TX);

Context ctx{.asm330 = ASM330(&SENSORS_SPI, SENSORS_ASM_CS),
            .lsm = LSM6(&SENSORS_SPI, SENSORS_LSM_CS),
            .baro = LPS22(&SENSORS_SPI, SENSORS_LPS_CS),
            .mag = LIS2MDL(&SENSORS_SPI, SENSORS_LIS_CS),
            .gps = LIV3F(GPS_SERIAL)};

SensorManager mgr{
    millis, ctx.asm330, ctx.lsm, ctx.baro, ctx.mag, ctx.gps,
};

StateID currentState;
StateData data;

void initStateData(StateData *data) {
  data->startTime = millis();
  data->currentTime = 0;
  data->deltaTime = 0;
  data->lastLoopTime = 0;
  data->loopCount = 0;
};

void updateStateData(StateData *data) {
  long long now = millis();
  data->currentTime = now - data->startTime;
  data->deltaTime = now - data->lastLoopTime;
  data->lastLoopTime = now;
  data->loopCount++;
}

void sensorsSetup() {
  Log.infoln("Starting MARS board initialization...");
  SENSORS_SPI.begin();

  mgr.sensorInit();

  Log.infoln("\n=== Sensor Initialization Summary ===");
  Log.infoln("Total sensors: %d", mgr.count());
}

void sensorLoop() {
  static unsigned long last_print = 0;
  static int loop_count = 0;

  // Update all sensors through manager
  mgr.loop();

  /*
  if (currentState >= PRELAUNCH) {
      return;
  }
  */

  // manager is not being used here to get data
  if (millis() - last_print > 200) {
    digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
    last_print = millis();
    loop_count++;

    Log.infoln("=== Loop %d ===", loop_count);

    // DIRECT ACCESS to sensor data - this is guaranteed to work
    const auto &asm330_desc = ctx.asm330.get_descriptor();
    const auto &lsm6_desc = ctx.lsm.get_descriptor();
    const auto &baro_desc = ctx.baro.get_descriptor();
    const auto &mag_desc = ctx.mag.get_descriptor();
    const auto &gps_desc = ctx.gps.get_descriptor();

    bool has_data = false;
    // Print LSM6 data
    if (lsm6_desc.getLastUpdated() > 0) {
      Log.infoln("LSM6DSO - Accel: %F, %F, %F | Gyro: %F, %F, %F",
                 lsm6_desc.data.accel0, lsm6_desc.data.accel1,
                 lsm6_desc.data.accel2, lsm6_desc.data.gyr0,
                 lsm6_desc.data.gyr1, lsm6_desc.data.gyr2);
      has_data = true;
    } else {
      Log.warningln("LSM6DSO: No data (timestamp = 0)");
    }

    // Print ASM330 data
    if (asm330_desc.getLastUpdated() > 0) {
      Log.infoln("ASM330- Accel: %F, %F, %F | Gyro: %F, %F, %F",
                 asm330_desc.data.accel0, asm330_desc.data.accel1,
                 asm330_desc.data.accel2, asm330_desc.data.gyr0,
                 asm330_desc.data.gyr1, asm330_desc.data.gyr2);
      has_data = true;
    } else {
      Log.warningln("ASM330: No data (timestamp = 0)");
    }
    // Print LPS22 data
    if (baro_desc.getLastUpdated() > 0) {
      Log.infoln("LPS22 - Pressure: %F hPa, Temp: %F C",
                 baro_desc.data.pressure, baro_desc.data.temp);
      has_data = true;
    } else {
      Log.warningln("LPS22: No data (timestamp = 0)");
    }

    if (mag_desc.getLastUpdated() > 0) {
      Log.infoln("LIS2MDL - Mag: %F, %F, %F", mag_desc.data.mag0,
                 mag_desc.data.mag1, mag_desc.data.mag2);
      has_data = true;
    } else {
      Log.warningln("ICM20948: No data (timestamp = 0)");
    }

    if (gps_desc.getLastUpdated() > 0) {
      Log.infoln("LIV3F - Lat, Lon, Alt: %F, %F, %F | Satellites - %d",
                 gps_desc.data.lat, gps_desc.data.lon, gps_desc.data.alt,
                 gps_desc.data.satellites);
      has_data = true;
    } else {
      Log.warningln("LIV3F: No data (timestamp = 0)");
    }

    Log.infoln("======================\n");
  }
}

void setup() {
  currentState = PRELAUNCH;
  data = {};

  initStateMap();

  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  digitalWrite(LED_RED, HIGH);

  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  delay(200);

  ctx.ekfLooping = false;
  ctx.sdInitialized = initializeLogging(&ctx);

#if DEBUG_MODE == 0
  Log.begin(LOG_LEVEL_INFO, &ctx.debugLogFile);
#elif DEBUG_MODE == 1
  Log.begin(LOG_LEVEL_INFO, &Serial);
#else
  Log.begin(LOG_LEVEL_TRACE, &Serial);
#endif
  Log.setPrefix([](Print *p, int level) { p->printf("[ %d ] ", millis()); });

  sensorsSetup();

  // NOTE: Run initialization on the first state
  initStateData(&data);
  (*initFuncs[currentState])(&data);

  // ctx.estimator = SplitStateEstimator();

  BLA::Matrix<3, 1> ecef = {0, 0, 0};
  // ctx.estimator.init(ecef, millis());

  digitalWrite(LED_GREEN, HIGH);
  Log.infoln("=== Starting main loop ===\n");
}

void ekfLoop(Context *ctx) {
  static uint32_t last_accel_time = 0;
  static uint32_t last_mag_time = 0;
  static uint32_t last_gps_time = 0;
  static uint32_t last_baro_time = 0;

  uint32_t now = millis();

  const auto &asm330_desc = ctx->asm330.get_descriptor();
  const auto &baro_desc = ctx->baro.get_descriptor();
  const auto &mag_desc = ctx->mag.get_descriptor();
  const auto &gps_desc = ctx->gps.get_descriptor();

  bool inAir = currentState == BOOST || currentState == COAST ||
               currentState == DROGUE_DESCENT || currentState == MAIN_DESCENT;

  // Accel and gyro becuase they are on the same sensor
  if (asm330_desc.getLastUpdated() > last_accel_time) {
    BLA::Matrix<3, 1> gyro = {asm330_desc.data.gyr0, asm330_desc.data.gyr1,
                              asm330_desc.data.gyr2};
    ctx->estimator.fastGyroProp(gyro, now);

    BLA::Matrix<3, 1> accel = {asm330_desc.data.accel0, asm330_desc.data.accel1,
                               asm330_desc.data.accel2};
    ctx->estimator.fastAccelProp(accel, now);
  }

  if (inAir) {
    if (baro_desc.getLastUpdated() > last_baro_time ||
        mag_desc.getLastUpdated() > last_mag_time ||
        gps_desc.getLastUpdated() > last_gps_time) {
      ctx->estimator.PVekfPredict(now);
    }
  } else {
    if (asm330_desc.getLastUpdated() > last_accel_time ||
        baro_desc.getLastUpdated() > last_baro_time ||
        mag_desc.getLastUpdated() > last_mag_time ||
        gps_desc.getLastUpdated() > last_gps_time) {
      ctx->estimator.PVekfPredict(now);
    }
  }

  if (asm330_desc.getLastUpdated() > last_accel_time) {
    last_accel_time = asm330_desc.getLastUpdated();
    BLA::Matrix<3, 1> accel = {asm330_desc.data.accel0, asm330_desc.data.accel1,
                               asm330_desc.data.accel2};
    ctx->estimator.runAccelUpdate(accel, now);
  }

  if (baro_desc.getLastUpdated() > last_baro_time) {
    last_baro_time = baro_desc.getLastUpdated();
    BLA::Matrix<1, 1> baro = {baro_desc.data.pressure};
    ctx->estimator.runBaroUpdate(baro, now);
    ctx->estimator.set_curr_temp(baro_desc.data.temp);
  }

  if (mag_desc.getLastUpdated() > last_mag_time) {
    last_mag_time = mag_desc.getLastUpdated();
    BLA::Matrix<3, 1> mag = {mag_desc.data.mag0, mag_desc.data.mag1,
                             mag_desc.data.mag2};
    ctx->estimator.runMagUpdate(mag, now);
  }

  // if (gps_desc.getLastUpdated() > last_gps_time)
  // {
  //     last_gps_time = gps_desc.getLastUpdated();
  //     BLA::Matrix<3, 1> gpsPos = {gps_desc.data.ecefX, gps_desc.data.ecefY,
  //     gps_desc.data.ecefZ}; BLA::Matrix<3, 1> gpsVel = {gps_desc.data.velN,
  //     gps_desc.data.velE, gps_desc.data.velD};
  //     ctx->estimator.runGPSUpdate(gpsPos, gpsVel, false, now);
  // }
}

void loop() {

  updateStateData(&data);
  StateID newState = (*loopFuncs[currentState])(&data, &ctx);

  if (currentState != newState) {
    initStateData(&data);
    (*initFuncs[newState])(&data);
    currentState = newState;
    ctx.debugLogFile.printf("to: %d @ %d\n", newState, millis());
  }

  sensorLoop();

  if (false && ctx.ekfLooping) {
    ekfLoop(&ctx);
  }

  loggingLoop(&ctx);
}
