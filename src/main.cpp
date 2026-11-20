#include "Context.h"
#include <Arduino.h>

#include <HardwareSerial.h>
#include <SPI.h>
#include <Wire.h>

#include "Packet_generated.h"

#include "boilerplate/Sensors/Impl/ASM330.h"
#include "boilerplate/Sensors/Impl/MAX10S.h"
#include "boilerplate/Sensors/SensorManager/SensorManager.h"

#include "logging.h"

#include "LoRaE22.h"
#include "RadioConfig.h"

#include "boilerplate/StateExtimator/simple_estimator.h"

SimpleEstimator simpleEstimator = SimpleEstimator();

SPIClass SENSORS_SPI(SENSORS_SPI_MOSI, SENSORS_SPI_MISO, SENSORS_SPI_SCK);
TwoWire GPS_I2C(GPS_I2C_SDA, GPS_I2C_SCL);
HardwareSerial GPS_SERIAL(GPS_SERIAL_RX, GPS_SERIAL_TX);
TwoWire CONNECTOR_I2C(CONNECTOR_I2C_SDA, CONNECTOR_I2C_SCL);
SPIClass CAMERA_SPI(CAMERA_MOSI, CAMERA_MISO, CAMERA_SCK);
HardwareSerial RADIO_SERIAL(RADIO_SERIAL_RX, RADIO_SERIAL_TX);

Context ctx{
    .asm330 = ASM330(&SENSORS_SPI, SENSORS_ASM_CS),
    .lsm = LSM6(&SENSORS_SPI, SENSORS_LSM_CS),
    .baro = LPS22(&SENSORS_SPI, SENSORS_LPS_CS),
    .mag = LIS2MDL(&SENSORS_SPI, SENSORS_LIS_CS),
    .gps = MAX10S(GPS_I2C),
    .radio = LoRaE22(&RADIO_SERIAL, RADIO_M0, RADIO_M1, RADIO_AUX, "KV0R"),
};

SensorManager mgr{
  millis, ctx.baro, ctx.asm330, ctx.lsm, ctx.mag, ctx.gps,
};

StateData data;
void *stateLocalData;

bool changeSerialPortConfig(RadioConfigTypes::SerialSpeeds baudRate,
                            RadioConfigTypes::ParityConfig parity) {
  // this is safe to call even when the port is not open.
  RADIO_SERIAL.end();

  uint32_t baud = 0;
  uint16_t parityConfig = 0;

  // the radio's baud rates don't follow any pattern over the entire range, so
  // ugly switch statement it is
  switch (baudRate) {
  case RadioConfigTypes::SerialSpeeds::BAUD_1200:
    baud = 1200;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_2400:
    baud = 2400;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_4800:
    baud = 4800;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_9600:
    baud = 9600;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_19200:
    baud = 19200;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_38400:
    baud = 38400;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_57600:
    baud = 57600;
    break;
  case RadioConfigTypes::SerialSpeeds::BAUD_115200:
    baud = 115200;
    break;
  };
  // this is just easier
  switch (parity) {
  case RadioConfigTypes::ParityConfig::Parity_8N1:
    parityConfig = SERIAL_8N1;
    break;
  case RadioConfigTypes::ParityConfig::Parity_8E1:
    parityConfig = SERIAL_8E1;
    break;
  case RadioConfigTypes::ParityConfig::Parity_8O1:
    parityConfig = SERIAL_8O1;
    break;
  };

  RADIO_SERIAL.begin(baud, parityConfig);

  return true;
}

void radioInit() {
  // build our config
  RadioConfig config;
  config.address = ADDRESS;
  config.networkId = NETWORKID;
  config.encryptionKey = ENCRYPTIONKEY;
  config.parityConfig = PARITYCONFIG;
  config.serialSpeed = SERIALSPEED;
  config.airDataRate = AIRDATARATE;
  config.packetSize = PACKETSIZE;
  config.worMode = WORMODE;
  config.worPeriod = WORPERIOD;
  config.relayMode = RELAYMODE;
  config.destination = DESTINATIONMODE;
  config.txPower = dBm33;
  config.ambientRSSIEnabled = AMBIENTRSSI;
  config.rssiReadingsEnabled = RSSIREADINGS;
  config.listenBeforeTxEnable = LISTENBEFORETX;
  ctx.radio.setConfig(config);
  ctx.radio.setFrequency(FREQUENCY);

  ctx.radio.changeSerialPortCallback(changeSerialPortConfig);
  ctx.radio.setTimeout(2000);

  int8_t code = ctx.radio.init(3);
  ctx.radio.setMode(RadioMode::Normal);

  Log.infoln("radio init done, code: %d", code);
}

void radioLoop() {
  static flatbuffers::FlatBufferBuilder builder;
  static uint8_t rxBuff[1024];
  static uint16_t lastCmdNum = 0;
  static uint32_t lastRadioSendTime = 0;
  static uint32_t loopCount = 0;

  ctx.radio.update();

  if (millis() - lastRadioSendTime >= 200) {
    lastRadioSendTime = millis();
    const auto &asm330_desc = ctx.asm330.get_descriptor();
    const auto &lsm6_desc = ctx.lsm.get_descriptor();
    const auto &baro_desc = ctx.baro.get_descriptor();
    const auto &mag_desc = ctx.mag.get_descriptor();
    const auto &gps_desc = ctx.gps.get_descriptor();

    builder.Clear();
    hprc::SensorsBuilder sensorBuilder(builder);

    static uint32_t lastASM330DataAt = 0;
    if (asm330_desc.getLastUpdated() > lastASM330DataAt) {
      lastASM330DataAt = asm330_desc.getLastUpdated();

      hprc::ASM330Data asm330Data(asm330_desc.data.accel0,
                                  asm330_desc.data.accel1,
                                  asm330_desc.data.accel2, asm330_desc.data.gyr0,
                                  asm330_desc.data.gyr1, asm330_desc.data.gyr2);
      sensorBuilder.add_asm330(&asm330Data);
    }

    static uint32_t lastLSM6DataAt = 0;
    if (lsm6_desc.getLastUpdated() > lastLSM6DataAt) {
      lastLSM6DataAt = lsm6_desc.getLastUpdated();

      hprc::LSM6Data lsm6Data(lsm6_desc.data.accel0, lsm6_desc.data.accel1,
                              lsm6_desc.data.accel2, lsm6_desc.data.gyr0,
                              lsm6_desc.data.gyr1, lsm6_desc.data.gyr2);
      sensorBuilder.add_lsm6(&lsm6Data);
    }

    static uint32_t lastBaroDataAt = 0;
    if (baro_desc.getLastUpdated() > lastBaroDataAt) {
      lastBaroDataAt = baro_desc.getLastUpdated();

      hprc::LPS22Data baroData(baro_desc.data.pressure, baro_desc.data.temp);
      sensorBuilder.add_lps22(&baroData);
    }

    static uint32_t lastMagDataAt = 0;
    if (mag_desc.getLastUpdated() > lastMagDataAt) {
      lastMagDataAt = mag_desc.getLastUpdated();

      hprc::LIS2MDLData magData(mag_desc.data.mag0, mag_desc.data.mag1,
                                mag_desc.data.mag2);
      sensorBuilder.add_lis2mdl(&magData);
    }

    static uint32_t lastGpsDataAt = 0;
    if (gps_desc.getLastUpdated() > lastGpsDataAt) {
      lastGpsDataAt = gps_desc.getLastUpdated();

      hprc::MAX10SData gpsData(gps_desc.data.lat, gps_desc.data.lon,
                              gps_desc.data.altMSL, gps_desc.data.satellites,
                              gps_desc.data.epochTime);
      sensorBuilder.add_max10s(&gpsData);
    }

    hprc::Shared sharedData;
    sharedData.mutate_time_from_boot(millis());
    sharedData.mutate_last_command_received(lastCmdNum);
    sharedData.mutate_sd_file_no(ctx.logFileIdx);
    sharedData.mutate_loop_count(loopCount);

    auto packetInner = hprc::CreateRocket30KTelemetryPacket(builder, &sharedData, stateToTelemState(ctx.currentState), sensorBuilder.Finish());
    auto packet = hprc::CreatePacket(builder, hprc::PacketUnion_Rocket30KTelemetryPacket, packetInner.Union());

    builder.Finish(packet);
    ctx.radio.sendMessage(builder.GetBufferPointer(), builder.GetSize());
    loopCount++;
  }
}

void sensorsSetup() {
  Log.infoln("Starting MARS board initialization...");
  SENSORS_SPI.begin();
  GPS_I2C.begin();

  mgr.sensorInit();

  Log.infoln("\n=== Sensor Initialization Summary ===");
  Log.infoln("Total sensors: %d", mgr.count());
}

void sensorLoop() {
  static unsigned long last_print = 0;
  static int loop_count = 0;

  // Update all sensors through manager
  mgr.loop();

  if (millis() - last_print > 200) {
    digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
    last_print = millis();
    loop_count++;

    Log.infoln("=== Loop %d ===", loop_count);
    Serial.println("upload test build " __DATE__ " " __TIME__);
    if (loop_count % 25 == 0) {
      extern void printBootTrace(Print &);
      printBootTrace(Serial);
    }

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
      // ArduinoLog's %F only prints 2 decimals
      char latStr[16], lonStr[16];
      dtostrf(gps_desc.data.lat, 0, 7, latStr);
      dtostrf(gps_desc.data.lon, 0, 7, lonStr);
      Log.infoln("MAX10S - Lat, Lon, Alt: %s, %s, %F | Satellites - %d",
                 latStr, lonStr, gps_desc.data.altMSL,
                 gps_desc.data.satellites);
      has_data = true;
    } else {
      Log.warningln("MAX10S: No data (timestamp = 0)");
    }

    Log.infoln("======================\n");
  }
}

void setup() {
  pinMode(MOSFET_GATE, OUTPUT);
  pinMode(ADC_INP4, INPUT);
  digitalWrite(MOSFET_GATE, HIGH);

  ctx.currentState = PRELAUNCH;
  data = {};

  initStateMap();

  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  digitalWrite(LED_RED, HIGH);

  Serial.begin(115200);
  // radioInit();

  #ifdef DEBUG
    // check that delays are right
    const int ONE_MINUTE_MS = 1000*60*1;
    const uint32_t start = millis();

    // if no serial, wait
    // if no serial after set wait fall through and exec
    while (!Serial && (millis() - start) < ONE_MINUTE_MS) {
      delay(10);
      // Serial has a hidden 10ms delay inside, might cause issues?
      // i think this time check tho should be ok
    }

    delay(200);
  #endif

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
  (*initFuncs[ctx.currentState])(&data);

  const auto &lsm6_desc = ctx.lsm.get_descriptor();
  const auto &baro_desc = ctx.baro.get_descriptor();

  // Initialize 
  BLA::Matrix<3, 1> accelVec = {lsm6_desc.data.accel0, lsm6_desc.data.accel1, lsm6_desc.data.accel2};
  BLA::Matrix<3, 1> gyroVec = {lsm6_desc.data.gyr0, lsm6_desc.data.gyr1, lsm6_desc.data.gyr2};
  simpleEstimator.initialize(accelVec, gyroVec, baro_desc.data.pressure);

  

  digitalWrite(LED_GREEN, HIGH);
  Log.infoln("=== Starting main loop ===\n");
}

void estimatorLoop(Context *ctx) {
  const auto &lsm6_desc = ctx->lsm.get_descriptor();
  BLA::Matrix<3, 1> gyroVec = {lsm6_desc.data.gyr0, lsm6_desc.data.gyr1, lsm6_desc.data.gyr2};

  if(ctx->currentState == PRELAUNCH) {
    simpleEstimator.initialize_loop(gyroVec);
    ctx->lastEstimatorUpdateTime = 0;
  } else {
    simpleEstimator.flight_loop(gyroVec, millis() - ctx->lastEstimatorUpdateTime);
    ctx->lastEstimatorUpdateTime = millis();
  }
}

void loop() {

  updateStateData(&data);
  StateID newState = (*loopFuncs[ctx.currentState])(&data, &ctx, stateLocalData);

  if (ctx.currentState != newState) {
    initStateData(&data);

    if (stateLocalData != nullptr) {
      free(stateLocalData);
    }
    stateLocalData = (*initFuncs[newState])(&data);
    ctx.currentState = newState;
    ctx.debugLogFile.printf("to: %d @ %d\n", newState, millis());
  }

  sensorLoop();

  estimatorLoop(&ctx);

  // radioLoop();
  loggingLoop(&ctx);
}
