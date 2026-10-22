#include "usb_dump.h"

#include <Arduino.h>

#ifdef USB_SENSOR_DUMP

static void usbDumpHeaders() {
  Serial.println("#HDR,ASM330,time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z");
  Serial.println("#HDR,LSM6,time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z");
  Serial.println("#HDR,LPS22,time_ms,pressure_hpa,temp_c");
  Serial.println("#HDR,LIS2MDL,time_ms,mag_x,mag_y,mag_z");
  Serial.println("#HDR,LIV3F,time_ms,lat,lon,alt,satellites,epoch_time");
}

// Starts a data row: "$<SENSOR>,<time_ms>"
static void usbDumpBegin(const char *sensor, uint32_t time_ms) {
  Serial.print('$');
  Serial.print(sensor);
  Serial.print(',');
  Serial.print(time_ms);
}

// newlib-nano printf has no float support, so print floats manually
static void usbDumpField(float value, int digits = 6) {
  Serial.print(',');
  Serial.print(value, digits);
}

static void usbDumpField(uint32_t value) {
  Serial.print(',');
  Serial.print(value);
}

// Prints one CSV row for every sensor that has new data since the last call
void usbDumpLoop(Context *ctx) {
  static uint32_t lastHeaderTime = 0;
  static uint32_t lastASM330 = 0, lastLSM6 = 0, lastBaro = 0, lastMag = 0,
                  lastGps = 0;

  if (lastHeaderTime == 0 ||
      millis() - lastHeaderTime >= USB_DUMP_HEADER_INTERVAL_MS) {
    lastHeaderTime = millis();
    usbDumpHeaders();
  }

  const auto &asm330_desc = ctx->asm330.get_descriptor();
  const auto &lsm6_desc = ctx->lsm.get_descriptor();
  const auto &baro_desc = ctx->baro.get_descriptor();
  const auto &mag_desc = ctx->mag.get_descriptor();
  const auto &gps_desc = ctx->gps.get_descriptor();

  if (asm330_desc.getLastUpdated() > lastASM330) {
    lastASM330 = asm330_desc.getLastUpdated();
    usbDumpBegin("ASM330", lastASM330);
    usbDumpField(asm330_desc.data.accel0);
    usbDumpField(asm330_desc.data.accel1);
    usbDumpField(asm330_desc.data.accel2);
    usbDumpField(asm330_desc.data.gyr0);
    usbDumpField(asm330_desc.data.gyr1);
    usbDumpField(asm330_desc.data.gyr2);
    Serial.println();
  }

  if (lsm6_desc.getLastUpdated() > lastLSM6) {
    lastLSM6 = lsm6_desc.getLastUpdated();
    usbDumpBegin("LSM6", lastLSM6);
    usbDumpField(lsm6_desc.data.accel0);
    usbDumpField(lsm6_desc.data.accel1);
    usbDumpField(lsm6_desc.data.accel2);
    usbDumpField(lsm6_desc.data.gyr0);
    usbDumpField(lsm6_desc.data.gyr1);
    usbDumpField(lsm6_desc.data.gyr2);
    Serial.println();
  }

  if (baro_desc.getLastUpdated() > lastBaro) {
    lastBaro = baro_desc.getLastUpdated();
    usbDumpBegin("LPS22", lastBaro);
    usbDumpField(baro_desc.data.pressure);
    usbDumpField(baro_desc.data.temp);
    Serial.println();
  }

  if (mag_desc.getLastUpdated() > lastMag) {
    lastMag = mag_desc.getLastUpdated();
    usbDumpBegin("LIS2MDL", lastMag);
    usbDumpField(mag_desc.data.mag0);
    usbDumpField(mag_desc.data.mag1);
    usbDumpField(mag_desc.data.mag2);
    Serial.println();
  }

  if (gps_desc.getLastUpdated() > lastGps) {
    lastGps = gps_desc.getLastUpdated();
    usbDumpBegin("LIV3F", lastGps);
    usbDumpField(gps_desc.data.lat);
    usbDumpField(gps_desc.data.lon);
    usbDumpField(gps_desc.data.alt);
    usbDumpField((uint32_t)gps_desc.data.satellites);
    usbDumpField(gps_desc.data.epochTime);
    Serial.println();
  }
}

#else

void usbDumpLoop(Context *ctx) {}

#endif
