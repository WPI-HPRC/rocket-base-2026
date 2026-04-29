#include <Arduino.h>
#include "Context.h"
#include "boilerplate/Sensors/Impl/LIV3F.h"
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

#include <Wire.h>
#include <HardwareSerial.h>
#include <SPI.h>

SPIClass SENSORS_SPI(SENSORS_SPI_MOSI, SENSORS_SPI_MISO, SENSORS_SPI_SCK);
TwoWire GPS_I2C(GPS_I2C_SDA, GPS_I2C_SCL);
HardwareSerial GPS_SERIAL(GPS_SERIAL_RX, GPS_SERIAL_TX);
TwoWire CONNECTOR_I2C(CONNECTOR_I2C_SDA, CONNECTOR_I2C_SCL);
SPIClass CAMERA_SPI(CAMERA_MOSI, CAMERA_MISO, CAMERA_SCK);
HardwareSerial RADIO_SERIAL(RADIO_SERIAL_RX, RADIO_SERIAL_TX);

Context ctx {
    .asm330 = ASM330(&SENSORS_SPI, SENSORS_ASM_CS),
    .lsm = LSM6(&SENSORS_SPI, SENSORS_LSM_CS),
    .baro = LPS22(&SENSORS_SPI, SENSORS_LPS_CS),
    .mag = LIS2MDL(&SENSORS_SPI, SENSORS_LIS_CS),
    .gps = LIV3F(GPS_SERIAL)
};


SensorManager mgr {
    millis,
    ctx.asm330,
    ctx.lsm,
    ctx.baro,
    ctx.mag,
    ctx.gps,
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
    Serial.println("Starting MARS board initialization...");
    SENSORS_SPI.begin();

    mgr.sensorInit();

    Serial.println("\n=== Sensor Initialization Summary ===");
    Serial.print("Total sensors: ");
    Serial.println(mgr.count());
    Serial.println("=== Starting main loop ===\n");
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
    if (millis() - last_print > 200)
    {
        digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
        last_print = millis();
        loop_count++;

        Serial.print("\n=== Loop ");
        Serial.print(loop_count);
        Serial.println(" ===");

        // DIRECT ACCESS to sensor data - this is guaranteed to work
        const auto &asm330_desc = ctx.asm330.get_descriptor();
        const auto &lsm6_desc = ctx.lsm.get_descriptor();
        const auto &baro_desc = ctx.baro.get_descriptor();
        const auto &mag_desc = ctx.mag.get_descriptor();
        const auto &gps_desc = ctx.gps.get_descriptor();
        // const auto &curr_desc = ctx.curr.get_descriptor();

        bool has_data = false;
        // Print LSM6 data
        if (lsm6_desc.getLastUpdated() > 0)
        {
            Serial.print("LSM6DSO - Accel: ");
            Serial.print(lsm6_desc.data.accel0, 4);
            Serial.print(", ");
            Serial.print(lsm6_desc.data.accel1, 4);
            Serial.print(", ");
            Serial.print(lsm6_desc.data.accel2, 4);
            Serial.print(" | Gyro: ");
            Serial.print(lsm6_desc.data.gyr0, 4);
            Serial.print(", ");
            Serial.print(lsm6_desc.data.gyr1, 4);
            Serial.print(", ");
            Serial.print(lsm6_desc.data.gyr2, 4);
            Serial.println();
            has_data = true;
        }
        else
        {
            Serial.println("LSM6DSO: No data (timestamp = 0)");
        }

        // Print ASM330 data
        if (asm330_desc.getLastUpdated() > 0)
        {
            Serial.print("ASM330 - Accel: ");
            Serial.print(asm330_desc.data.accel0, 4);
            Serial.print(", ");
            Serial.print(asm330_desc.data.accel1, 4);
            Serial.print(", ");
            Serial.print(asm330_desc.data.accel2, 4);
            Serial.print(" | Gyro: ");
            Serial.print(asm330_desc.data.gyr0, 4);
            Serial.print(", ");
            Serial.print(asm330_desc.data.gyr1, 4);
            Serial.print(", ");
            Serial.print(asm330_desc.data.gyr2, 4);
            Serial.println();
            has_data = true;
        }
        else
        {
            Serial.println("LSM6DSO: No data (timestamp = 0)");
        }
        // Print LPS22 data
        if (baro_desc.getLastUpdated() > 0)
        {
            Serial.print("LPS22 - Pressure: ");
            Serial.print(baro_desc.data.pressure, 4);
            Serial.print(" hPa, Temp: ");
            Serial.print(baro_desc.data.temp, 4);
            Serial.println(" C");
            has_data = true;
        }
        else
        {
            Serial.println("LPS22: No data (timestamp = 0)");
        }

        if(mag_desc.getLastUpdated()  > 0)
        {
            Serial.print("LIS2MDL - Mag: ");
            Serial.print(mag_desc.data.mag0, 4);
            Serial.print(", ");
            Serial.print(mag_desc.data.mag1, 4);
            Serial.print(", ");
            Serial.print(mag_desc.data.mag2, 4);
            Serial.println();
            has_data = true;
        }
        else
        {
            Serial.println("ICM20948: No data (timestamp = 0)");
        }

        if(gps_desc.getLastUpdated() > 0)
        {
            Serial.print("MAX10S - Lat, Lon, Alt: ");
            Serial.print(gps_desc.data.lat, 4);
            Serial.print(", ");
            Serial.print(gps_desc.data.lon, 4);
            Serial.print(", ");
            Serial.print(gps_desc.data.alt, 4);
            Serial.print(", ");
            Serial.print("| Satellites - ");
            Serial.print(gps_desc.data.satellites);
            Serial.println();
            has_data = true;
        }
        else
        {
            Serial.println("LIV3F: No data (timestamp = 0)");
        }

        Serial.println("======================");
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
    while(!Serial) {
        delay(10);
    }

    delay(200);
    
    // NOTE: Run initialization on the first state
    initStateData(&data);
    (*initFuncs[currentState])(&data);
    sensorsSetup();
    ctx.ekfLooping = false;
    ctx.sdInitialized = initializeLogging(&ctx);

    // ctx.airBrakes.attach(SERVO_PIN);
    // ctx.airBrakes.writeMicroseconds(SERVO_MIN);

    //ctx.estimator = SplitStateEstimator();
    
    BLA::Matrix<3, 1> ecef = {0, 0, 0};
    // ctx.estimator.init(ecef, millis());

    digitalWrite(LED_GREEN, HIGH);
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

    bool inAir = currentState == BOOST ||
                 currentState == COAST ||
                 currentState == DROGUE_DESCENT ||
                 currentState == MAIN_DESCENT;

    // Accel and gyro becuase they are on the same sensor
    if(asm330_desc.getLastUpdated() > last_accel_time) {
        BLA::Matrix<3, 1> gyro = {asm330_desc.data.gyr0, asm330_desc.data.gyr1, asm330_desc.data.gyr2};
        ctx->estimator.fastGyroProp(gyro, now);

        BLA::Matrix<3, 1> accel = {asm330_desc.data.accel0, asm330_desc.data.accel1, asm330_desc.data.accel2};
        ctx->estimator.fastAccelProp(accel, now);
    }

    if(inAir) {
        if(baro_desc.getLastUpdated() > last_baro_time ||
           mag_desc.getLastUpdated() > last_mag_time ||
           gps_desc.getLastUpdated() > last_gps_time) {
            ctx->estimator.PVekfPredict(now); 
        }
    } else {
        if(asm330_desc.getLastUpdated() > last_accel_time ||
           baro_desc.getLastUpdated() > last_baro_time ||
           mag_desc.getLastUpdated() > last_mag_time ||
           gps_desc.getLastUpdated() > last_gps_time) {
            ctx->estimator.PVekfPredict(now); 
        }
    }

    if(asm330_desc.getLastUpdated() > last_accel_time) {
        last_accel_time = asm330_desc.getLastUpdated();
        BLA::Matrix<3, 1> accel = {asm330_desc.data.accel0, asm330_desc.data.accel1, asm330_desc.data.accel2};
        ctx->estimator.runAccelUpdate(accel, now);
    }

    if (baro_desc.getLastUpdated() > last_baro_time)
    {
        last_baro_time = baro_desc.getLastUpdated();
        BLA::Matrix<1, 1> baro = {baro_desc.data.pressure};
        ctx->estimator.runBaroUpdate(baro, now);
        ctx->estimator.set_curr_temp(baro_desc.data.temp);
    }

    if (mag_desc.getLastUpdated() > last_mag_time)
    {
        last_mag_time = mag_desc.getLastUpdated();
        BLA::Matrix<3, 1> mag = {mag_desc.data.mag0, mag_desc.data.mag1, mag_desc.data.mag2};
        ctx->estimator.runMagUpdate(mag, now);
    }

    // if (gps_desc.getLastUpdated() > last_gps_time)
    // {
    //     last_gps_time = gps_desc.getLastUpdated();
    //     BLA::Matrix<3, 1> gpsPos = {gps_desc.data.ecefX, gps_desc.data.ecefY, gps_desc.data.ecefZ};
    //     BLA::Matrix<3, 1> gpsVel = {gps_desc.data.velN, gps_desc.data.velE, gps_desc.data.velD};
    //     ctx->estimator.runGPSUpdate(gpsPos, gpsVel, false, now);
    // }
}

void loop() {

    updateStateData(&data);
    StateID newState = (*loopFuncs[currentState])(&data, &ctx);

    if(currentState != newState) {
        initStateData(&data);
        (*initFuncs[newState])(&data);
        currentState = newState;
        ctx.errorLogFile.printf("%d %d\n", newState, millis());
    }

    sensorLoop();

    if(false && ctx.ekfLooping) {
        ekfLoop(&ctx);
    }

    loggingLoop(&ctx);
}
