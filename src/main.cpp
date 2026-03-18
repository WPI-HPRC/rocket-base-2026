#include <Arduino.h>
#include "Context.h"
#include "State.h"
#include "States.h"

#include "boilerplate/Sensors/SensorManager/SensorManager.h"
#include "config.h"

#include "logging.h"

Context ctx;

uint32_t millisSource() { return millis(); }

SensorManager mgr {
    millisSource,
    ctx.accel,
    ctx.baro,
    ctx.mag,
    ctx.gps,
    ctx.curr,
};


StateID currentState;
StateData data;

StateInitFunc initFuncs[NUM_STATES] = {};
StateLoopFunc loopFuncs[NUM_STATES] = {};

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
    Serial.begin(115200);
    while(!Serial) {
        delay(10);
    }

    Serial.println("Starting MARS board initialization...");

    Wire.setSDA(SENSOR_SDA);
    Wire.setSCL(SENSOR_SCL);
    Wire.begin();

    Serial.print("I2C initialized on SDA: ");
    Serial.print(SENSOR_SDA);
    Serial.print(", SCL: ");
    Serial.println(SENSOR_SCL);

    mgr.sensorInit();

    Wire.setClock(400000);

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
        last_print = millis();
        loop_count++;

        Serial.print("\n=== Loop ");
        Serial.print(loop_count);
        Serial.println(" ===");

        // DIRECT ACCESS to sensor data - this is guaranteed to work
        const auto &accel_desc = ctx.accel.get_descriptor();
        const auto &baro_desc = ctx.baro.get_descriptor();
        const auto &mag_desc = ctx.mag.get_descriptor();
        const auto &gps_desc = ctx.gps.get_descriptor();
        const auto &curr_desc = ctx.curr.get_descriptor();

        bool has_data = false;
        // Print ASM330 data
        /*
        if (accel_desc.getLastUpdated() > 0)
        {
            Serial.print("ASM330 - Accel: ");
            Serial.print(accel_desc.data.accel0, 4);
            Serial.print(", ");
            Serial.print(accel_desc.data.accel1, 4);
            Serial.print(", ");
            Serial.print(accel_desc.data.accel2, 4);
            Serial.print(" | Gyro: ");
            Serial.print(accel_desc.data.gyr0, 4);
            Serial.print(", ");
            Serial.print(accel_desc.data.gyr1, 4);
            Serial.print(", ");
            Serial.print(accel_desc.data.gyr2, 4);
            Serial.println();
            has_data = true;
        }
        else
        {
            Serial.println("ASM330: No data (timestamp = 0)");
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
            Serial.print("ICM20948 - Accel: ");
            Serial.print(mag_desc.data.accel0, 4);
            Serial.print(", ");
            Serial.print(mag_desc.data.accel1, 4);
            Serial.print(", ");
            Serial.print(mag_desc.data.accel2, 4);
            Serial.print(" | Gyro: ");
            Serial.print(mag_desc.data.gyr0, 4);
            Serial.print(", ");
            Serial.print(mag_desc.data.gyr1, 4);
            Serial.print(", ");
            Serial.print(mag_desc.data.gyr2, 4);
            Serial.print(" | Mag: ");
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
            Serial.print("MAX10S - Lat, Lon, AltMSL, AltElipsoid: ");
            Serial.print(gps_desc.data.lat, 4);
            Serial.print(", ");
            Serial.print(gps_desc.data.lon, 4);
            Serial.print(", ");
            Serial.print(gps_desc.data.altMSL, 4);
            Serial.print(", ");
            Serial.print(gps_desc.data.altEllipsoid, 4);
            Serial.print("| GPS Lock Type - ");
            Serial.print(gps_desc.data.gpsLockType);
            Serial.println();
            has_data = true;
        }
        else
        {
            Serial.println("MAX10S: No data (timestamp = 0)");
        }

        if(curr_desc.getLastUpdated() > 0) {
            Serial.print("INA219 - Curr: ");
            Serial.println(curr_desc.data.curr, 4);
            has_data = true;
        } else {
            Serial.println("INA219: No data (timestamp = 0)");
        }

        if (!has_data)
        {
            Serial.println("No sensor data received yet...");
        }

        Serial.println("======================");
        */
    }
}

void setup() {
    currentState = PRELAUNCH;
    data = {};

    initFuncs[PRELAUNCH] = &prelaunchInit;
    initFuncs[BOOST] = &boostInit;
    initFuncs[COAST] = &coastInit;
    initFuncs[DROGUE_DESCENT] = &drogueDescentInit;
    initFuncs[MAIN_DESCENT] = &mainDescentInit;
    initFuncs[RECOVERY] = &recoveryInit;
    initFuncs[ABORT] = &abortInit;

    loopFuncs[PRELAUNCH] = &prelaunchLoop;
    loopFuncs[BOOST] = &boostLoop;
    loopFuncs[COAST] = &coastLoop;
    loopFuncs[DROGUE_DESCENT] = &drogueDescentLoop;
    loopFuncs[MAIN_DESCENT] = &mainDescentLoop;
    loopFuncs[RECOVERY] = &recoveryLoop;
    loopFuncs[ABORT] = &abortLoop;

    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN1, OUTPUT);
    pinMode(RED_LED_PIN2, OUTPUT);

    
    // NOTE: Run initialization on the first state
    initStateData(&data);
    (*initFuncs[currentState])(&data);
    sensorsSetup();
    ctx.ekfLooping = false;
    ctx.sdInitialized = initializeLogging(&ctx);

    ctx.airBrakes.attach(SERVO_PIN);
    ctx.airBrakes.writeMicroseconds(SERVO_MIN);

    ctx.estimator = StateEstimator();
    
    BLA::Matrix<3, 1> ecef = {0, 0, 0};
    ctx.estimator.init(ecef, millis());
}

void ekfLoop(Context *ctx) {
    static uint32_t last_accel_time = 0;
    static uint32_t last_mag_time = 0;
    static uint32_t last_gps_time = 0;
    static uint32_t last_baro_time = 0;

    uint32_t now = millis();

    const auto &accel_desc = ctx->accel.get_descriptor();
    const auto &baro_desc = ctx->baro.get_descriptor();
    const auto &mag_desc = ctx->mag.get_descriptor();
    const auto &gps_desc = ctx->gps.get_descriptor();

    bool inAir = currentState == BOOST ||
                 currentState == COAST ||
                 currentState == DROGUE_DESCENT ||
                 currentState == MAIN_DESCENT;

    // Accel and gyro becuase they are on the same sensor
    if(accel_desc.getLastUpdated() > last_accel_time) {
        BLA::Matrix<3, 1> gyro = {accel_desc.data.gyr0, accel_desc.data.gyr1, accel_desc.data.gyr2};
        ctx->estimator.fastGyroProp(gyro, now);

        BLA::Matrix<3, 1> accel = {accel_desc.data.accel0, accel_desc.data.accel1, accel_desc.data.accel2};
        ctx->estimator.fastAccelProp(accel, now);
    }

    if(inAir) {
        if(baro_desc.getLastUpdated() > last_baro_time ||
           mag_desc.getLastUpdated() > last_mag_time ||
           gps_desc.getLastUpdated() > last_gps_time) {
            ctx->estimator.ekfPredict(now); 
        }
    } else {
        if(accel_desc.getLastUpdated() > last_accel_time ||
           baro_desc.getLastUpdated() > last_baro_time ||
           mag_desc.getLastUpdated() > last_mag_time ||
           gps_desc.getLastUpdated() > last_gps_time) {
            ctx->estimator.ekfPredict(now); 
        }
    }

    if(accel_desc.getLastUpdated() > last_accel_time) {
        last_accel_time = accel_desc.getLastUpdated();
        BLA::Matrix<3, 1> accel = {accel_desc.data.accel0, accel_desc.data.accel1, accel_desc.data.accel2};
        ctx->estimator.runAccelUpdate(accel, now);
    }

    if (baro_desc.getLastUpdated() > last_baro_time)
    {
        last_baro_time = baro_desc.getLastUpdated();
        BLA::Matrix<1, 1> baro = {baro_desc.data.pressure};
        ctx->estimator.runBaroUpdate(baro, now);
        ctx->estimator.setTemp(baro_desc.data.temp);
    }

    if (mag_desc.getLastUpdated() > last_mag_time)
    {
        last_mag_time = mag_desc.getLastUpdated();
        BLA::Matrix<3, 1> mag = {mag_desc.data.mag0, mag_desc.data.mag1, mag_desc.data.mag2};
        ctx->estimator.runMagUpdate(mag, now);
    }

    if (gps_desc.getLastUpdated() > last_gps_time)
    {
        last_gps_time = gps_desc.getLastUpdated();
        BLA::Matrix<3, 1> gpsPos = {gps_desc.data.ecefX, gps_desc.data.ecefY, gps_desc.data.ecefZ};
        BLA::Matrix<3, 1> gpsVel = {gps_desc.data.velN, gps_desc.data.velE, gps_desc.data.velD};
        ctx->estimator.runGPSUpdate(gpsPos, gpsVel, false, now);
    }
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

    if(ctx.ekfLooping) {
        ekfLoop(&ctx);
    }

    loggingLoop(&ctx);
}
