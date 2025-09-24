// TODO: pull out into a header
enum State {
    PRELAUNCH,
    BOOST,
    COAST,
    DROGUE_DESCENT,
    MAIN_DESCENT,
    RECOVERY,
    ABORT,
};

struct Context {
    ...
};

struct stateData {
    long long currentTime;
    long long deltaTime;
    long long loopCount;
    // Context *ctx -- Passed to the function every time anyway
    long long startTime;
    long long lastLoopTime;
};

// ---------------------

Context ctx = {
#if defined(MARS)
    .accel = ASM330(),
    .baro = LPS22(),
    .mag = ICM20948(),
    .sd = SdFs(),
#elif defined(POLARIS)
    .accel = ICM42688_(),
    .baro = MS5611(),
    .mag = MMC5983(),
#endif
    .gps = MAX10S(),
    .airbrakes = AirbrakeController(AIRBRAKE_SERVO_PIN, AIRBRAKE_FEEDBACK_PIN),
    .flightMode = false,
    .xbeeLoggingDelay = 50,
    .attEkfLogger = AttEkfLogger(),
    .pvKFLogger = PVEkfLogger(),
};

void commonStateLoop(stateData *data) {
    long long now = millis();
    // These values may be used in the state code
    data->currentTime = now - data->startTime;
    data->deltaTime = now - data->lastLoopTime;
    data->lastLoopTime = now;
    data->loopCount++;
}

void commonStateInitialization(stateData *data) {
    data->startTime = millis();
}

// NOTE: state implimentation function can be pulled out into their own files it reduce the complexity of main
void prelaunchInitialize(stateData *data) { 
    commonStateInitialization(data);
        /* initializatoin */ 
}

State prelaunchLoop(stateData *data, Context *ctx) {
    // NOTE: could be moved out of this funtio to where it is called from to reduce dupliction
    commonStateLoop(data); 

    // do state loop stuff

    if(/* Should procede */) {
        return BOOST;
    } else if(/* Should abort */) {
        return ABORT;
    } else {
        return PRELAUNCH;
    }
}

// NOTE: probably shouldn't be globals 
State currentState;
stateData data;
bool initialized;

void setup() {
    currentState = PRELAUNCH;
    data = {};
    initialized = false;
}

void loop() {
    switch(currentState) {
        case PRELAUNCH: {
            // NOTE: I am certian this is not the best way to do initialization
            // probably a point for fnptr method
            if (!initialized) {
                prelaunchInitialize(&data);
                initialized = true;
            }
            State newState = prelaunchLoop(&data, &ctx);
            if (newState != currentState) {
                currentState = newState;
                initialized = false;
            }
        } break;
    }
}
