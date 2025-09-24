// TODO: pull out into a header
enum State {
    PRELAUNCH,
    BOOST,
    COAST,
    DROGUE_DESCENT,
    MAIN_DESCENT,
    RECOVERY,
    ABORT,
    NUM_STATES
};

struct Context {
    // ...
};

struct stateData {
    long long currentTime;
    long long deltaTime;
    long long loopCount;
    // Context *ctx -- Passed to the function every time anyway
    long long startTime;
    long long lastLoopTime;
};

typedef void (*StateInitFunc)(stateData *data);
typedef State (*StateLoopFunc)(stateData *data, Context *ctx);

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

// NOTE: state implimentation function can be pulled out into their own files to reduce the complexity of main
void prelaunchInitialize(stateData *data) { 
    commonStateInitialization(data);
        /* initializatoin */ 
}

// NOTE: Maybe a preprocessor macro to make these signatures uniform
State prelaunchLoop(stateData *data, Context *ctx) {
    // NOTE: could be moved out of this funtio to where it is called from to reduce dupliction
    commonStateLoop(data); 

    // do state loop stuff

    if(/* Should I procede? */) {
        return BOOST;
    } else if(/* Should abort */) {
        return ABORT;
    } else {
        return PRELAUNCH;
    }
}

// NOTE: probably shouldn't be globals?
State currentState;
stateData data;

StateInitFunc initFuncs[NUM_STATES] = {};
StateLoopFunc loopFuncs[NUM_STATES] = {};

void setup() {
    currentState = PRELAUNCH;
    data = {};

    initFuncs[PRELAUNCH] = prelaunchInitialize;

    loopFuncs[PRELAUNCH] = prelaunchLoop;
    // TODO: add the rest of the states 

    // NOTE: maybe this should be done somewhere else just needs to happen
    (*initFuncs[currentState])(&data);
}

void loop() {
    State newState = (*loopFuncs[currentState])(&data, &ctx);

    if(currentState != newState) {
        (*initFuncs[newState])(&data);
        currentState = newState;
    }
}
