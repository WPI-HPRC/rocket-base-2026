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
    long long startTime;
    long long lastLoopTime;
};

typedef void (*StateInitFunc)(stateData *data);
typedef State (*StateLoopFunc)(stateData *data, Context *ctx);