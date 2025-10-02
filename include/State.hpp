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

struct StateData {
    long long currentTime;
    long long deltaTime;
    long long loopCount;
    long long startTime;
    long long lastLoopTime;
};

typedef void (*StateInitFunc)(StateData *data);
typedef State (*StateLoopFunc)(StateData *data, Context *ctx);