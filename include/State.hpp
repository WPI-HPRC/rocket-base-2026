#pragma once

#include "Context.hpp"

enum StateID {
    PRELAUNCH,
    BOOST,
    COAST,
    DROGUE_DESCENT,
    MAIN_DESCENT,
    RECOVERY,
    ABORT,
    NUM_STATES
};

struct StateData {
    long long currentTime;
    long long deltaTime;
    long long loopCount;
    long long startTime;
    long long lastLoopTime;
};

typedef void (*StateInitFunc)(StateData *data);
typedef StateID (*StateLoopFunc)(StateData *data, Context *ctx);