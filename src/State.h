#pragma once

#include "Context.h"
#include "debouncer.h"

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
    uint32_t lastAccelReadingTime;
    uint32_t lastBaroReadingTime;
    Debouncer baroDebouncer = Debouncer(20);
    Debouncer accelDebouncer = Debouncer(20);
    Debouncer velDebouncer = Debouncer(20);
};

typedef void (*StateInitFunc)(StateData *data);
typedef StateID (*StateLoopFunc)(StateData *data, Context *ctx);
