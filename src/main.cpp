#include <Arduino.h>
#include "Context.hpp"
#include "State.hpp"
#include "States.hpp"

/*
#include "states/Abort.cpp"
#include "states/Boost.cpp"
#include "states/Coast.cpp"
#include "states/DrogueDescent.cpp"
#include "states/MainDescent.cpp"
#include "states/PreLaunch.cpp"
#include "states/Recovery.cpp"
*/


Context ctx = {
    .contextData = 7
};


StateID currentState;
StateData data;

StateInitFunc initFuncs[NUM_STATES] = {};
StateLoopFunc loopFuncs[NUM_STATES] = {};

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

    // NOTE: Run initialization on the first state
    (*initFuncs[currentState])(&data);
}

void loop() {
    StateID newState = (*loopFuncs[currentState])(&data, &ctx);

    if(currentState != newState) {
        (*initFuncs[newState])(&data);
        currentState = newState;
    }
}
