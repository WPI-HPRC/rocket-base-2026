#pragma once

#ifdef __has_include
  #if defined(TEMPLATE_STATES_OVERRIDE) || !__has_include("states/States.h")
    #include "template_states/States.h"
  #elif __has_include("states/States.h")
    #include "states/States.h"
  #endif
#else
  #warning No __has_include, falling back to template_states
  #include "template_states/States.h"
#endif

#include "Context.h"

typedef void (*StateInitFunc)(StateData *data);
typedef StateID (*StateLoopFunc)(StateData *data, Context *ctx);

extern StateInitFunc initFuncs[NUM_STATES];
extern StateLoopFunc loopFuncs[NUM_STATES];

void initStateMap();
