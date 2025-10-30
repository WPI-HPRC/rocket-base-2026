#include <State.hpp>
#include <stdio.h>

void coastInit (StateData* data) {
    // initialize altitude
    printf("Coast Started\n");
}

StateID coastLoop (StateData* data, Context* ctx) {
    /*
    - Poll acceleration data from ctx
    - Check acceleration to detect drouge deployment
    - Check if maximum coast time is exceeded
    - Check if need to abort
    - Update sensor data and ctx for next iteration?
    */

    return DROGUE_DESCENT;
}