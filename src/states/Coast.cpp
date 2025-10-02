#include <State.hpp>

void coastInit (StateData* data) {
    // initialize altitude
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