#include <State.hpp>
#include <stdio.h>


void boostInit (StateData* data) {
    printf("Boost Started\n");
}

StateID boostLoop (StateData* data, Context* ctx) {
    /*
    - Poll acceleration data from ctx
    - Check acceleration to detect coast stage
    - Check if maximum boost time is exceeded
    - Check if need to abort
    - Update sensor data and ctx for next iteration?
    */

    return COAST;
}