#include <State.hpp>
#include <stdio.h>

void prelaunchInit (StateData* data) {
    printf("Prelaunch started\n");
}

StateID prelaunchLoop (StateData* data, Context* ctx) {
    /*
    - Poll acceleration data from ctx
    - Check acceleration to detect launch
    - Check if need to abort
    - Update sensor data and ctx for next iteration?
    */

    // Test Stuff
    if(ctx->contextData == 0) {
        return BOOST;
    } else {
        ctx->contextData -= 1;
    }

    printf("Count Down: %d\n", ctx->contextData);
    return PRELAUNCH;
}