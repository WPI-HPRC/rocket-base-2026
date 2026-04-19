#include "../State.h"

void mainDescentInit(StateData *data) {}

StateID mainDescentLoop (StateData* data, Context* ctx) {
    // when we land we are no longer falling
    // should see vertical velocity that is zero

    const auto vel_vec = ctx->estimator.get_vel_ned();
    // check if |vertical velocity| less than 2 m/s
    if(data->velDebouncer.update(abs(vel_vec(2, 0)) < 2, millis())) {  
        return RECOVERY;
    }

    return MAIN_DESCENT;
}
