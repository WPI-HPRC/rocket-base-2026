#include "../State.h"
#include "StateMachineConstants.h"

void drogueDescentInit(StateData *data) {}

StateID drogueDescentLoop (StateData* data, Context* ctx) {

    // under main descent if velocity down is between 16 to 30 fps
    // this is abt 5m/s to 9m/s
    const auto vel_vec = ctx->estimator.get_vel_ned();
    // velocity in the vertical direction is negative
    // going down will be a positive value
    if(data->accelDebouncer.update(abs(vel_vec(2, 0)) > MAIN_MIN_VEL, millis()) && abs(vel_vec(2, 0)) < MAIN_MAX_VEL, millis()) {
        return MAIN_DESCENT;
    }

    return DROGUE_DESCENT;
}
