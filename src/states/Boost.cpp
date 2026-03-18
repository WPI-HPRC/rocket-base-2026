#include "../State.h"
#include "StateMachineConstants.h"
//#include "../qmekf.h"

void boostInit (StateData* data) {}

StateID boostLoop (StateData* data, Context* ctx) {
    /*
    - Poll acceleration data from ctx
    - Check acceleration to detect coast stage
    - Check if maximum boost time is exceeded
    - Check if need to abort
    - Update sensor data and ctx for next iteration?
    */
    //const auto &accel_desc = ctx->accel.get_descriptor();
    //StateEstimator state_estimator = ctx->estimator;
    const auto acc_vec = ctx->estimator.get_accel_prev();
    if (data->accelDebouncer.update(abs(acc_vec(0, 0)) < COAST_THRESHOLD, millis()) ||  data->currentTime > 2000) {
        // check that acceleration up is less than threshold, or
        // current time > 2000
        return COAST;
    }

    return BOOST;
}
