#include "../State.h"

void coastInit (StateData* data) { }

StateID coastLoop (StateData* data, Context* ctx) {
    static bool airBrakesOut = false;
    static bool airBrakesDone = false;
    static double prevAltitude = 0;

    /*
    - Poll acceleration data from ctx
    - Check acceleration to detect drouge deployment
    - Check if maximum coast time is exceeded
    - Check if need to abort
    - Update sensor data and ctx for next iteration?
    */

    //const auto &baro_desc = ctx->baro.get_descriptor();
    //if (baro_desc.getLastUpdated() != data->lastBaroReadingTime) {
    //    data->lastBaroReadingTime = baro_desc.getLastUpdated();
    //    double currentAltitiude = solveAltitude(baro_desc.data.pressure);
    //    if(data->baroDebouncer.update((currentAltitiude - prevAltitude) < 0 ,millis()) && airBrakesDone) {
    //        return DROGUE_DESCENT;
    //    }
    //    prevAltitude = currentAltitiude;
    //}
    const auto acc_vec = ctx->estimator.get_vel_ned();
    if(acc_vec(2, 0) < 0.2 && acc_vec(2, 0) > -0.2 && airBrakesDone) {
        // check acceleration down in NED frame is between 0.2 and -0.2
        // and airbreaks done
        return DROGUE_DESCENT;
    }

    return COAST;
}
