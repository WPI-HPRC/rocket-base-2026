#include "../State.h"

void IRECRocketCoastAction() {
    static bool airBrakesOut = false;
    static bool airBrakesDone = false;

    /*
    - Poll acceleration data from ctx
    - Check acceleration to detect drouge deployment
    - Check if maximum coast time is exceeded
    - Check if need to abort
    - Update sensor data and ctx for next iteration?
    */

    if(data->currentTime > 1000 && !airBrakesOut) {
        ctx->airBrakes.writeMicroseconds(SERVO_MAX);
        airBrakesOut = true;
    }
    if(data->currentTime > 5000 && airBrakesOut) {
        ctx->airBrakes.writeMicroseconds(SERVO_MIN);
        airBrakesDone = true;
    }
}

void TwoStageRocketCoastAction() {
    //TODO
}

double solveAltitude(double pressure) {
    // physical parameters for model
    const double pb = 101325;   // [Pa] pressure at sea level
    const double Tb = 288.15;   // [K] temperature at seal level
    const double Lb = -0.0065;  // [K/m] standard temperature lapse rate
    const double hb = 0;        // [m] height at bottom of atmospheric layer (sea level)
    const double R = 8.31432;   // [N*m/mol*K] universal gas constant
    const double g0 = 9.80665;  // [m/s^2] Earth standard gravity
    const double M = 0.0289644; // [kg/mol] molar mass of Earth's air

    double pressure_Pa = pressure * 100;

    return hb +
           (Tb / Lb) * (pow((pressure_Pa / pb), (-R * Lb / (g0 * M))) - 1);
}

void coastInit (StateData* data) { }

// Rocket types:
//  IREC_ROCKET
//  TWO_STAGE_ROCKET
StateID coastLoop (StateData* data, Context* ctx) {
    static double prevAltitude = 0; // is this still needed?

    #ifdef IREC_ROCKET
    IRECRocketCoastAction();
    #endif
    #ifdef TWO_STAGE_ROCKET
    TwoStageRocketCoastAction();
    #endif

    //const auto &baro_desc = ctx->baro.get_descriptor();
    //if (baro_desc.getLastUpdated() != data->lastBaroReadingTime) {
    //    data->lastBaroReadingTime = baro_desc.getLastUpdated();
    //    double currentAltitiude = solveAltitude(baro_desc.data.pressure);
    //    if(data->baroDebouncer.update((currentAltitiude - prevAltitude) < 0 ,millis()) && airBrakesDone) {
    //        return DROGUE_DESCENT;
    //    }
    //    prevAltitude = currentAltitiude;
    //}
    const auto acc_vec = ctx->estimator.get_vel_prev_ned();
    if(acc_vec(2, 0) < 0.2 && acc_vec(2, 0) > -0.2 && airBrakesDone) {
        // check acceleration down in NED frame is between 0.2 and -0.2
        // and airbreaks done
        return DROGUE_DESCENT;
    }

    return COAST;
}