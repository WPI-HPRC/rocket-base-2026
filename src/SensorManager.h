#pragma once

#include "../Sensor/Sensor.h"

class SensorManager {
  public:
    // NOTE: the `sensors field` being declared as Sensor * (&sensors)[N] is to
    // force the compiler to not decay the array into a poiner, and therefore
    // allow template argument deduction to work.
    SensorManager<MillisFn, N>(Sensor *(&sensors)[N], MillisFn millis)
        : sensors(sensors), millis(millis) {}

    // read the sensors
    void loop() {
        for (size_t i = 0; i < N; i++) {
            if (sensors[i]->getInitStatus()) {
                uint32_t currentTime = this->millis();
                if (currentTime - sensors[i]->getLastTimePolled() >=
                    sensors[i]->getPollingPeriod()) {
                    sensors[i]->poll();
                }
            }
        }
    }

    bool sensorInit() {
        bool success = true;
        for (size_t i = 0; i < N; i++) {
            sensors[i]->init();
            success = success && sensors[i]->getInitStatus();
        }
        return success;
    } // true if success false if something fail

  private:
    Sensor **sensors;
    MillisFn millis; // time
    uint32_t currentTime = 0;
};
