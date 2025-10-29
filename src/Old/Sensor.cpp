#include "Sensor.h"

bool Sensor::init() {
    initStatus = init_impl();
    return initStatus;
}

uint32_t Sensor::getLastTimePolled() { return data.getLastUpdated(); }

bool Sensor::getInitStatus() { return initStatus; }

uint32_t Sensor::getPollingPeriod() { return pollingPeriod; }

