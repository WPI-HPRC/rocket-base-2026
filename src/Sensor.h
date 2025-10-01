#pragma once

#include "Print.h"
#include "../../TimedPointer/TimedPointer.h"
#include <cstdlib>

class Sensor {
  protected:
    TimedPointer<void> data;
    uint32_t pollingPeriod;
    bool initStatus = false;

    virtual bool init_impl() = 0;

    /**
     * @param dataSize Must be sizeof the struct pointed to by `data`
     */
    Sensor(size_t dataSize, uint32_t pollingPeriod)
        : data(dataSize), pollingPeriod(pollingPeriod) {}

  public:
    /**
     * @brief gets if the sensor was successfully initialized
     * @return whether or not the sensor was initialized
     */
    bool getInitStatus();

    /**
     * @brief Polls the sensor.
     * @note _Must_ set fields in struct pointed to by `data`
     */
    virtual void poll() = 0;

    uint32_t getLastTimePolled();

    bool init();

    /**
     * gets polling period of the sensor in millis
     * @return long, the polling period of the sensor in millis
     */
    uint32_t getPollingPeriod();

    virtual ~Sensor() = default;
};

