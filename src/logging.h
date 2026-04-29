#pragma once

#include <stdint.h>
#include "Context.h"
#include "SdData_generated.h"
#include "flatbuffers/flatbuffer_builder.h"

bool initializeLogging(Context *ctx);

void loggingLoop(Context *ctx);

void logSensorData(flatbuffers::FlatBufferBuilder *builder, File *logFile, hprc::SensorData *const &d);