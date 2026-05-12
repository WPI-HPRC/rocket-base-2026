#pragma once

#include "Context.h"
#include "SdData_generated.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "pgmspace.h"
#include <stdint.h>

bool initializeLogging(Context *ctx);

void loggingLoop(Context *ctx);
