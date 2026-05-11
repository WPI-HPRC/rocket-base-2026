#pragma once

#include "Context.h"
#include "SdData_generated.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "pgmspace.h"
#include <stdint.h>

bool initializeLogging(Context *ctx);

void loggingLoop(Context *ctx);

template <typename T>
void logSensorData(Context *ctx, flatbuffers::FlatBufferBuilder &builder,
                   hprc::SensorData data_type, const T &d) {
  builder.Clear();

  auto sensorData = builder.CreateStruct(d);

  auto packet =
      hprc::CreateSDPacket(builder, millis(), data_type, sensorData.Union());

  builder.FinishSizePrefixed(packet);

  if (ctx->logFileBufferEnd + builder.GetSize() >= LOG_FILE_BUFFER_SIZE) {
    ctx->logFile.write(ctx->logFileBuffer, ctx->logFileBufferEnd);
    ctx->logFile.flush();
    ctx->logFileBufferEnd = 0;
  }
  memcpy(ctx->logFileBuffer + ctx->logFileBufferEnd, builder.GetBufferPointer(),
         builder.GetSize());
  ctx->logFileBufferEnd += builder.GetSize();
}
