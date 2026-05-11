#include "SdData_generated.h"
#include "flatbuffers/flatbuffers.h"
#include <cstdio>
#include <cstring>
#include <sys/stat.h>

using namespace flatbuffers;
using namespace hprc;

int main(int argc, char **argv) {
  if (argc < 2) {
    printf("Usage: %s <flatbuffer_binary_data_file>\n", argv[0]);
    return 1;
  }

  FILE *data = fopen(argv[1], "rb");
  if (data == NULL) {
    printf("Failed to open file %s, errno: %d\n", argv[1], errno);
    return 1;
  }

  if (mkdir("decoded", 0777) < 0 && errno != EEXIST) {
    printf("Failed to create directory for decoded files: %d\n", errno);
    return 1;
  }

  FILE *asm330 = fopen("decoded/asm330.csv", "w");
  fprintf(asm330, "timestamp,accel0,accel1,accel2,gyr0,gyr1,gyr2\n");
  FILE *lsm6 = fopen("decoded/lsm6.csv", "w");
  fprintf(lsm6, "timestamp,accel0,accel1,accel2,gyr0,gyr1,gyr2\n");
  FILE *lis2mdl = fopen("decoded/lis2mdl.csv", "w");
  fprintf(lis2mdl, "timestamp,mag0,mag1,mag2\n");
  FILE *lps22 = fopen("decoded/lps22.csv", "w");
  fprintf(lps22, "timestamp,pressure,temp\n");
  FILE *liv3f = fopen("decoded/liv3f.csv", "w");
  fprintf(liv3f, "timestamp,lat,lon,alt,satellites,epochTime\n");

  char buff[1024];

  char *head = buff;
  size_t endIdx = fread(head, 1, 1024, data);

  while (true) {
    if (head - buff >= endIdx) {
      break;
    }

    uoffset_t remainingBytes = endIdx - (head - buff);

    uoffset_t nextPacketSize = *(uoffset_t *)head + sizeof(uoffset_t);
    if (nextPacketSize > remainingBytes) {
      memmove(buff, head, remainingBytes);
      size_t bytesNeeded = 1024 - remainingBytes;
      endIdx =
          remainingBytes + fread(buff + remainingBytes, 1, bytesNeeded, data);
      if (endIdx == remainingBytes) {
        break;
      }

      head = buff;

      continue;
    }

    const SDPacket *packet = GetSizePrefixedSDPacket(head);
    head += nextPacketSize;
    switch (packet->data_type()) {
    case SensorData_NONE:
      break;
    case SensorData_ASM330:
      fprintf(
          asm330, "%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n", packet->timestamp(),
          packet->data_as_ASM330()->accel0(),
          packet->data_as_ASM330()->accel1(),
          packet->data_as_ASM330()->accel2(), packet->data_as_ASM330()->gyr0(),
          packet->data_as_ASM330()->gyr1(), packet->data_as_ASM330()->gyr2());
      break;
    case SensorData_LSM6:
      fprintf(lsm6, "%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n", packet->timestamp(),
              packet->data_as_LSM6()->accel0(),
              packet->data_as_LSM6()->accel1(),
              packet->data_as_LSM6()->accel2(), packet->data_as_LSM6()->gyr0(),
              packet->data_as_LSM6()->gyr1(), packet->data_as_LSM6()->gyr2());
      break;
    case SensorData_LIS2MDL:
      fprintf(lis2mdl, "%d,%.8f,%.8f,%.8f\n", packet->timestamp(),
              packet->data_as_LIS2MDL()->mag0(),
              packet->data_as_LIS2MDL()->mag1(),
              packet->data_as_LIS2MDL()->mag2());
      break;
    case SensorData_LPS22:
      fprintf(lps22, "%d,%.8f,%.8f\n", packet->timestamp(),
              packet->data_as_LPS22()->pressure(),
              packet->data_as_LPS22()->temp());
      break;
    case SensorData_LIV3F:
      fprintf(liv3f, "%d,%.8f,%.8f,%.8f,%hhd,%d\n", packet->timestamp(),
              packet->data_as_LIV3F()->lat(), packet->data_as_LIV3F()->lon(),
              packet->data_as_LIV3F()->alt(),
              packet->data_as_LIV3F()->satellites(),
              packet->data_as_LIV3F()->epochTime());
      break;
    }
  }

  exit(0);
}
