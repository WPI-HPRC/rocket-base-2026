#include "SdData_generated.h"
#include "Sensors_generated.h"
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
    const Sensors *sensors = packet->sensors();

    if (sensors->ASM330() != nullptr) {
      fprintf(asm330, "%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n", packet->timestamp(),
              sensors->ASM330()->accel0(), sensors->ASM330()->accel1(),
              sensors->ASM330()->accel2(), sensors->ASM330()->gyr0(),
              sensors->ASM330()->gyr1(), sensors->ASM330()->gyr2());
    }
    if (sensors->LSM6() != nullptr) {
      fprintf(lsm6, "%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n", packet->timestamp(),
              sensors->LSM6()->accel0(), sensors->LSM6()->accel1(),
              sensors->LSM6()->accel2(), sensors->LSM6()->gyr0(),
              sensors->LSM6()->gyr1(), sensors->LSM6()->gyr2());
    }
    if (sensors->LIS2MDL() != nullptr) {
      fprintf(lis2mdl, "%d,%.8f,%.8f,%.8f\n", packet->timestamp(),
              sensors->LIS2MDL()->mag0(), sensors->LIS2MDL()->mag1(),
              sensors->LIS2MDL()->mag2());
    }
    if (sensors->LPS22() != nullptr) {
      fprintf(lps22, "%d,%.8f,%.8f\n", packet->timestamp(),
              sensors->LPS22()->pressure(), sensors->LPS22()->temp());
    }
    if (sensors->LIV3F() != nullptr) {
      fprintf(liv3f, "%d,%.8f,%.8f,%.8f,%hhd,%d\n", packet->timestamp(),
              sensors->LIV3F()->lat(), sensors->LIV3F()->lon(),
              sensors->LIV3F()->alt(), sensors->LIV3F()->satellites(),
              sensors->LIV3F()->epochTime());
    }
  }

  exit(0);
}
