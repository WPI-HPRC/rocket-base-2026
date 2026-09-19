#include "SdData_generated.h"
#include "Sensors_generated.h"
#include "flatbuffers/flatbuffers.h"
#include <cerrno>
#include <cstdint>
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

  // The log can start mid-packet and lose bytes (e.g. when captured over a
  // serial link), so every packet is verified before it is decoded and the
  // decoder rescans byte by byte for the next valid one after any damage.
  constexpr size_t MAX_PACKET_SIZE = 1024;
  alignas(8) uint8_t packetBuf[MAX_PACKET_SIZE + sizeof(uoffset_t)];

  size_t packetCount = 0;
  size_t skippedBytes = 0;
  size_t skippedRegions = 0;
  bool skipping = false;

  uint8_t buff[4096];
  size_t endIdx = fread(buff, 1, sizeof(buff), data);
  size_t head = 0;

  while (true) {
    size_t remainingBytes = endIdx - head;

    // Refill once fewer bytes remain than the largest possible packet
    if (remainingBytes < MAX_PACKET_SIZE + sizeof(uoffset_t) && !feof(data)) {
      memmove(buff, buff + head, remainingBytes);
      endIdx = remainingBytes +
               fread(buff + remainingBytes, 1, sizeof(buff) - remainingBytes,
                     data);
      head = 0;
      remainingBytes = endIdx;
    }

    if (remainingBytes < sizeof(uoffset_t)) {
      skippedBytes += remainingBytes;
      break;
    }

    // Packets aren't guaranteed to be 4-byte aligned in the file
    uoffset_t packetSize;
    memcpy(&packetSize, buff + head, sizeof(packetSize));
    size_t totalSize = (size_t)packetSize + sizeof(uoffset_t);

    bool valid = false;
    if (packetSize <= MAX_PACKET_SIZE && totalSize <= remainingBytes) {
      // Verify on an aligned copy, flatbuffers checks alignment
      memcpy(packetBuf, buff + head, totalSize);
      Verifier verifier(packetBuf, totalSize);
      valid = VerifySizePrefixedSDPacketBuffer(verifier) &&
              GetSizePrefixedSDPacket(packetBuf)->sensors() != nullptr;
    }

    if (!valid) {
      if (!skipping) {
        skippedRegions++;
        skipping = true;
      }
      head++;
      skippedBytes++;
      continue;
    }
    skipping = false;

    head += totalSize;
    packetCount++;

    const SDPacket *packet = GetSizePrefixedSDPacket(packetBuf);
    const Sensors *sensors = packet->sensors();

    if (sensors->asm330() != nullptr) {
      fprintf(asm330, "%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n", packet->timestamp(),
              sensors->asm330()->accel0(), sensors->asm330()->accel1(),
              sensors->asm330()->accel2(), sensors->asm330()->gyr0(),
              sensors->asm330()->gyr1(), sensors->asm330()->gyr2());
    }
    if (sensors->lsm6() != nullptr) {
      fprintf(lsm6, "%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n", packet->timestamp(),
              sensors->lsm6()->accel0(), sensors->lsm6()->accel1(),
              sensors->lsm6()->accel2(), sensors->lsm6()->gyr0(),
              sensors->lsm6()->gyr1(), sensors->lsm6()->gyr2());
    }
    if (sensors->lis2mdl() != nullptr) {
      fprintf(lis2mdl, "%d,%.8f,%.8f,%.8f\n", packet->timestamp(),
              sensors->lis2mdl()->mag0(), sensors->lis2mdl()->mag1(),
              sensors->lis2mdl()->mag2());
    }
    if (sensors->lps22() != nullptr) {
      fprintf(lps22, "%d,%.8f,%.8f\n", packet->timestamp(),
              sensors->lps22()->pressure(), sensors->lps22()->temp());
    }
    if (sensors->liv3f() != nullptr) {
      fprintf(liv3f, "%d,%.8f,%.8f,%.8f,%hhd,%d\n", packet->timestamp(),
              sensors->liv3f()->lat(), sensors->liv3f()->lon(),
              sensors->liv3f()->alt(), sensors->liv3f()->satellites(),
              sensors->liv3f()->epoch_time());
    }
  }

  printf("Decoded %zu packets, skipped %zu bytes in %zu corrupt region(s)\n",
         packetCount, skippedBytes, skippedRegions);

  fclose(asm330);
  fclose(lsm6);
  fclose(lis2mdl);
  fclose(lps22);
  fclose(liv3f);
  fclose(data);

  return 0;
}
