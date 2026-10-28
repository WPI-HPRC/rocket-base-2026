#pragma once

#include <Print.h>
#include <stddef.h>
#include <stdint.h>

// Session based logging to bank 2 of the on-chip flash (0x08100000, 1 MB).
// Firmware lives in bank 1 (platformio.ini caps it at 1 MB), so writing here
// never stalls code fetches.
//
// Bank 2 is 8 sectors of 128 KB, which is the erase unit. Flash is programmed
// one 32 byte flash word at a time and, because of ECC, a word can only be
// programmed once between erases. So nothing is ever updated in place:
//
// Sector 0 is the table: an append-only journal of 32 byte FlashRecords.
// Replaying it at boot gives the owner of every data sector, and a sector with
// no owner is free. Records: FORMAT (always slot 0), SESSION_START,
// SECTOR_CLAIM (session took a sector, with its order in the session),
// SECTOR_RELEASE (sector was erased), SESSION_PROTECT and SESSION_LANDED.
// The journal is compacted at boot once it is 3/4 full.
//
// Sectors 1-7 hold data. Word 0 is a SECTOR_HEADER record (owning session and
// order), word 1 is a SECTOR_PROTECT record once the session is protected,
// word 2 is a SECTOR_SYNC record giving the offset of the first whole data
// record in the sector, then data words of
// [stream id][payload length][30 payload bytes].
// The headers duplicate the table, so a lost table is rebuilt from them (and
// scripts/flashlog_extract.py reads them directly).
//
// Writes go into a small RAM queue that is drained to flash every loop, so a
// power cut loses at most what is in the queue plus one partial word. Erases
// (1-2 s per sector) run in the background while the queue absorbs incoming
// data; nothing blocks the main loop after boot.
//
// Each boot is a new session, written to flash from the start:
// - While unprotected (on the pad), a session keeps at most
//   FLASH_LOG_PRELAUNCH_SECTORS sectors and erases its own oldest one to
//   continue, so pad time can't fill the flash.
// - Protecting a session (launch) stops that; it keeps every sector it fills.
// - If the previous session was protected but never marked landed, the board
//   rebooted mid flight, so the new session starts protected.
// - Old unprotected sessions are erased oldest first to keep
//   FLASH_LOG_MIN_FREE_SECTORS free. Protected sessions are only removed by
//   formatting.

enum FlashLogStream : uint8_t {
  FLASH_LOG_STREAM_DATA = 1, // size prefixed SDPackets, same as flightDataN.fb.bin
  FLASH_LOG_STREAM_TEXT = 2, // debug log text, same as debugLogN.txt
};

// Mounts the log and starts a new session. format erases every session,
// protected or not, and blocks while it does.
bool flashLogBegin(bool format);

// Runs background erases and drains the RAM queue to flash. Call every loop.
void flashLogLoop();

// Queues one record. Data records are never split across sectors, so the
// extractor can resync on them. Returns false (and drops it) if the queue is full.
bool flashLogWrite(FlashLogStream stream, const uint8_t *data, size_t len);

// Writes out partially filled words once the queue has drained.
void flashLogFlush();

// Keeps the current session forever (until format). Call at launch.
void flashLogProtectSession();

// Marks a normal end of flight, so the next boot is treated as on the ground.
void flashLogMarkLanded();

bool flashLogIsProtected();
bool flashLogIsFull();
uint32_t flashLogSessionId();
uint32_t flashLogDroppedBytes();

// Payload bytes the current session can still write without erasing.
size_t flashLogBytesFree();

void flashLogPrintTable(Print &out);

// Print adapter for ArduinoLog, queues text a line at a time.
class FlashTextLog : public Print {
public:
  size_t write(uint8_t c) override;

private:
  uint8_t line[96];
  size_t lineLen = 0;
};
