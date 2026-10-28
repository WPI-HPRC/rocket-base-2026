#include "flash_log.h"

#include "config.h"
#include <Arduino.h>
#include <string.h>

#define FLASH_LOG_BASE 0x08100000UL
#define FLASH_LOG_SECTOR_SIZE FLASH_SECTOR_SIZE
#define FLASH_LOG_NUM_SECTORS 8
#define FLASH_LOG_TABLE_SECTOR 0
#define FLASH_LOG_WORD_SIZE 32
#define FLASH_LOG_WORDS_PER_SECTOR (FLASH_LOG_SECTOR_SIZE / FLASH_LOG_WORD_SIZE)
#define FLASH_LOG_HEADER_WORDS 3
#define FLASH_LOG_DATA_WORDS (FLASH_LOG_WORDS_PER_SECTOR - FLASH_LOG_HEADER_WORDS)
#define FLASH_LOG_PAYLOAD_PER_WORD (FLASH_LOG_WORD_SIZE - 2)
#define FLASH_LOG_COMPACT_AT (FLASH_LOG_WORDS_PER_SECTOR * 3 / 4)

// Queue bytes moved to flash per flashLogLoop, ~70 words
#define FLASH_LOG_DRAIN_BYTES_PER_LOOP 2048
#define FLASH_LOG_MAX_RECORD 1024
#define FLASH_LOG_QUEUE_HEADER 3

#define FLASH_LOG_MAGIC 0x474F4C46UL // "FLOG"
#define FLASH_LOG_VERSION 1

// Owners that aren't sessions
#define SESSION_UNUSABLE UINT32_MAX     // erase failed, never handed out this boot
#define SESSION_DIRTY (UINT32_MAX - 1)  // free in the table but not blank, gets erased

enum RecordType : uint8_t {
  RECORD_FORMAT = 1,          // seq = version, arg = next session id
  RECORD_SESSION_START = 2,   // session
  RECORD_SECTOR_CLAIM = 3,    // session, sector, seq
  RECORD_SESSION_PROTECT = 4, // session
  RECORD_SESSION_LANDED = 5,  // session
  RECORD_SECTOR_HEADER = 6,   // word 0 of a data sector: session, sector, seq
  RECORD_SECTOR_PROTECT = 7,  // word 1 of a data sector: session, sector
  RECORD_SECTOR_SYNC = 8,     // word 2 of a data sector: arg = offset of the first whole data record
  RECORD_SECTOR_RELEASE = 9,  // session, sector (it has been erased)
};

struct alignas(4) FlashRecord {
  uint32_t magic;
  uint8_t type;
  uint8_t sector;
  uint16_t seq;
  uint32_t session;
  uint32_t arg;
  uint32_t reserved[3];
  uint32_t crc; // CRC-32 of the first 28 bytes
};
static_assert(sizeof(FlashRecord) == FLASH_LOG_WORD_SIZE,
              "FlashRecord must be one flash word");
static_assert(FLASH_LOG_PRELAUNCH_SECTORS >= 2 && FLASH_LOG_PRELAUNCH_SECTORS < FLASH_LOG_NUM_SECTORS - 1,
              "FLASH_LOG_PRELAUNCH_SECTORS must be 2-6");

struct SectorState {
  uint32_t session; // 0 = free
  uint16_t seq;
  bool isProtected;
};

enum WordState { WORD_BLANK, WORD_VALID, WORD_INVALID };

// Table model
static SectorState sectors[FLASH_LOG_NUM_SECTORS];
static uint32_t nextSessionId = 1;
static uint32_t tableSlot = 0;
static uint32_t latestSession = 0;
static bool latestProtected = false;
static bool latestLanded = false;

// Current session
static uint32_t sessionId = 0; // 0 = no session
static uint16_t sessionSeq = 0;
static bool sessionProtected = false;
static bool sessionFull = false;
static int8_t curSector = -1;
static uint8_t claimCursor = 0;
static uint32_t writeAddr = 0;
static uint32_t sectorEnd = 0;
static bool syncWritten = false;

static bool pendingProtect = false;
static bool pendingLanded = false;
static bool pendingFlush = false;
static bool wantSector = false;
static int8_t erasingSector = -1;
static uint32_t droppedBytes = 0;

struct Staging {
  uint8_t len;
  uint8_t bytes[FLASH_LOG_PAYLOAD_PER_WORD];
};
static Staging staging[FLASH_LOG_STREAM_TEXT + 1];

// Data stream positions, used to find where whole records start in a sector
static uint32_t dataWritten = 0; // bytes programmed
static uint32_t dataStaged = 0;  // bytes programmed or staged
static uint32_t recordStart = 0;
static uint32_t recordEnd = 0;

// Queue of [stream][len lo][len hi][payload] records waiting for flash
static uint8_t queue[FLASH_LOG_BUFFER_SIZE];
static size_t queueHead = 0;
static size_t queueSize = 0;

static uint32_t sectorAddr(uint8_t sector) {
  return FLASH_LOG_BASE + sector * FLASH_LOG_SECTOR_SIZE;
}

static uint32_t crc32(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
  }
  return ~crc;
}

static FlashRecord makeRecord(RecordType type, uint32_t session, uint8_t sector,
                              uint16_t seq, uint32_t arg) {
  FlashRecord rec = {};
  rec.magic = FLASH_LOG_MAGIC;
  rec.type = type;
  rec.sector = sector;
  rec.seq = seq;
  rec.session = session;
  rec.arg = arg;
  rec.crc = crc32((const uint8_t *)&rec, offsetof(FlashRecord, crc));
  return rec;
}

// A word that lost power mid-program can fail its ECC check, and reading it
// raises a bus fault. Reads run at FAULTMASK priority with BFHFNMIGN set so
// that fault is ignored and reported here, instead of hard faulting on every
// boot after a bad power cut.
static bool readWord(uint32_t addr, uint32_t out[8]) {
  SCB_InvalidateDCache_by_Addr((void *)addr, FLASH_LOG_WORD_SIZE);
  __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_SNECCERR_BANK2 | FLASH_FLAG_DBECCERR_BANK2);
  SCB->CFSR = SCB_CFSR_BUSFAULTSR_Msk;

  SCB->CCR |= SCB_CCR_BFHFNMIGN_Msk;
  __DSB();
  __ISB();
  __set_FAULTMASK(1);
  volatile const uint32_t *src = (volatile const uint32_t *)addr;
  for (int i = 0; i < 8; i++) {
    out[i] = src[i];
  }
  __DSB();
  __set_FAULTMASK(0);
  SCB->CCR &= ~SCB_CCR_BFHFNMIGN_Msk;
  __DSB();
  __ISB();

  bool ok = (SCB->CFSR & SCB_CFSR_BUSFAULTSR_Msk) == 0 &&
            !__HAL_FLASH_GET_FLAG_BANK2(FLASH_FLAG_DBECCERR_BANK2);
  SCB->CFSR = SCB_CFSR_BUSFAULTSR_Msk;
  __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_SNECCERR_BANK2 | FLASH_FLAG_DBECCERR_BANK2);
  if (!ok) {
    SCB_InvalidateDCache_by_Addr((void *)addr, FLASH_LOG_WORD_SIZE);
  }
  return ok;
}

static bool isBlank(const uint32_t word[8]) {
  for (int i = 0; i < 8; i++) {
    if (word[i] != 0xFFFFFFFF) {
      return false;
    }
  }
  return true;
}

static WordState readRecord(uint32_t addr, FlashRecord *rec) {
  if (!readWord(addr, (uint32_t *)rec)) {
    return WORD_INVALID;
  }
  if (isBlank((const uint32_t *)rec)) {
    return WORD_BLANK;
  }
  if (rec->magic != FLASH_LOG_MAGIC ||
      rec->crc != crc32((const uint8_t *)rec, offsetof(FlashRecord, crc))) {
    return WORD_INVALID;
  }
  return WORD_VALID;
}

static bool sectorIsBlank(uint8_t sector) {
  uint32_t word[8];
  for (uint32_t i = 0; i < FLASH_LOG_WORDS_PER_SECTOR; i++) {
    if (!readWord(sectorAddr(sector) + i * FLASH_LOG_WORD_SIZE, word) ||
        !isBlank(word)) {
      return false;
    }
  }
  return true;
}

static bool programWord(uint32_t addr, const void *word) {
  __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2);
  bool ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr, (uint32_t)word) == HAL_OK;
  SCB_InvalidateDCache_by_Addr((void *)addr, FLASH_LOG_WORD_SIZE);
  return ok;
}

// Only used at boot
static bool eraseSectorBlocking(uint8_t sector) {
  FLASH_EraseInitTypeDef erase = {};
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Banks = FLASH_BANK_2;
  erase.Sector = sector;
  erase.NbSectors = 1;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  uint32_t sectorError;
  __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2);
  bool ok = HAL_FLASHEx_Erase(&erase, &sectorError) == HAL_OK;
  SCB_InvalidateDCache_by_Addr((void *)sectorAddr(sector), FLASH_LOG_SECTOR_SIZE);
  return ok;
}

static void startErase(uint8_t sector) {
  __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2 | FLASH_FLAG_EOP_BANK2);
  FLASH_Erase_Sector(sector, FLASH_BANK_2, FLASH_VOLTAGE_RANGE_3);
  erasingSector = sector;
}

static bool appendTable(const FlashRecord &rec) {
  if (tableSlot >= FLASH_LOG_WORDS_PER_SECTOR) {
    return false;
  }
  uint32_t addr = sectorAddr(FLASH_LOG_TABLE_SECTOR) + tableSlot * FLASH_LOG_WORD_SIZE;
  // A failed program still leaves the slot unusable
  tableSlot++;
  return programWord(addr, &rec);
}

// Finishes a background erase once the flash is idle
static void pollErase() {
  if (erasingSector < 0 || (FLASH->SR2 & FLASH_SR_QW) != 0) {
    return;
  }
  bool ok = (FLASH->SR2 & (FLASH_FLAG_ALL_ERRORS_BANK2 & 0x7FFFFFFFU)) == 0;
  FLASH->CR2 &= ~(FLASH_CR_SER | FLASH_CR_SNB);
  __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2 | FLASH_FLAG_EOP_BANK2);

  uint8_t sector = erasingSector;
  erasingSector = -1;
  SCB_InvalidateDCache_by_Addr((void *)sectorAddr(sector), FLASH_LOG_SECTOR_SIZE);

  if (!ok) {
    sectors[sector] = {SESSION_UNUSABLE, 0, true};
    return;
  }
  if (sectors[sector].session != SESSION_DIRTY) {
    appendTable(makeRecord(RECORD_SECTOR_RELEASE, sectors[sector].session, sector, 0, 0));
  }
  sectors[sector] = {};
}

static bool isSession(uint32_t owner) {
  return owner != 0 && owner != SESSION_DIRTY && owner != SESSION_UNUSABLE;
}

static uint8_t countFreeSectors() {
  uint8_t count = 0;
  for (uint8_t s = 1; s < FLASH_LOG_NUM_SECTORS; s++) {
    if (sectors[s].session == 0) {
      count++;
    }
  }
  return count;
}

static void applyRecord(const FlashRecord &rec) {
  switch (rec.type) {
  case RECORD_SESSION_START:
    nextSessionId = max(nextSessionId, rec.session + 1);
    if (rec.session > latestSession) {
      latestSession = rec.session;
      latestProtected = false;
      latestLanded = false;
    }
    break;
  case RECORD_SECTOR_CLAIM: {
    if (rec.sector == FLASH_LOG_TABLE_SECTOR || rec.sector >= FLASH_LOG_NUM_SECTORS) {
      break;
    }
    // Sectors claimed after a protect inherit it
    bool isProtected = false;
    for (uint8_t s = 1; s < FLASH_LOG_NUM_SECTORS; s++) {
      if (sectors[s].session == rec.session && sectors[s].isProtected) {
        isProtected = true;
      }
    }
    isProtected |= rec.session == latestSession && latestProtected;
    sectors[rec.sector] = {rec.session, rec.seq, isProtected};
    nextSessionId = max(nextSessionId, rec.session + 1);
    break;
  }
  case RECORD_SECTOR_RELEASE:
    if (rec.sector < FLASH_LOG_NUM_SECTORS && sectors[rec.sector].session == rec.session) {
      sectors[rec.sector] = {};
    }
    break;
  case RECORD_SESSION_PROTECT:
    for (uint8_t s = 1; s < FLASH_LOG_NUM_SECTORS; s++) {
      if (sectors[s].session == rec.session) {
        sectors[s].isProtected = true;
      }
    }
    if (rec.session == latestSession) {
      latestProtected = true;
    }
    break;
  case RECORD_SESSION_LANDED:
    if (rec.session == latestSession) {
      latestLanded = true;
    }
    break;
  }
}

static bool loadTable() {
  memset(sectors, 0, sizeof(sectors));
  tableSlot = FLASH_LOG_WORDS_PER_SECTOR;
  latestSession = 0;
  latestProtected = false;
  latestLanded = false;

  FlashRecord rec;
  uint32_t base = sectorAddr(FLASH_LOG_TABLE_SECTOR);
  if (readRecord(base, &rec) != WORD_VALID || rec.type != RECORD_FORMAT ||
      rec.seq != FLASH_LOG_VERSION) {
    return false;
  }
  nextSessionId = max(rec.arg, (uint32_t)1);

  for (uint32_t slot = 1; slot < FLASH_LOG_WORDS_PER_SECTOR; slot++) {
    WordState state = readRecord(base + slot * FLASH_LOG_WORD_SIZE, &rec);
    if (state == WORD_BLANK) {
      tableSlot = slot;
      break;
    }
    if (state == WORD_VALID) {
      applyRecord(rec);
    }
  }
  return true;
}

// Erases the table and writes the current model back as a fresh journal
static bool writeTable() {
  if (!sectorIsBlank(FLASH_LOG_TABLE_SECTOR) && !eraseSectorBlocking(FLASH_LOG_TABLE_SECTOR)) {
    return false;
  }
  tableSlot = 0;
  bool ok = appendTable(makeRecord(RECORD_FORMAT, 0, 0, FLASH_LOG_VERSION, nextSessionId));

  uint32_t lastSession = 0;
  while (true) {
    uint32_t session = UINT32_MAX;
    for (uint8_t s = 1; s < FLASH_LOG_NUM_SECTORS; s++) {
      if (isSession(sectors[s].session) && sectors[s].session > lastSession &&
          sectors[s].session < session) {
        session = sectors[s].session;
      }
    }
    if (session == UINT32_MAX) {
      break;
    }
    lastSession = session;

    ok &= appendTable(makeRecord(RECORD_SESSION_START, session, 0, 0, 0));

    // Claims go back in their original order
    bool isProtected = false;
    int lastSeq = -1;
    while (true) {
      int next = -1;
      for (uint8_t s = 1; s < FLASH_LOG_NUM_SECTORS; s++) {
        if (sectors[s].session == session && sectors[s].seq > lastSeq &&
            (next < 0 || sectors[s].seq < sectors[next].seq)) {
          next = s;
        }
      }
      if (next < 0) {
        break;
      }
      lastSeq = sectors[next].seq;
      isProtected |= sectors[next].isProtected;
      ok &= appendTable(makeRecord(RECORD_SECTOR_CLAIM, session, next, sectors[next].seq, 0));
    }

    if (isProtected) {
      ok &= appendTable(makeRecord(RECORD_SESSION_PROTECT, session, 0, 0, 0));
    }
  }

  // The latest session decides if the next boot is mid flight, keep it even with no sectors
  if (latestSession != 0) {
    if (latestSession != lastSession) {
      ok &= appendTable(makeRecord(RECORD_SESSION_START, latestSession, 0, 0, 0));
      if (latestProtected) {
        ok &= appendTable(makeRecord(RECORD_SESSION_PROTECT, latestSession, 0, 0, 0));
      }
    }
    if (latestLanded) {
      ok &= appendTable(makeRecord(RECORD_SESSION_LANDED, latestSession, 0, 0, 0));
    }
  }
  return ok;
}

// Used when the table is missing or corrupt
static bool rebuildTable() {
  memset(sectors, 0, sizeof(sectors));
  nextSessionId = 1;

  for (uint8_t s = 1; s < FLASH_LOG_NUM_SECTORS; s++) {
    FlashRecord rec;
    if (readRecord(sectorAddr(s), &rec) != WORD_VALID || rec.type != RECORD_SECTOR_HEADER) {
      continue;
    }
    sectors[s] = {rec.session, rec.seq, false};
    nextSessionId = max(nextSessionId, rec.session + 1);

    if (readRecord(sectorAddr(s) + FLASH_LOG_WORD_SIZE, &rec) == WORD_VALID &&
        rec.type == RECORD_SECTOR_PROTECT && rec.session == sectors[s].session) {
      sectors[s].isProtected = true;
    }
  }

  // Landed isn't stored in the sectors, so a protected latest session is
  // assumed to still be flying. Safe side: the next session starts protected.
  latestSession = nextSessionId - 1;
  latestProtected = false;
  latestLanded = false;
  for (uint8_t s = 1; s < FLASH_LOG_NUM_SECTORS; s++) {
    for (uint8_t other = 1; other < FLASH_LOG_NUM_SECTORS; other++) {
      if (sectors[other].session == sectors[s].session && sectors[other].isProtected) {
        sectors[s].isProtected = true;
      }
    }
    if (sectors[s].session == latestSession && sectors[s].isProtected) {
      latestProtected = true;
    }
  }

  return writeTable();
}

static bool claimSector() {
  for (uint8_t i = 0; i < FLASH_LOG_NUM_SECTORS - 1; i++) {
    // Round robin to spread erase wear
    uint8_t s = 1 + (claimCursor + i) % (FLASH_LOG_NUM_SECTORS - 1);
    if (sectors[s].session != 0) {
      continue;
    }

    // Claim before the header, so a power cut leaves an owned (and later
    // reclaimed) sector rather than an unowned one with data in it
    if (!appendTable(makeRecord(RECORD_SECTOR_CLAIM, sessionId, s, sessionSeq, 0))) {
      return false;
    }
    sectors[s] = {sessionId, sessionSeq, sessionProtected};

    FlashRecord header = makeRecord(RECORD_SECTOR_HEADER, sessionId, s, sessionSeq, 0);
    programWord(sectorAddr(s), &header);
    if (sessionProtected && !pendingProtect) {
      FlashRecord protect = makeRecord(RECORD_SECTOR_PROTECT, sessionId, s, sessionSeq, 0);
      programWord(sectorAddr(s) + FLASH_LOG_WORD_SIZE, &protect);
    }

    curSector = s;
    claimCursor = s;
    writeAddr = sectorAddr(s) + FLASH_LOG_HEADER_WORDS * FLASH_LOG_WORD_SIZE;
    sectorEnd = sectorAddr(s) + FLASH_LOG_SECTOR_SIZE;
    syncWritten = false;
    sessionSeq++;
    return true;
  }
  return false;
}

// Words left in the current sector
static uint32_t wordsLeft() {
  return curSector < 0 ? 0 : (sectorEnd - writeAddr) / FLASH_LOG_WORD_SIZE;
}

// True if `words` more words can be programmed, claiming at most one sector
static bool haveRoom(uint32_t words) {
  return words <= wordsLeft() ||
         (countFreeSectors() > 0 && words - wordsLeft() <= FLASH_LOG_DATA_WORDS);
}

static bool appendWord(uint8_t stream, const uint8_t *payload, uint8_t len) {
  if (curSector < 0 || writeAddr >= sectorEnd) {
    if (!claimSector()) {
      return false;
    }
  }

  if (stream == FLASH_LOG_STREAM_DATA && !syncWritten) {
    // First data in this sector. If the previous sector is gone (pad window)
    // or damaged, the extractor starts decoding at this offset.
    uint32_t firstRecord = recordStart >= dataWritten ? recordStart : recordEnd;
    FlashRecord sync = makeRecord(RECORD_SECTOR_SYNC, sessionId, curSector,
                                  sectors[curSector].seq, firstRecord - dataWritten);
    programWord(sectorAddr(curSector) + 2 * FLASH_LOG_WORD_SIZE, &sync);
    syncWritten = true;
  }

  uint32_t word[8] = {};
  uint8_t *bytes = (uint8_t *)word;
  bytes[0] = stream;
  bytes[1] = len;
  memcpy(bytes + 2, payload, len);

  bool ok = programWord(writeAddr, word);
  writeAddr += FLASH_LOG_WORD_SIZE;
  if (stream == FLASH_LOG_STREAM_DATA) {
    dataWritten += len;
  }
  return ok;
}

static void stageBytes(uint8_t stream, const uint8_t *data, size_t len) {
  Staging &st = staging[stream];
  while (len > 0) {
    size_t n = min(len, (size_t)(FLASH_LOG_PAYLOAD_PER_WORD - st.len));
    memcpy(st.bytes + st.len, data, n);
    st.len += n;
    data += n;
    len -= n;
    if (stream == FLASH_LOG_STREAM_DATA) {
      dataStaged += n;
    }

    if (st.len == FLASH_LOG_PAYLOAD_PER_WORD) {
      appendWord(stream, st.bytes, st.len);
      st.len = 0;
    }
  }
}

static uint8_t queueByte(size_t offset) {
  return queue[(queueHead + offset) % FLASH_LOG_BUFFER_SIZE];
}

static void drainQueue(size_t budget) {
  wantSector = false;

  while (queueSize > 0 && budget > 0) {
    uint8_t stream = queueByte(0);
    size_t len = queueByte(1) | (queueByte(2) << 8);

    if (!haveRoom((staging[stream].len + len) / FLASH_LOG_PAYLOAD_PER_WORD)) {
      wantSector = true;
      return;
    }

    queueHead = (queueHead + FLASH_LOG_QUEUE_HEADER) % FLASH_LOG_BUFFER_SIZE;
    queueSize -= FLASH_LOG_QUEUE_HEADER + len;
    if (stream == FLASH_LOG_STREAM_DATA) {
      recordStart = dataStaged;
      recordEnd = dataStaged + len;
    }
    while (len > 0) {
      size_t n = min(len, FLASH_LOG_BUFFER_SIZE - queueHead);
      stageBytes(stream, queue + queueHead, n);
      queueHead = (queueHead + n) % FLASH_LOG_BUFFER_SIZE;
      len -= n;
      budget -= min(budget, n);
    }
  }

  if (queueSize == 0 && pendingFlush) {
    uint8_t words = (staging[FLASH_LOG_STREAM_DATA].len > 0) + (staging[FLASH_LOG_STREAM_TEXT].len > 0);
    if (!haveRoom(words)) {
      wantSector = true;
      return;
    }
    for (uint8_t stream = FLASH_LOG_STREAM_DATA; stream <= FLASH_LOG_STREAM_TEXT; stream++) {
      if (staging[stream].len > 0) {
        appendWord(stream, staging[stream].bytes, staging[stream].len);
        staging[stream].len = 0;
      }
    }
    pendingFlush = false;
  }
}

static void writePending() {
  if (pendingProtect) {
    pendingProtect = false;
    appendTable(makeRecord(RECORD_SESSION_PROTECT, sessionId, 0, 0, 0));
    for (uint8_t s = 1; s < FLASH_LOG_NUM_SECTORS; s++) {
      if (sectors[s].session == sessionId) {
        sectors[s].isProtected = true;
        FlashRecord protect = makeRecord(RECORD_SECTOR_PROTECT, sessionId, s, sectors[s].seq, 0);
        programWord(sectorAddr(s) + FLASH_LOG_WORD_SIZE, &protect);
      }
    }
  }
  if (pendingLanded) {
    pendingLanded = false;
    appendTable(makeRecord(RECORD_SESSION_LANDED, sessionId, 0, 0, 0));
  }
}

// Oldest sector (lowest seq) of a session, never the one being written
static int oldestSectorOf(uint32_t session) {
  int oldest = -1;
  for (uint8_t s = 1; s < FLASH_LOG_NUM_SECTORS; s++) {
    if (sectors[s].session == session && s != curSector &&
        (oldest < 0 || sectors[s].seq < sectors[oldest].seq)) {
      oldest = s;
    }
  }
  return oldest;
}

static int pickEraseTarget() {
  for (uint8_t s = 1; s < FLASH_LOG_NUM_SECTORS; s++) {
    if (sectors[s].session == SESSION_DIRTY) {
      return s;
    }
  }

  uint8_t freeSectors = countFreeSectors();
  bool stuck = wantSector && freeSectors == 0;

  // Pad window: roll our own oldest sector
  if (!sessionProtected) {
    uint8_t own = 0;
    for (uint8_t s = 1; s < FLASH_LOG_NUM_SECTORS; s++) {
      own += sectors[s].session == sessionId;
    }
    int oldest = oldestSectorOf(sessionId);
    if (oldest >= 0 && (own > FLASH_LOG_PRELAUNCH_SECTORS || stuck)) {
      return oldest;
    }
  }

  // Old sessions that never launched
  if (freeSectors < FLASH_LOG_MIN_FREE_SECTORS || stuck) {
    uint32_t oldestSession = UINT32_MAX;
    for (uint8_t s = 1; s < FLASH_LOG_NUM_SECTORS; s++) {
      if (isSession(sectors[s].session) && sectors[s].session != sessionId &&
          !sectors[s].isProtected && sectors[s].session < oldestSession) {
        oldestSession = sectors[s].session;
      }
    }
    if (oldestSession != UINT32_MAX) {
      return oldestSectorOf(oldestSession);
    }
  }
  return -1;
}

bool flashLogBegin(bool format) {
  sessionId = 0;
  sessionSeq = 0;
  sessionFull = false;
  curSector = -1;
  pendingProtect = pendingLanded = pendingFlush = wantSector = false;
  erasingSector = -1;
  droppedBytes = 0;
  memset(staging, 0, sizeof(staging));
  dataWritten = dataStaged = recordStart = recordEnd = 0;
  queueHead = queueSize = 0;
  HAL_FLASH_Unlock();

  bool tableLoaded = loadTable();
  bool tableOk;
  if (format) {
    for (uint8_t s = 1; s < FLASH_LOG_NUM_SECTORS; s++) {
      if (!sectorIsBlank(s)) {
        eraseSectorBlocking(s);
      }
    }
    memset(sectors, 0, sizeof(sectors));
    latestSession = 0;
    latestProtected = latestLanded = false;
    tableOk = writeTable();
  } else if (!tableLoaded) {
    tableOk = rebuildTable();
  } else if (tableSlot >= FLASH_LOG_COMPACT_AT && !(latestProtected && !latestLanded)) {
    // Compacting blocks for an erase, so never do it if we may be mid flight
    tableOk = writeTable();
  } else {
    tableOk = true;
  }
  if (!tableOk) {
    return false;
  }

  for (uint8_t s = 1; s < FLASH_LOG_NUM_SECTORS; s++) {
    if (sectors[s].session == 0 && !sectorIsBlank(s)) {
      // Free sectors have to really be blank before they are handed out
      sectors[s].session = SESSION_DIRTY;
    } else if (isSession(sectors[s].session) && !sectors[s].isProtected && sectorIsBlank(s)) {
      // Power was cut between an erase finishing and its release record
      appendTable(makeRecord(RECORD_SECTOR_RELEASE, sectors[s].session, s, 0, 0));
      sectors[s] = {};
    }
  }

  // The last session launched and never landed, so we rebooted mid flight
  bool inFlight = latestProtected && !latestLanded;

  sessionId = nextSessionId++;
  sessionProtected = inFlight;
  latestSession = sessionId;
  latestProtected = inFlight;
  latestLanded = false;
  if (!appendTable(makeRecord(RECORD_SESSION_START, sessionId, 0, 0, 0))) {
    sessionId = 0;
    return false;
  }
  if (inFlight) {
    appendTable(makeRecord(RECORD_SESSION_PROTECT, sessionId, 0, 0, 0));
  }
  // May find nothing free yet, flashLogLoop claims once an erase finishes
  claimSector();
  return true;
}

void flashLogLoop() {
  if (sessionId == 0) {
    return;
  }

  pollErase();
  if (erasingSector >= 0) {
    return;
  }

  writePending();
  if (sessionFull) {
    return;
  }
  drainQueue(FLASH_LOG_DRAIN_BYTES_PER_LOOP);

  // Start erases with the queue mostly empty, it has to hold everything until they finish
  if (wantSector || queueSize < FLASH_LOG_BUFFER_SIZE / 4) {
    int target = pickEraseTarget();
    if (target >= 0) {
      startErase(target);
    } else if (wantSector && countFreeSectors() == 0) {
      sessionFull = true;
      droppedBytes += queueSize;
      queueSize = 0;
    }
  }
}

bool flashLogWrite(FlashLogStream stream, const uint8_t *data, size_t len) {
  if (sessionId == 0 || len == 0) {
    return false;
  }
  if ((stream != FLASH_LOG_STREAM_DATA && stream != FLASH_LOG_STREAM_TEXT) ||
      sessionFull || len > FLASH_LOG_MAX_RECORD ||
      FLASH_LOG_BUFFER_SIZE - queueSize < FLASH_LOG_QUEUE_HEADER + len) {
    droppedBytes += len;
    return false;
  }

  uint8_t header[FLASH_LOG_QUEUE_HEADER] = {stream, (uint8_t)len, (uint8_t)(len >> 8)};
  size_t tail = (queueHead + queueSize) % FLASH_LOG_BUFFER_SIZE;
  for (size_t i = 0; i < FLASH_LOG_QUEUE_HEADER; i++) {
    queue[(tail + i) % FLASH_LOG_BUFFER_SIZE] = header[i];
  }
  tail = (tail + FLASH_LOG_QUEUE_HEADER) % FLASH_LOG_BUFFER_SIZE;
  size_t first = min(len, FLASH_LOG_BUFFER_SIZE - tail);
  memcpy(queue + tail, data, first);
  memcpy(queue, data + first, len - first);
  queueSize += FLASH_LOG_QUEUE_HEADER + len;
  return true;
}

void flashLogFlush() { pendingFlush = true; }

void flashLogProtectSession() {
  if (sessionId == 0 || sessionProtected) {
    return;
  }
  sessionProtected = true;
  latestProtected = true;
  pendingProtect = true;
}

void flashLogMarkLanded() {
  if (sessionId == 0 || latestLanded) {
    return;
  }
  latestLanded = true;
  pendingLanded = true;
}

bool flashLogIsProtected() { return sessionProtected; }

bool flashLogIsFull() { return sessionFull; }

uint32_t flashLogSessionId() { return sessionId; }

uint32_t flashLogDroppedBytes() { return droppedBytes; }

size_t flashLogBytesFree() {
  if (sessionId == 0 || sessionFull) {
    return 0;
  }
  size_t words = wordsLeft() + countFreeSectors() * FLASH_LOG_DATA_WORDS;
  return words * FLASH_LOG_PAYLOAD_PER_WORD;
}

void flashLogPrintTable(Print &out) {
  out.printf("Flash log: session %lu%s, table %lu/%lu slots, %u KB free, %lu B dropped\n",
             sessionId, sessionProtected ? " (protected)" : "", tableSlot,
             (uint32_t)FLASH_LOG_WORDS_PER_SECTOR, (unsigned)(flashLogBytesFree() / 1024),
             droppedBytes);
  for (uint8_t s = 1; s < FLASH_LOG_NUM_SECTORS; s++) {
    const SectorState &sector = sectors[s];
    if (sector.session == 0) {
      out.printf("  sector %u: free\n", s);
    } else if (sector.session == SESSION_DIRTY) {
      out.printf("  sector %u: not blank, will be erased\n", s);
    } else if (sector.session == SESSION_UNUSABLE) {
      out.printf("  sector %u: erase failed, unusable\n", s);
    } else {
      out.printf("  sector %u: session %lu part %u%s%s\n", s, sector.session,
                 sector.seq, sector.isProtected ? " (protected)" : "",
                 s == curSector ? " (writing)" : "");
    }
  }
}

size_t FlashTextLog::write(uint8_t c) {
  line[lineLen++] = c;
  if (c == '\n' || lineLen == sizeof(line)) {
    flashLogWrite(FLASH_LOG_STREAM_TEXT, line, lineLen);
    lineLen = 0;
  }
  return 1;
}
