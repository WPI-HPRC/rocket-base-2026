#!/usr/bin/env python3
"""Pull sessions out of a dump of the on-chip flash log (STM32H753 bank 2).

Dump bank 2 with the board in DFU mode (hold BOOT while plugging in USB, so
the flight firmware doesn't run and start a new session):
    dfu-util -a 0 -s 0x08100000:1048576 -U flashlog.bin
or with an ST-Link:
    st-flash read flashlog.bin 0x08100000 0x100000

Then:
    python3 scripts/flashlog_extract.py flashlog.bin [out_dir]

Each session is written to out_dir/sessionN/ as flightData.fb.bin (feed it to
the flatbuffer_decoder env) and debugLog.txt. The layout is described in
src/flash_log.h. Sessions are found from the sector headers, so this works
even if the table sector is damaged.

Also written to out_dir:
    table.txt        every record in the allocation table, plus sector owners
    sectorN.bin      raw copy of any sector that isn't blank but has no header
    reset_table.bin  a fresh table (one FORMAT record) that continues the
                     session numbering. scripts/extract_flashlog.sh writes it
                     to the board to free everything after a dump.
"""

import os
import struct
import sys
import zlib

SECTOR_SIZE = 128 * 1024
NUM_SECTORS = 8
WORD = 32
WORDS_PER_SECTOR = SECTOR_SIZE // WORD
HEADER_WORDS = 3
PAYLOAD_PER_WORD = WORD - 2
MAGIC = 0x474F4C46

RECORD = struct.Struct("<IBBHII12xI")
RECORD_NAMES = {
    1: "FORMAT",
    2: "SESSION_START",
    3: "SECTOR_CLAIM",
    4: "SESSION_PROTECT",
    5: "SESSION_LANDED",
    6: "SECTOR_HEADER",
    7: "SECTOR_PROTECT",
    8: "SECTOR_SYNC",
    9: "SECTOR_RELEASE",
}
STREAM_DATA = 1
STREAM_TEXT = 2
BLANK = b"\xff" * WORD


def parse_record(word):
    if word == BLANK:
        return None
    magic, rtype, sector, seq, session, arg, crc = RECORD.unpack(word)
    if magic != MAGIC or crc != zlib.crc32(word[:28]):
        return "invalid"
    return {"type": rtype, "sector": sector, "seq": seq, "session": session, "arg": arg}


def word_at(image, sector, index):
    offset = sector * SECTOR_SIZE + index * WORD
    return image[offset:offset + WORD]


def write_table(image, path):
    """Writes every table record to path. Returns the highest session id seen."""
    first = parse_record(word_at(image, 0, 0))
    if not isinstance(first, dict) or first["type"] != 1:
        print("Table: missing or corrupt (firmware will rebuild it from sector headers)")
    used = invalid = 0
    max_session = 0
    with open(path, "w") as f:
        f.write("slot  record           session  sector  seq  arg\n")
        for slot in range(WORDS_PER_SECTOR):
            rec = parse_record(word_at(image, 0, slot))
            if rec is None:
                break
            used += 1
            if rec == "invalid":
                invalid += 1
                f.write(f"{slot:4}  (unreadable)\n")
                continue
            if rec["type"] == 1:
                max_session = max(max_session, rec["arg"] - 1)
            else:
                max_session = max(max_session, rec["session"])
            name = RECORD_NAMES.get(rec["type"], f"type {rec['type']}")
            f.write(f"{slot:4}  {name:16} {rec['session']:8}  {rec['sector']:6}  "
                    f"{rec['seq']:3}  {rec['arg']}\n")
    print(f"Table: {used}/{WORDS_PER_SECTOR} slots used, {invalid} unreadable -> {path}")
    return max_session


def table_owners(image):
    """Replays the table into {sector: session}, or None if the table is unusable."""
    first = parse_record(word_at(image, 0, 0))
    if not isinstance(first, dict) or first["type"] != 1:
        return None
    owners = {}
    for slot in range(1, WORDS_PER_SECTOR):
        rec = parse_record(word_at(image, 0, slot))
        if rec is None:
            break
        if rec == "invalid":
            continue
        if rec["type"] == 3:
            owners[rec["sector"]] = rec["session"]
        elif rec["type"] == 9 and owners.get(rec["sector"]) == rec["session"]:
            del owners[rec["sector"]]
    return owners


def make_format_record(next_session):
    body = RECORD.pack(MAGIC, 1, 0, 1, 0, next_session, 0)[:28]
    return body + struct.pack("<I", zlib.crc32(body))


def find_sectors(image, out_dir):
    sectors = {}
    for s in range(1, NUM_SECTORS):
        header = parse_record(word_at(image, s, 0))
        if not isinstance(header, dict) or header["type"] != 6:
            raw = image[s * SECTOR_SIZE:(s + 1) * SECTOR_SIZE]
            if raw != b"\xff" * SECTOR_SIZE:
                path = os.path.join(out_dir, f"sector{s}.bin")
                with open(path, "wb") as f:
                    f.write(raw)
                print(f"  sector {s}: not blank but has no valid header, raw copy -> {path}")
            continue
        protect = parse_record(word_at(image, s, 1))
        protected = (isinstance(protect, dict) and protect["type"] == 7
                     and protect["session"] == header["session"])
        sync = parse_record(word_at(image, s, 2))
        sync_offset = sync["arg"] if isinstance(sync, dict) and sync["type"] == 8 else None
        sectors[s] = (header["session"], header["seq"], protected, sync_offset)
    return sectors


def read_sector_streams(image, sector, sync_offset, aligned, streams):
    """Appends one sector's payloads. Data is only kept while aligned to whole
    records: if the previous sector is missing or damaged, data starts at the
    sector's sync offset. Returns (still aligned, unreadable word count)."""
    bad = 0
    skip = 0 if aligned else sync_offset
    for i in range(HEADER_WORDS, WORDS_PER_SECTOR):
        word = word_at(image, sector, i)
        if word == BLANK:
            break
        stream, length = word[0], word[1]
        if stream not in streams or not 0 < length <= PAYLOAD_PER_WORD:
            bad += 1
            skip = None  # lost track of data records until the next sync
            continue
        payload = word[2:2 + length]
        if stream == STREAM_DATA:
            if skip is None:
                continue
            if skip > 0:
                cut = min(skip, len(payload))
                payload = payload[cut:]
                skip -= cut
        streams[stream] += payload
    return skip is not None, bad


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    with open(sys.argv[1], "rb") as f:
        image = f.read()
    out_dir = sys.argv[2] if len(sys.argv) > 2 else "flashlog"

    if len(image) < NUM_SECTORS * SECTOR_SIZE:
        print(f"Dump is {len(image)} bytes, expected {NUM_SECTORS * SECTOR_SIZE}")
        return 1

    os.makedirs(out_dir, exist_ok=True)
    table_path = os.path.join(out_dir, "table.txt")
    max_session = write_table(image, table_path)
    sectors = find_sectors(image, out_dir)
    max_session = max([max_session] + [session for session, _, _, _ in sectors.values()])

    # A valid table is the truth: data sectors it doesn't assign were already
    # dumped and cleared (or released) and are just waiting for the firmware
    # to erase them. Without a table, fall back to the sector headers.
    owners = table_owners(image)
    stale = []
    if owners is not None:
        stale = [s for s, (session, *_) in sectors.items() if owners.get(s) != session]
        for s in stale:
            del sectors[s]
        if stale:
            print(f"Skipping sectors {stale}: not in the table (already cleared, "
                  f"erased by the firmware on its next boot)")

    with open(table_path, "a") as f:
        f.write("\nsector  owner (from sector headers)\n")
        for s in range(1, NUM_SECTORS):
            if s in stale:
                f.write(f"{s:6}  cleared, erased by the firmware on its next boot\n")
            elif s in sectors:
                session, seq, protected, _ = sectors[s]
                f.write(f"{s:6}  session {session} part {seq}{' protected' if protected else ''}\n")
            else:
                f.write(f"{s:6}  {'blank' if word_at(image, s, 0) == BLANK else 'no header'}\n")

    with open(os.path.join(out_dir, "reset_table.bin"), "wb") as f:
        f.write(make_format_record(max_session + 1))

    sessions = {}
    for s, (session, seq, protected, sync_offset) in sectors.items():
        sessions.setdefault(session, []).append((seq, s, protected, sync_offset))

    if not sessions:
        print("No sessions found")
        return 0

    for session in sorted(sessions):
        parts = sorted(sessions[session])
        streams = {STREAM_DATA: bytearray(), STREAM_TEXT: bytearray()}
        bad = 0
        aligned = False
        prev_seq = None
        for seq, s, _, sync_offset in parts:
            # Session start, or straight after the sector before it, is aligned
            aligned = seq == 0 or (aligned and prev_seq == seq - 1)
            aligned, sector_bad = read_sector_streams(image, s, sync_offset, aligned, streams)
            bad += sector_bad
            prev_seq = seq
        protected = any(p for _, _, p, _ in parts)

        path = os.path.join(out_dir, f"session{session}")
        os.makedirs(path, exist_ok=True)
        with open(os.path.join(path, "flightData.fb.bin"), "wb") as f:
            f.write(streams[STREAM_DATA])
        with open(os.path.join(path, "debugLog.txt"), "wb") as f:
            f.write(streams[STREAM_TEXT])

        sector_list = ",".join(f"{s}" for _, s, _, _ in parts)
        first_seq = parts[0][0]
        print(f"session {session}: sectors [{sector_list}]"
              f"{' protected' if protected else ''}"
              f"{f', pad data before part {first_seq} rolled off' if first_seq else ''}, "
              f"{len(streams[STREAM_DATA])} B data, {len(streams[STREAM_TEXT])} B text"
              f"{f', {bad} unreadable words' if bad else ''} -> {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
