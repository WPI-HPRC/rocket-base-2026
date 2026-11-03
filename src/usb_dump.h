#pragma once

#include "Context.h"

// Prints every new sensor reading over USB serial as a CSV row. Only active
// when built with -D USB_SENSOR_DUMP (the marsv21_usb_dump env), otherwise a
// no-op. Format:
//   #HDR,<SENSOR>,time_ms,<field>,<field>,...   (every USB_DUMP_HEADER_INTERVAL_MS)
//   $<SENSOR>,<time_ms>,<value>,<value>,...
// Data rows start with '$' so they can be split out from regular log output,
// e.g. `grep '^\$ASM330,' dump.txt`
void usbDumpLoop(Context *ctx);
