#pragma once

// 0 = Flight mode, most logs disabled, almost everything logged to SD card
// 1 = Standard on the ground mode, logs to serial
// 2 = Extra debug info, sends trace logs to serial as well
// XXX: This must be set to 0 before launch!!
#define DEBUG_MODE 1

// If defined, replaces sensors with mock version
// XXX: This line must be commented out before launch (potentially also during testing)!!
// #define MOCK_SENSORS

#if defined(MOCK_SENSORS)
    #define ASM_DATA_FILENAME "imu_corr.csv"
    #define ASM_DATA_RATE 250
    
    // #define LSM_DATA_FILENAME ""
    // #define LSM_DATA_RATE 0

    #define LIS_DATA_FILENAME "mag_corr.csv"
    #define LIS_DATA_RATE 250
#endif

#define LOG_FOLDER_NAME "flightData"

// On-chip flash logging, see flash_log.h. 7 data sectors of 128 KB.
// Sectors to keep free for the current session; older sessions that never
// launched are erased oldest first to get there. 7 = always reclaim them.
#define FLASH_LOG_MIN_FREE_SECTORS 7
// Sectors a session keeps while on the pad, it erases its own oldest past this (2-6)
#define FLASH_LOG_PRELAUNCH_SECTORS 2
// RAM queue in front of the flash. Drained every loop, but sized to hold
// incoming data through a 1-2 s sector erase.
#define FLASH_LOG_BUFFER_SIZE 65536

// If defined, erases every flash log session (including flights) at boot
// XXX: This line must be commented out before launch!!
// #define FLASH_LOG_FORMAT_ON_BOOT
