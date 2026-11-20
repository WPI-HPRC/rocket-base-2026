#pragma once

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
