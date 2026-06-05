#include <Arduino.h>
// #include "LoRaE22/RadioConfigDatatypes.h"

// constants for both of them
#define FREQUENCY 223.625f

#define ENCRYPTIONKEY 0x0000 // can't encrypt on a ham band (except we kinda can??)
#define PARITYCONFIG RadioConfigTypes::ParityConfig::Parity_8N1
#define SERIALSPEED RadioConfigTypes::SerialSpeeds::BAUD_115200
#define AIRDATARATE RadioConfigTypes::AirDataRate::RATE_15600
#define PACKETSIZE RadioConfigTypes::PacketSize::SIZE_240
#define WORMODE RadioConfigTypes::WakeOnReceiveMode::NormalWOR
#define WORPERIOD RadioConfigTypes::WakeOnReceiveListenPeriod::TIME_500
#define RELAYMODE RadioConfigTypes::RelayMode::RelayDisabled
#define DESTINATIONMODE RadioConfigTypes::Destination::Broadcast


#define ADDRESS 0xFFFF
#define NETWORKID 0x01
#define DESTINATION 0xFFFF02

#ifdef TX_SIDE
    #define RSSIREADINGS RadioConfigTypes::EnableRSSIReadings::Disabled
    #define AMBIENTRSSI RadioConfigTypes::EnableRSSIReadings::Disabled
#else
    #define RSSIREADINGS RadioConfigTypes::EnableRSSIReadings::Disabled
    #define AMBIENTRSSI RadioConfigTypes::EnableRSSIReadings::Enabled
#endif

#define LISTENBEFORETX RadioConfigTypes::EnableListenBeforeTX::LBTDisabled
#define TXPOWER RadioConfigTypes::TransmitPower::dBm33
