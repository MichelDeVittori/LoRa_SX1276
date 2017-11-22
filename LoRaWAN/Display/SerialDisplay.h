/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: VT100 serial display management

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __SERIAL_DISPLAY_H__
#define __SERIAL_DISPLAY_H__

#include <stdbool.h>
#include <stdint.h>

void SerialDisplayInit( void );
void SerialDisplayUpdateUplink( bool acked, uint8_t datarate, uint16_t counter, uint8_t port, uint8_t *buffer, uint8_t bufferSize );
void SerialDisplayUpdateDownlink( bool rxData, int16_t rssi, int8_t snr, uint16_t counter, uint8_t port, uint8_t *buffer, uint8_t bufferSize );
void SerialDisplayPrintCheckBox( bool activated );
void SerialDisplayUpdateLedState( uint8_t id, uint8_t state );
void SerialDisplayUpdateActivationMode( bool otaa );
void SerialDisplayUpdateEui( uint8_t line, uint8_t *eui );
void SerialDisplayUpdateKey( uint8_t line, uint8_t *key );
void SerialDisplayUpdateNwkId( uint8_t id );
void SerialDisplayUpdateDevAddr( uint32_t addr );
void SerialDisplayUpdateFrameType( bool confirmed );
void SerialDisplayUpdateAdr( bool adr );
void SerialDisplayUpdateDutyCycle( bool dutyCycle );
void SerialDisplayUpdatePublicNetwork( bool network );
void SerialDisplayUpdateData( uint8_t line, uint8_t *buffer, uint8_t size );
void SerialDisplayUpdateNetworkIsJoined( bool state );
void SerialDisplayUpdateUplinkAcked( bool state );
void SerialDisplayUpdateDonwlinkRxData( bool state );
bool SerialDisplayReadable( void );
uint8_t SerialDisplayGetChar( void );

#endif // __SERIAL_DISPLAY_H__
