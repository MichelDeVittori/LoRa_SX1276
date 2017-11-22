/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech
 
Description: End device commissioning parameters
 
License: Revised BSD License, see LICENSE.TXT file include in the project
 
Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __LORA_COMMISSIONING_H__
#define __LORA_COMMISSIONING_H__
 
#define USE_BAND_868

#define TX_TIMEOUT  3000

/*!
 * When set to 1 the application uses the Over-the-Air activation procedure
 * When set to 0 the application uses the Personalization activation procedure
 */
#define OVER_THE_AIR_ACTIVATION                     1
 
/*!
 * Indicates if the end-device is to be connected to a private or public network
 */
#define LORAWAN_PUBLIC_NETWORK                      true
 
/*!
 * IEEE Organizationally Unique Identifier ( OUI ) (big endian)
 */
// #define IEEE_OUI                                    0x11, 0x22, 0x33
 
/*!
 * Mote device IEEE EUI (big endian)
 */
#define LORAWAN_DEVICE_EUI { 0xC0, 0xEE, 0x40, 0x00, 0x01, 0x01, 0x95, 0x01 } 
// C0-EE-40-00-01-01-95-01

/*!
 * Application IEEE EUI (big endian)
 */
#define LORAWAN_APPLICATION_EUI { 0xF0, 0x3D, 0x29, 0xAC, 0x71, 0x00, 0x00, 0x01 }   
// F0-3D-29-AC-71-00-00-01

/*!
 * AES encryption/decryption cipher application key
 */
#define LORAWAN_APPLICATION_KEY { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
// 00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00

/*!
 * Current network ID
 */
#define LORAWAN_NETWORK_ID                          ( uint32_t )0
 
/*!
 * Device address on the network (big endian)
 */
#define LORAWAN_DEVICE_ADDRESS                      ( uint32_t )0x12345678
 
/*!
 * AES encryption/decryption cipher network session key
 */
#define LORAWAN_NWKSKEY                             { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C }
 
/*!
 * AES encryption/decryption cipher application session key
 */
#define LORAWAN_APPSKEY                             { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C }
 
#endif // __LORA_COMMISSIONING_H__