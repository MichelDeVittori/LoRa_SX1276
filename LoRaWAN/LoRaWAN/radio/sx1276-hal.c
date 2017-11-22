/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C) 2014 Semtech
 
Description: -
 
License: Revised BSD License, see LICENSE.TXT file include in the project
 
Maintainers: Miguel Luis, Gregory Cristian and Nicolas Huguenin
*/

#include "sx1276-hal.h"
#include "sx1276/sx1276.h"
#include <stdbool.h>

#include "driver/spi/static/drv_spi_static.h"
#include "../../PCB/IO.h"
#include "../../Delay.h"

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,
    SX1276SetStby, 
    SX1276SetRx,
    SX1276StartCad,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength
};
   
void SX1276IoInit( void )
{
    /* Do nothing */
}
 
void SX1276IoDeInit( void )
{
    //nothing
}
 
uint8_t SX1276GetPaSelect( uint32_t channel )
{
    if( channel > RF_MID_BAND_THRESH )
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

void SX1276SetAntSwLowPower( bool status )
{
    if( status == false )
    {
        SX1276AntSwInit( );
    }
    else
    {
        SX1276AntSwDeInit( );
    }    
}
 
void SX1276AntSwInit( void )
{
    //antSwitch = 0;
}
 
void SX1276AntSwDeInit( void )
{
    //antSwitch = 0;
}
 
void SX1276SetAntSw( uint8_t rxTx )
{
 
    // 1: Tx, 0: Rx
    if( rxTx != 0 )
    {
        //antSwitch = 1;
    }
    else
    {
        //antSwitch = 0;
    }
}

 
bool SX1276CheckRfFrequency( uint32_t frequency )
{
    //TODO: Implement check, currently all frequencies are supported
    return true;
}

uint8_t SX1276spi(uint8_t data)
{
    while (!SPI1STATbits.SPITBE); /* Wait for tx buffer empty */
    SPI1BUF = data;
    while (SPI1STATbits.SPIBUSY);
    return SPI1BUF;
}

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    //NSS = 0;
    pin_Set(RFM95W_CS, PIN_LOW);
    
    SX1276spi(addr | 0x80);
    
    while( size-- ) SX1276spi(*buffer++);
    
    //NSS = 1;
    pin_Set(RFM95W_CS, PIN_HIGH);
}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    //NSS = 0;
    pin_Set(RFM95W_CS, PIN_LOW);

    SX1276spi(addr & 0x7F);
    
    while( size-- ) *buffer++ = SX1276spi(0x00);

    //NSS = 1;
    pin_Set(RFM95W_CS, PIN_HIGH);
}

void SX1276Reset( void )
{
    // Set RESET pin to 0
    pin_Set(RFM95W_RST, PIN_LOW);

    // Wait 1 ms
    DelayMs( 1 );

    // Set RESET pin to 1
    pin_Set(RFM95W_RST, PIN_HIGH);

    // Wait 6 ms
    DelayMs( 6 );        
}
