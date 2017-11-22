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
#include "VT100/vt100.h"
#include "SerialDisplay.h"

#include <stdbool.h>
#include <stdint.h>

#include "../USART.h"

void SerialPrintCheckBox( bool activated, uint8_t color )
{
    if( activated == true )
    {
        VT100_SetAttribute2( ATTR_OFF, color, color );
    }
    else
    {
        VT100_SetAttribute( ATTR_OFF );
    }
    VT100_printf( " " );
    VT100_SetAttribute( ATTR_OFF );
}

void SerialDisplayUpdateActivationMode( bool otaa )
{
    VT100_SetCursorPos( 4, 17 );
    SerialPrintCheckBox( otaa, WHITE );
    VT100_SetCursorPos( 9, 17 );
    SerialPrintCheckBox( !otaa, WHITE );
}

void SerialDisplayUpdateEui( uint8_t line, uint8_t *eui )
{
    VT100_SetCursorPos( line, 27 );
    int8_t i;
    for( i = 0; i < 8; i++ )
    {
        VT100_printf( "%02X ", eui[i] );
    }
    VT100_SetCursorPos( line, 50 );
    VT100_printf( "]" );
}

void SerialDisplayUpdateKey( uint8_t line, uint8_t *key )
{
    VT100_SetCursorPos( line, 27 );
    int8_t i;
    for( i = 0; i < 16; i++ )
    {
        VT100_printf( "%02X ", key[i] );
    }
    VT100_SetCursorPos( line, 74 );
    VT100_printf( "]" );
}

void SerialDisplayUpdateNwkId( uint8_t id )
{
    VT100_SetCursorPos( 10, 27 );
    VT100_printf( "%03d", id );
}

void SerialDisplayUpdateDevAddr( uint32_t addr )
{
    VT100_SetCursorPos( 11, 27 );
    VT100_printf( "%02X %02X %02X %02X", ( addr >> 24 ) & 0xFF, ( addr >> 16 ) & 0xFF, ( addr >> 8 ) & 0xFF, addr & 0xFF );
}

void SerialDisplayUpdateFrameType( bool confirmed )
{
    VT100_SetCursorPos( 15, 17 );
    SerialPrintCheckBox( confirmed, WHITE );
    VT100_SetCursorPos( 15, 32 );
    SerialPrintCheckBox( !confirmed, WHITE );
}

void SerialDisplayUpdateAdr( bool adr )
{
    VT100_SetCursorPos( 16, 27 );
    if( adr == true )
    {
        VT100_printf( " ON" );
    }
    else
    {
        VT100_printf( "OFF" );
    }
}

void SerialDisplayUpdateDutyCycle( bool dutyCycle )
{
    VT100_SetCursorPos( 17, 27 );
    if( dutyCycle == true )
    {
        VT100_printf( " ON" );
    }
    else
    {
        VT100_printf( "OFF" );
    }
}

void SerialDisplayUpdatePublicNetwork( bool network )
{
    VT100_SetCursorPos( 19, 17 );
    SerialPrintCheckBox( network, WHITE );
    VT100_SetCursorPos( 19, 30 );
    SerialPrintCheckBox( !network, WHITE );
}

void SerialDisplayUpdateNetworkIsJoined( bool state )
{
    VT100_SetCursorPos( 20, 17 );
    SerialPrintCheckBox( !state, RED );
    VT100_SetCursorPos( 20, 30 );
    SerialPrintCheckBox( state, GREEN );
}

void SerialDisplayUpdateLedState( uint8_t id, uint8_t state )
{
    switch( id )
    {
        case 1:
            VT100_SetCursorPos( 22, 17 );
            SerialPrintCheckBox( state, RED );
            break;
        case 2:
            VT100_SetCursorPos( 22, 31 );
            SerialPrintCheckBox( state, GREEN );
            break;
        case 3:
            VT100_SetCursorPos( 22, 45 );
            SerialPrintCheckBox( state, CYAN );
            break;
        default:
            break;
    }
}

void SerialDisplayUpdateData( uint8_t line, uint8_t *buffer, uint8_t size )
{
    int8_t i;
    
    if( size != 0 )
    {
        VT100_SetCursorPos( line, 27 );
        for( i = 0; i < size; i++ )
        {
            VT100_printf( "%02X ", buffer[i] );
            if( ( ( i + 1 ) % 16 ) == 0 )
            {
                line++;
                VT100_SetCursorPos( line, 27 );
            }
        }
        for( i = size; i < 64; i++ )
        {
            VT100_printf( "__ " );
            if( ( ( i + 1 ) % 16 ) == 0 )
            {
                line++;
                VT100_SetCursorPos( line, 27 );
            }
        }
        VT100_SetCursorPos( line - 1, 74 );
        VT100_printf( "]" );
    }
    else
    {
        VT100_SetCursorPos( line, 27 );
        for( i = 0; i < 64; i++ )
        {
            VT100_printf( "__ " );
            if( ( ( i + 1 ) % 16 ) == 0 )
            {
                line++;
                VT100_SetCursorPos( line, 27 );
            }
        }
        VT100_SetCursorPos( line - 1, 74 );
        VT100_printf( "]" );
    }
}

void SerialDisplayUpdateUplinkAcked( bool state )
{
    VT100_SetCursorPos( 24, 36 );
    SerialPrintCheckBox( state, GREEN );
}

void SerialDisplayUpdateUplink( bool acked, uint8_t datarate, uint16_t counter, uint8_t port, uint8_t *buffer, uint8_t bufferSize )
{
    // Acked
    SerialDisplayUpdateUplinkAcked( acked );
    // Datarate
    VT100_SetCursorPos( 25, 33 );
    VT100_printf( "DR%d", datarate );
    // Counter
    VT100_SetCursorPos( 26, 27 );
    VT100_printf( "%10d", counter );
    // Port
    VT100_SetCursorPos( 27, 34 );
    VT100_printf( "%3d", port );
    // Data
    SerialDisplayUpdateData( 28, buffer, bufferSize );
    // Help message
    VT100_SetCursorPos( 42, 1 );
    //VT100_printf( "To refresh screen please hit 'r' key." );
}

void SerialDisplayUpdateDonwlinkRxData( bool state )
{
    VT100_SetCursorPos( 34, 4 );
    SerialPrintCheckBox( state, GREEN );
}

void SerialDisplayUpdateDownlink( bool rxData, int16_t rssi, int8_t snr, uint16_t counter, uint8_t port, uint8_t *buffer, uint8_t bufferSize )
{
    // Rx data
    SerialDisplayUpdateDonwlinkRxData( rxData );
    // RSSI
    VT100_SetCursorPos( 33, 32 );
    VT100_printf( "%5d", rssi );
    // SNR
    VT100_SetCursorPos( 34, 32 );
    VT100_printf( "%5d", snr );
    // Counter
    VT100_SetCursorPos( 35, 27 );
    VT100_printf( "%10d", counter );
    if( rxData == true )
    {
        // Port
        VT100_SetCursorPos( 36, 34 );
        VT100_printf( "%3d", port );
        // Data
        SerialDisplayUpdateData( 37, buffer, bufferSize );
    }
    else
    {
        // Port
        VT100_SetCursorPos( 36, 34 );
        VT100_printf( "   " );
        // Data
        SerialDisplayUpdateData( 37, NULL, 0 );
    }
}

void SerialDisplayDrawFirstLine( void )
{
    VT100_PutBoxDrawingChar( 'l' );
    int8_t i;
    for( i = 0; i <= 77; i++ )
    {
        VT100_PutBoxDrawingChar( 'q' );
    }
    VT100_PutBoxDrawingChar( 'k' );
    VT100_printf( "\r\n" );
}

void SerialDisplayDrawTitle( const char* title )
{
    VT100_PutBoxDrawingChar( 'x' );
    VT100_printf( "%s", title );
    VT100_PutBoxDrawingChar( 'x' );
    VT100_printf( "\r\n" );
}
void SerialDisplayDrawTopSeparator( void )
{
    VT100_PutBoxDrawingChar( 't' );
    int8_t i;
    for( i = 0; i <= 11; i++ )
    {
        VT100_PutBoxDrawingChar( 'q' );
    }
    VT100_PutBoxDrawingChar( 'w' );
    for( i = 0; i <= 64; i++ )
    {
        VT100_PutBoxDrawingChar( 'q' );
    }
    VT100_PutBoxDrawingChar( 'u' );
    VT100_printf( "\r\n" );
}

void SerialDisplayDrawColSeparator( void )
{
    VT100_PutBoxDrawingChar( 'x' );
    int8_t i;
    for( i = 0; i <= 11; i++ )
    {
        VT100_PutBoxDrawingChar( ' ' );
    }
    VT100_PutBoxDrawingChar( 't' );
    for( i = 0; i <= 64; i++ )
    {
        VT100_PutBoxDrawingChar( 'q' );
    }
    VT100_PutBoxDrawingChar( 'u' );
    VT100_printf( "\r\n" );
}

void SerialDisplayDrawSeparator( void )
{
    VT100_PutBoxDrawingChar( 't' );
    int8_t i;
    for( i = 0; i <= 11; i++ )
    {
        VT100_PutBoxDrawingChar( 'q' );
    }
    VT100_PutBoxDrawingChar( 'n' );
    for( i = 0; i <= 64; i++ )
    {
        VT100_PutBoxDrawingChar( 'q' );
    }
    VT100_PutBoxDrawingChar( 'u' );
    VT100_printf( "\r\n" );
}

void SerialDisplayDrawLine( const char* firstCol, const char* secondCol )
{
    VT100_PutBoxDrawingChar( 'x' );
    VT100_printf( "%s", firstCol );
    VT100_PutBoxDrawingChar( 'x' );
    VT100_printf( "%s", secondCol );
    VT100_PutBoxDrawingChar( 'x' );
    VT100_printf( "\r\n" );
}

void SerialDisplayDrawBottomLine( void )
{
    VT100_PutBoxDrawingChar( 'm' );
    int8_t i;
    for( i = 0; i <= 11; i++ )
    {
        VT100_PutBoxDrawingChar( 'q' );
    }
    VT100_PutBoxDrawingChar( 'v' );
    for( i = 0; i <= 64; i++ )
    {
        VT100_PutBoxDrawingChar( 'q' );
    }
    VT100_PutBoxDrawingChar( 'j' );
    VT100_printf( "\r\n" );
}

void SerialDisplayInit( void )
{
    VT100_ClearScreen( 2 );
    VT100_SetCursorMode( false );
    VT100_SetCursorPos( 0, 0 );
    
    SerialDisplayDrawFirstLine( );
    SerialDisplayDrawTitle( "                      LoRaWAN Test Application                                " );
    SerialDisplayDrawTopSeparator( );
    SerialDisplayDrawLine( " Activation ", " [ ]Over The Air                                                 " );
    SerialDisplayDrawLine( "            ", " DevEui    [__ __ __ __ __ __ __ __]                             " );
    SerialDisplayDrawLine( "            ", " AppEui    [__ __ __ __ __ __ __ __]                             " );
    SerialDisplayDrawLine( "            ", " AppKey    [__ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __]     " );
    SerialDisplayDrawColSeparator( );
    SerialDisplayDrawLine( "            ", " [ ]Personalisation                                              " );
    SerialDisplayDrawLine( "            ", " NwkId     [___]                                                 " );
    SerialDisplayDrawLine( "            ", " DevAddr   [__ __ __ __]                                         " );
    SerialDisplayDrawLine( "            ", " NwkSKey   [__ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __]     " );
    SerialDisplayDrawLine( "            ", " AppSKey   [__ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __]     " );
    SerialDisplayDrawSeparator( );
    SerialDisplayDrawLine( " MAC params ", " [ ]Confirmed / [ ]Unconfirmed                                   " );
    SerialDisplayDrawLine( "            ", " ADR       [   ]                                                 " );
    SerialDisplayDrawLine( "            ", " Duty cycle[   ]                                                 " );
    SerialDisplayDrawSeparator( );
    SerialDisplayDrawLine( " Network    ", " [ ]Public  / [ ]Private                                         " );
    SerialDisplayDrawLine( "            ", " [ ]Joining / [ ]Joined                                          " );
    SerialDisplayDrawSeparator( );
    SerialDisplayDrawLine( " LED status ", " [ ]LED1(Tx) / [ ]LED2(Rx) / [ ]LED3(Blink if running)           " );
    SerialDisplayDrawSeparator( );
    SerialDisplayDrawLine( " Uplink     ", " Acked              [ ]                                          " );
    SerialDisplayDrawLine( "            ", " Datarate        [    ]                                          " );
    SerialDisplayDrawLine( "            ", " Counter   [          ]                                          " );
    SerialDisplayDrawLine( "            ", " Port             [   ]                                          " );
    SerialDisplayDrawLine( "            ", " Data      [__ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __      " );
    SerialDisplayDrawLine( "            ", "            __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __      " );
    SerialDisplayDrawLine( "            ", "            __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __      " );
    SerialDisplayDrawLine( "            ", "            __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __      " );
    SerialDisplayDrawSeparator( );
    SerialDisplayDrawLine( " Downlink   ", " RSSI           [     ] dBm                                      " );
    SerialDisplayDrawLine( " [ ]Data    ", " SNR            [     ] dB                                       " );
    SerialDisplayDrawLine( "            ", " Counter   [          ]                                          " );
    SerialDisplayDrawLine( "            ", " Port             [   ]                                          " );
    SerialDisplayDrawLine( "            ", " Data      [__ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __      " );
    SerialDisplayDrawLine( "            ", "            __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __      " );
    SerialDisplayDrawLine( "            ", "            __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __      " );
    SerialDisplayDrawLine( "            ", "            __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __      " );
    SerialDisplayDrawBottomLine( );
    //VT100_printf( "To refresh screen please hit 'r' key.\r\n" );
}

bool SerialDisplayReadable( void )
{
    return VT100_Readable( );
}

uint8_t SerialDisplayGetChar( void )
{
    return VT100_GetChar( );
}
