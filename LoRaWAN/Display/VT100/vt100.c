/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: VT100 terminal support class

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "vt100.h"

#include <stdbool.h>
#include <stdint.h>
#include "../../USART.h"

void VT100_init( void )
{
    // initializes terminal to "power-on" settings
    // ESC c
    SerialPrint( "\x1B\x63" );
}

void VT100_ClearScreen( uint8_t param )
{
    // ESC [ Ps J
    // 0    Clear screen from cursor down
    // 1    Clear screen from cursor up
    // 2    Clear entire screen 
    VT100_printf( "\x1B[%dJ", param );
}

void VT100_ClearLine( uint8_t param )
{
    // ESC [ Ps K
    // 0    Erase from the active position to the end of the line, inclusive (default)
    // 1    Erase from the start of the screen to the active position, inclusive
    // 2    Erase all of the line, inclusive
    VT100_printf( "\x1B[%dK", param );
}

void VT100_SetAttribute( uint8_t attr )
{
    // ESC [ Ps;...;Ps m
    VT100_printf( "\x1B[%dm", attr );
}

void VT100_SetAttribute2( uint8_t attr, uint8_t fgcolor, uint8_t bgcolor )
{
    // ESC [ Ps;...;Ps m
    VT100_printf( "\x1B[%d;%d;%dm", attr, fgcolor + 30, bgcolor + 40 );
}

void VT100_SetCursorMode( uint8_t visible )
{
    if( visible == true )
    {
        // ESC [ ? 25 h
        SerialPrint( "\x1B[?25h" );
    }
    else
    {
        // ESC [ ? 25 l
        SerialPrint( "\x1B[?25l" );
    }
}

void VT100_SetCursorPos( uint8_t line, uint8_t col )
{
    // ESC [ Pl ; Pc H
    VT100_printf( "\x1B[%d;%dH", line, col );
}

void VT100_PutStringAt( uint8_t line, uint8_t col, const char *s )
{
    VT100_SetCursorPos( line, col );
    VT100_printf( "%s", s );
}

void VT100_PutCharAt( uint8_t line, uint8_t col, uint8_t c )
{
    VT100_SetCursorPos( line, col );
    VT100_printf( "%c", c );
}

void VT100_PutHexAt( uint8_t line, uint8_t col, uint16_t n )
{
    VT100_SetCursorPos( line, col );
    VT100_printf( "%X", n );
}

void VT100_PutBoxDrawingChar( uint8_t c )
{
    VT100_printf( "\x1B(0%c\x1b(B", c );
}

bool VT100_Readable( void )
{
    return true;
}

uint8_t VT100_GetChar( void )
{
    return VT100_getc( );
}

/*
 * RawSerial class implmentation copy.
 */
/** Read a char from the serial port
 *
 * @returns The char read from the serial port
 */
int VT100_getc( )
{
    return 0;
}

/** Write a char to the serial port
 *
 * @param c The char to write
 *
 * @returns The written char or -1 if an error occured
 */
int VT100_putc( int c )
{
    char ch[] = {c, '\0'};
    SerialPrint(ch);    
    return 0;
}

/** Write a string to the serial port
 *
 * @param str The string to write
 *
 * @returns 0 if the write succeeds, EOF for error
 */
int VT100_puts( const char *str )
{
    while( *str ) VT100_putc( *str++ );
    return 0;
}

// Experimental support for printf in RawSerial. No Stream inheritance
// means we can't call printf() directly, so we use sprintf() instead.
// We only call malloc() for the sprintf() buffer if the buffer
// length is above a certain threshold, otherwise we use just the stack.
int VT100_printf( const char *format, ... )
{
    va_list arg;
    va_start( arg, format );
    int len = vsnprintf( NULL, 0, format, arg );
    
    if( len < STRING_STACK_LIMIT ) {
        
        char temp[STRING_STACK_LIMIT];
        vsprintf( temp, format, arg );
        VT100_puts( temp );
    }
    else {
        
        char temp[len + 1];
        vsprintf( temp, format, arg );
        VT100_puts( temp );
    }
    va_end( arg );
    return len;
}