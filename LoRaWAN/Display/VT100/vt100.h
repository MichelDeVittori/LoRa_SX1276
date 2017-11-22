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
#ifndef __VT100_H__
#define __VT100_H__

#include <stdbool.h>
#include <stdint.h>

#ifndef STRING_STACK_LIMIT
#define STRING_STACK_LIMIT    120
#endif

/**
 * Implements VT100 terminal commands support.
 * Implments also the same behaviour has RawSerial class. The only difference
 * is located in putc fucntion where writeable check is made befor sending the character.
 */

enum VT100_TextAttributes
{
    ATTR_OFF      = 0,
    BOLD          = 1,
    USCORE        = 4,
    BLINK         = 5,
    REVERSE       = 7,
    BOLD_OFF      = 21,
    USCORE_OFF    = 24,
    BLINK_OFF     = 25,
    REVERSE_OFF   = 27,
};

enum VT100_Colors
{
    BLACK   = 0,
    RED     = 1,
    GREEN   = 2,
    BROWN   = 3,
    BLUE    = 4,
    MAGENTA = 5,
    CYAN    = 6,
    WHITE   = 7,
};

void VT100_init(void);

void VT100_ClearScreen( uint8_t param );

void VT100_ClearLine( uint8_t param );

void VT100_SetAttribute( uint8_t attr );

void VT100_SetAttribute2( uint8_t attr, uint8_t fgcolor, uint8_t bgcolor );

void VT100_SetCursorMode( uint8_t visible );

void VT100_SetCursorPos( uint8_t line, uint8_t col );

void VT100_PutStringAt( uint8_t line, uint8_t col, const char *s );

void VT100_PutCharAt( uint8_t line, uint8_t col, uint8_t c );

void VT100_PutHexAt( uint8_t line, uint8_t col, uint16_t n );

void VT100_PutBoxDrawingChar( uint8_t c );

bool VT100_Readable( void );

uint8_t VT100_GetChar( void );
/*
 * RawSerial class implmentation copy.
 */
/** Read a char from the serial port
 *
 * @returns The char read from the serial port
 */
int VT100_getc( );

/** Write a char to the serial port
 *
 * @param c The char to write
 *
 * @returns The written char or -1 if an error occured
 */
int VT100_putc( int c );


/** Write a string to the serial port
 *
 * @param str The string to write
 *
 * @returns 0 if the write succeeds, EOF for error
 */
int VT100_puts( const char *str );

// Experimental support for printf in RawSerial. No Stream inheritance
// means we can't call printf() directly, so we use sprintf() instead.
// We only call malloc() for the sprintf() buffer if the buffer
// length is above a certain threshold, otherwise we use just the stack.
int VT100_printf( const char *format, ... );


#endif // __VT100_H__
