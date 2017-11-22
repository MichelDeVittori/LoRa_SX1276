/* Include files **************************************************************/
#include <stdint.h>
#include "xc.h"

/* Include my files ***********************************************************/
#include "IO.h"
#include "peripheral/ports/plib_ports.h"

/* Functions ******************************************************************/
/* Hardware dependent */
/* Defines pinout
 * A -> 0x0----
 * B -> 0x1----
 * C -> 0x2----
 * D -> 0x3----
 * E -> 0x4----
 * F -> 0x5----
 * G -> 0x6----*/
void pin_Set(uint32_t port_pin, uint8_t state)
{         
    uint16_t port_sel = (port_pin>>16), pin = port_pin;
    
    if (port_sel == 1) {
             if (state == PIN_LOW)  PORTBCLR = pin;
        else if (state == PIN_HIGH) PORTBSET = pin;
        else                        PORTBINV = pin;
    }
    else if (port_sel == 2) {
             if (state == PIN_LOW)  PORTCCLR = pin;
        else if (state == PIN_HIGH) PORTCSET = pin;
        else                        PORTCINV = pin;
    }        
    else if (port_sel == 3) {
             if (state == PIN_LOW)  PORTDCLR = pin;
        else if (state == PIN_HIGH) PORTDSET = pin;
        else                        PORTDINV = pin;
    }    
    else if (port_sel == 4) {
             if (state == PIN_LOW)  PORTECLR = pin;
        else if (state == PIN_HIGH) PORTESET = pin;
        else                        PORTEINV = pin;
    } 
    else if (port_sel == 5) {
             if (state == PIN_LOW)  PORTFCLR = pin;
        else if (state == PIN_HIGH) PORTFSET = pin;
        else                        PORTFINV = pin;
    } 
    else if (port_sel == 6) {
             if (state == PIN_LOW)  PORTGCLR = pin;
        else if (state == PIN_HIGH) PORTGSET = pin;
        else                        PORTGINV = pin;
    } 
}

/* Hardware dependent */
uint8_t pin_Get(uint32_t port_pin)
{
    uint16_t port_sel = port_pin>>16, pin = port_pin;
    
         if (port_sel == 1) return ((PORTB&pin) ? 1 : 0);
    else if (port_sel == 2) return ((PORTC&pin) ? 1 : 0);
    else if (port_sel == 3) return ((PORTD&pin) ? 1 : 0);
    else if (port_sel == 4) return ((PORTE&pin) ? 1 : 0);
    else if (port_sel == 5) return ((PORTF&pin) ? 1 : 0);
    else if (port_sel == 6) return ((PORTG&pin) ? 1 : 0);              
            
    return 0;
}

/* Hardware dependent */
void pin_Init(void)
{
    //Init I/O
    PORTESET = LED_1 | LED_2 | LED_3;
    
    PORTESET = TDC_CS1;
    PORTGSET = TDC_CS2 | TDC_CS3;
    
    PORTGCLR = D0;
    PORTBCLR = D1 | D2;
    
    PORTBCLR = GPS_RST;
    
    PORTFCLR = TDC_EN;

    PORTCSET = FT232_RST; /* FT232 Enable*/
    
    PORTDSET = RFM95W_CS;
    PORTECLR = RFM95W_RST;
}