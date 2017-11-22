#ifndef IO_H
#define IO_H

/* Include files **************************************************************/
#include <stdint.h>

/* Include my files ***********************************************************/
#include "Pinout.h"

/* Define *********************************************************************/
#define PIN_LOW     1   /* Hardware dependent */
#define PIN_HIGH    2   /* Hardware dependent */
#define PIN_TOGGLE  3   /* Hardware dependent */

/* Prototipes *****************************************************************/
void    pin_Set(uint32_t port_pin, uint8_t state);    /* Hardware dependent */
uint8_t pin_Get(uint32_t port_pin);                   /* Hardware dependent */

void pin_Init(void); /* Hardware dependent */

#endif