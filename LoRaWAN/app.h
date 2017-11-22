#ifndef _APP_H
#define _APP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

#define ID_LED_1 0
#define ID_LED_2 1
#define ID_LED_3 2

#define LED_OFF     0 
#define LED_ON      1 
#define LED_TOGGLE  2 

typedef enum
{
	/* Application's state machine's initial state. */
	APP_STATE_INIT=0,
	APP_STATE_IDLE,
    APP_STATE_EXECUTE_CMD

	/* TODO: Define states used by the application state machine. */

} APP_STATES;

typedef struct
{
    /* The application's current state */
    APP_STATES state;

    /* TODO: Define any additional data used by the application. */

} APP_DATA;

void APP_Initialize ( void );

void APP_Tasks( void );

#endif