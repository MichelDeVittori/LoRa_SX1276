/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Timer objects and scheduling management

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/

#include "driver/tmr/drv_tmr_static.h"
#include "timer.h"
#include "../USART.h"

volatile uint64_t CurrentTime = 0;

void TimerResetTimeCounter( void )
{
    CurrentTime = 0;    
    DRV_TMR0_CounterClear();
}

void TimerTimeCounterInit( void )
{
    TimerResetTimeCounter();    
    DRV_TMR0_Start();
}

TimerTime_t TimerGetCurrentTime( void )
{   
    //CurrentTime += DRV_TMR0_CounterValueGet();  
    //RV_TMR0_CounterClear();
    
    CurrentTime = DRV_TMR0_CounterValueGet()/1000; //Miglioria: settare clock TMR a 1Khz
    
    return ( ( TimerTime_t )CurrentTime );
}

TimerTime_t TimerGetElapsedTime( TimerTime_t savedTime )
{    
    return ( TimerTime_t )( TimerGetCurrentTime() - savedTime );
}

TimerTime_t TimerGetFutureTime( TimerTime_t eventInFuture )
{  
    return ( TimerTime_t )( TimerGetCurrentTime() + eventInFuture );
}

unsigned char ID = 0;
TimerEvent_t timers[NR_TIMERS];

void TimerInit( TimerEvent_t *obj, void ( *callback )( void ))
{        
    obj->Callback = callback;
    obj->run = 0;
    obj->TimerID = ID;
    
    timers[ID].Callback = callback;
    timers[ID].run = 0;
    timers[ID].TimerID = ID;
    
    ID++;
}

void TimerStart( TimerEvent_t *obj )
{
    obj->run = 1;
    timers[obj->TimerID].run = 1;
}

void TimerStop( TimerEvent_t *obj )
{
    obj->run = 0;
    timers[obj->TimerID].run = 0;
}

void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
    obj->t_delta = value;    
    obj->t_end   = value + TimerGetCurrentTime();
    
    timers[obj->TimerID].t_delta = obj->t_delta;
    timers[obj->TimerID].t_end   = obj->t_end;     
}

void TimerCheck(void)
{
    TimerGetCurrentTime();
    
    int i;
    for (i = 0; i < ID; i++) {
        
        if( timers[i].run ) {
            
            if ( timers[i].t_end <= CurrentTime ) { 
            
                timers[i].Callback();
                timers[i].t_end = TimerGetCurrentTime() + timers[i].t_delta;
            }
        }
    }
}