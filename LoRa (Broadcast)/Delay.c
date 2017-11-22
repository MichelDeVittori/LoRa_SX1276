/* Include files **************************************************************/
#include "system/clk/sys_clk.h"

/* Include my files */
#include "Delay.h"

/* Functions ******************************************************************/
void delay_us(uint32_t us)
{
    double cyclesRequiredForEntireDelay ;

    // We want to pre-calculate number of cycles required to delay 1ms, using a 1 cycle granule.
    cyclesRequiredForEntireDelay = (SYS_CLK_SystemFrequencyGet()/4000000.0) * us;
    
    while (cyclesRequiredForEntireDelay > 0) cyclesRequiredForEntireDelay -= 2.27;
}

void delay_ms(uint32_t ms)
{
    while (ms-- > 0) delay_us(1000);
}

void delay_s(uint32_t s)
{
    while(s--) delay_ms(1000);
}

uint64_t millis(void)
{
    return ((seconds_since_start*1000) + ((((uint64_t)TMR3<<16) | TMR2)/3125.0));
}