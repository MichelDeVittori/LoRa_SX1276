/* Include files **************************************************************/
#include "system/clk/sys_clk.h"

/* Include my files */
#include "Delay.h"

/* Functions ******************************************************************/
void DelayUs(uint32_t us)
{
    double cyclesRequiredForEntireDelay ;

    // We want to pre-calculate number of cycles required to delay 1ms, using a 1 cycle granule.
    cyclesRequiredForEntireDelay = (SYS_CLK_SystemFrequencyGet()/4000000.0) * us;
    
    while (cyclesRequiredForEntireDelay > 0) cyclesRequiredForEntireDelay -= 2.27;
}

void DelayMs(uint32_t ms)
{
    while (ms-- > 0) DelayUs(1000);
}

void Delay(uint32_t s)
{
    while(s--) DelayMs(1000);
}