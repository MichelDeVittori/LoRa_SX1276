/* Include files **************************************************************/
#include <stdint.h>
#include <string.h>

/* Include my files ***********************************************************/
#include "driver/usart/drv_usart_static.h"
#include "USART.h"
#include "PCB/Settings.h"

extern UsartQueue_t usart_tx_pc;

/* Functions ******************************************************************/
void usart_QueueWrite(UsartQueue_t *queue, uint8_t *buffer, uint16_t size)
{
    uint16_t i;
    
    for(i = 0; i < size; i++) {

        queue->buffer[queue->wr_pt++] = *(buffer+i);
        if (queue->wr_pt == USART_BUFFER_SIZE) queue->wr_pt = 0;
        
    }
}

void SerialPrint(uint8_t *buff)
{
    usart_QueueWrite(&usart_tx_pc, buff, strlen(buff));
    
    do { 
        while ((!DRV_USART0_TransmitBufferIsFull()) && (usart_tx_pc.rd_pt != usart_tx_pc.wr_pt)) {

                    DRV_USART0_WriteByte(usart_tx_pc.buffer[usart_tx_pc.rd_pt++]);
                    if (usart_tx_pc.rd_pt == USART_BUFFER_SIZE) usart_tx_pc.rd_pt = 0;
        }
    } while (usart_tx_pc.rd_pt != usart_tx_pc.wr_pt); 
}

void SerialPrintBuffer(UsartQueue_t *queue, uint8_t *buffer, uint16_t size)
{
    usart_QueueWrite(queue, buffer, size);
    
    do { 
        while ((!DRV_USART0_TransmitBufferIsFull()) && (usart_tx_pc.rd_pt != usart_tx_pc.wr_pt)) {

                    DRV_USART0_WriteByte(usart_tx_pc.buffer[usart_tx_pc.rd_pt++]);
                    if (usart_tx_pc.rd_pt == USART_BUFFER_SIZE) usart_tx_pc.rd_pt = 0;
        }
    } while (usart_tx_pc.rd_pt != usart_tx_pc.wr_pt); 
}

void SerialPrintNr(uint32_t buff, uint8_t base)
{
    uint8_t tmp[32];
    
    if (base == DEC) sprintf(tmp, "%03d", buff);
    else sprintf(tmp, "0x%02X", buff);
    
    usart_QueueWrite(&usart_tx_pc, tmp, strlen(tmp));
    
    do { 
        while ((!DRV_USART0_TransmitBufferIsFull()) && (usart_tx_pc.rd_pt != usart_tx_pc.wr_pt)) {

                    DRV_USART0_WriteByte(usart_tx_pc.buffer[usart_tx_pc.rd_pt++]);
                    if (usart_tx_pc.rd_pt == USART_BUFFER_SIZE) usart_tx_pc.rd_pt = 0;
        }
    } while (usart_tx_pc.rd_pt != usart_tx_pc.wr_pt); 
}