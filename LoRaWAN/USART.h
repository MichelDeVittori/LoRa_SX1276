#ifndef USART_H
#define USART_H

/* Include files **************************************************************/
#include <stdio.h>

/* Defines ********************************************************************/
#define USART_BUFFER_SIZE 512

/* Structures *****************************************************************/
typedef struct USART_QUEUE{
    int8_t buffer[USART_BUFFER_SIZE];
    uint16_t rd_pt, wr_pt;
}UsartQueue_t;

/* Prototipes *****************************************************************/
void usart_QueueWrite(UsartQueue_t *queue, uint8_t *buffer, uint16_t size);

void SerialPrint(uint8_t *buff);
void SerialPrintBuffer(UsartQueue_t *queue, uint8_t *buffer, uint16_t size);

#define DEC 0
#define HEX 1
void SerialPrintNr(uint32_t buff, uint8_t base);

#endif