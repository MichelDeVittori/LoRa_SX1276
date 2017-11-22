/*******************************************************************************
  USART driver static implementation of Buffer Queue model.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usart_static_buffer_queue.c

  Summary:
    Source code for the USART driver static implementation of Buffer queue model.

  Description:
    This file contains the source code for the static implementation of the
    USART driver Buffer queue model.

  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.

    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
//DOM-IGNORE-END
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "system_config.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

/* This is the array of USART Driver Buffer object of combined Queue Depth length*/
DRV_USART_BUFFER_OBJ gDrvUSARTBufferObj[DRV_USART_QUEUE_DEPTH_COMBINED];

/*This a global token counter used to generate unique buffer handles*/
static uint16_t gDrvUSARTTokenCount = 0;

extern DRV_USART_OBJ  gDrvUSART0Obj ;

// *****************************************************************************
// *****************************************************************************
// Section: Instance 0 static driver functions
// *****************************************************************************
// *****************************************************************************
void DRV_USART0_BufferAddWrite (DRV_USART_BUFFER_HANDLE * bufferHandle, void * source, const size_t nBytes)
{
    /* This function adds a buffer to the write queue */

    DRV_USART_OBJ *dObj = (DRV_USART_OBJ*)NULL;
    DRV_USART_BUFFER_OBJ * bufferObj = NULL;
    DRV_USART_BUFFER_OBJ * iterator;
    bool interruptWasEnabled = false;
    unsigned int i;

    dObj = &gDrvUSART0Obj;

    /* Check the arguments */
    if((0 == nBytes) || (NULL == source) || (NULL == bufferHandle))
    {
        /* We either got an invalid client handle,
           invalid source pointer or 0 bytes to
           transfer */

        SYS_ASSERT(false, "Invalid parameters");
        return;
    }

    /*Initialize the buffer handle */
    *bufferHandle = DRV_USART_BUFFER_HANDLE_INVALID;

    /*Comparing with the Transmit Queue size*/
    if(dObj->queueSizeCurrentWrite >= 128)
    {
        /* This means the queue is full. We cannot add
           this request */

        SYS_ASSERT(false, "Transmit Queue is full");
        return;
    }

    /* We will allow buffers to be added in the interrupt
       context of this USART driver. But we must make
       sure that if we are in interrupt, then we should
       not modify mutexes in case of RTOS. */

    if(dObj->interruptNestingCount == 0)
    {

        /* We will disable interrupts so that the queue
           status does not get updated asynchronously.
           This code will always execute. */

        interruptWasEnabled = SYS_INT_SourceDisable(INT_SOURCE_USART_2_TRANSMIT);
    }

    /* Search the buffer pool for a free buffer object */
    for(i = 0 ; i < DRV_USART_QUEUE_DEPTH_COMBINED; i ++)
    {
        if(!gDrvUSARTBufferObj[i].inUse)
        {
            /* This means this object is free.
             * Configure the object and then
             * break */
            bufferObj = &gDrvUSARTBufferObj[i];
            bufferObj->drvInstance = 0;
            bufferObj->size     = nBytes;
            bufferObj->inUse    = true;
            bufferObj->buffer   = (uint8_t*)source;
            bufferObj->nCurrentBytes = 0;
            bufferObj->next = NULL;
            bufferObj->previous = NULL;
            bufferObj->currentState    = DRV_USART_BUFFER_IS_IN_WRITE_QUEUE;
            bufferObj->flags = (DRV_USART_BUFFER_OBJ_FLAG_BUFFER_ADD);

            /* Assign a handle to this buffer. The buffer handle must be unique.
             * To do this, we construct the buffer handle out of the
             * gDrvUSARTTokenCount and allocated buffer index. Note that
             * gDrvUSARTTokenCount is incremented and wrapped around when the
             * value reaches OxFFFF. We do avoid a case where the token value
             * becomes 0xFFFF and the buffer index also becomes 0xFFFF */

            gDrvUSARTTokenCount ++;
            if(gDrvUSARTTokenCount == 0xFFFF)
            {
                gDrvUSARTTokenCount = 0;
            }

            *bufferHandle = (DRV_USART_BUFFER_HANDLE)((gDrvUSARTTokenCount << 16) | i);
            bufferObj->bufferHandle = *bufferHandle;
            break;
        }
    }

    if(i == DRV_USART_QUEUE_DEPTH_COMBINED)
    {
        /* This means we could not find a buffer. */

        SYS_ASSERT(false, "Insufficient Combined Queue Depth");

        /* Enable the interrupt if it was enabled */
        if(interruptWasEnabled)
        {
            SYS_INT_SourceEnable(INT_SOURCE_USART_2_TRANSMIT);
        }
        return;
    }

    /* Increment the current queue size*/
    dObj->queueSizeCurrentWrite ++;

    /* Check if the queue is empty */
    if(dObj->queueWrite == NULL)
    {
        /* This is the first buffer in the
           queue */

        dObj->queueWrite = bufferObj;

        /* Because this is the first buffer in the queue, we need to send the
         first byte so that we trigger the transmit interrupt generation. The
         rest of the buffer then gets processed in the task routine, which
         may or may not be called from the interrupt service routine. */

        /* Check if TX FIFO is full before sending a byte even though it is a
         first buffer in the queue, because FIFO may be full when using 
         Read/Write model along with the Buffer queue model */

        while(PLIB_USART_TransmitterBufferIsFull(USART_ID_2));
        bufferObj->nCurrentBytes ++;
        PLIB_USART_TransmitterByteSend(USART_ID_2, bufferObj->buffer[0]);

        /* If the driver is configured for interrupt mode, then the following
         statement should enable the interrupt. */

        SYS_INT_SourceEnable(INT_SOURCE_USART_2_TRANSMIT);
    }
    else
    {
        /* This means the write queue is not empty. We must add
         * the buffer object to the end of the queue */

        iterator = dObj->queueWrite;
        while(iterator->next != NULL)
        {
            /* Get the next buffer object */
            iterator = iterator->next;
        }

        /* At this point, iterator will point to the
           last object in the queue. We add the buffer
           object to the linked list. Note that we
           need to set up the previous pointer as well
           because buffer should be deleted when the
           client closes the driver */

        iterator->next = bufferObj;
        bufferObj->previous = iterator;

        /* We are done. Restore the interrupt enable status
           and return. */

        if(interruptWasEnabled)
        {
            SYS_INT_SourceEnable(INT_SOURCE_USART_2_TRANSMIT);
        }

    }
    return;
}


void DRV_USART0_BufferAddRead(DRV_USART_BUFFER_HANDLE * const bufferHandle,void * destination,const size_t nBytes)
{
    /* This function adds a buffer to the read queue */
    DRV_USART_OBJ *dObj = (DRV_USART_OBJ*)NULL;
    DRV_USART_BUFFER_OBJ * bufferObj = NULL;
    DRV_USART_BUFFER_OBJ * iterator;
    bool interruptWasEnabled = false;
    unsigned int i;

    dObj = &gDrvUSART0Obj;

    /* Check the arguments*/
    if((nBytes == 0) || (NULL == destination) || (bufferHandle == NULL))
    {
        /* We either got an invalid client handle,
           invalid destination pointer or 0 bytes to
           transfer */

        SYS_ASSERT(false, "Invalid parameters");
        return;
    }

    /* Initialize the buffer handle */
    *bufferHandle = DRV_USART_BUFFER_HANDLE_INVALID;

    /* Compare with the Receiver Queue size */
    if(dObj->queueSizeCurrentRead >= 128)
    {
        /* This means the queue is full. We cannot add
         * this request */

        //SYS_ASSERT(false, "Receive Queue is full");
        return;
    }

    /* We will allow buffers to be added in the interrupt
     * context of this USART driver. But we must make
     * sure that if we are in interrupt, then we should
     * not modify mutexes in case of RTOS. */

    if(dObj->interruptNestingCount == 0)
    {

        /* We will disable interrupts so that the queue
         * status does not get updated asynchronously.
         * This code will always execute. */

        interruptWasEnabled = SYS_INT_SourceDisable(INT_SOURCE_USART_2_RECEIVE);
    }

    /* Search the buffer pool for a free buffer object */
    for(i = 0 ; i < DRV_USART_QUEUE_DEPTH_COMBINED; i ++)
    {
        if(!gDrvUSARTBufferObj[i].inUse)
        {
            /* This means this object is free.
             * Configure the object and then
             * break */
            bufferObj = &gDrvUSARTBufferObj[i];
            bufferObj->drvInstance = 0;
            bufferObj->size     = nBytes;
            bufferObj->inUse    = true;
            bufferObj->buffer   = (uint8_t*)destination;
            bufferObj->next     = NULL;
            bufferObj->previous = NULL;
            bufferObj->nCurrentBytes = 0;
            bufferObj->currentState    = DRV_USART_BUFFER_IS_IN_READ_QUEUE;
            bufferObj->flags = (DRV_USART_BUFFER_OBJ_FLAG_BUFFER_ADD);

            /* Assign a handle to this buffer. The buffer handle must be unique.
             * To do this, we construct the buffer handle out of the
             * gDrvUSARTTokenCount and allocated buffer index. Note that
             * gDrvUSARTTokenCount is incremented and wrapped around when the
             * value reaches OxFFFF. We do avoid a case where the token value
             * becomes 0xFFFF and the buffer index also becomes 0xFFFF */

            gDrvUSARTTokenCount ++;
            if(gDrvUSARTTokenCount == 0xFFFF)
            {
                gDrvUSARTTokenCount = 0;
            }

            *bufferHandle = (DRV_USART_BUFFER_HANDLE)((gDrvUSARTTokenCount << 16) | i);
            bufferObj->bufferHandle = *bufferHandle;

            break;
        }
    }

    if(i == DRV_USART_QUEUE_DEPTH_COMBINED)
    {
        /* This means we could not find a buffer. */

        SYS_ASSERT(false, "Insufficient Combined Queue Depth");

        /* Enable the interrupt if it was enabled */
        if(interruptWasEnabled)
        {
            SYS_INT_SourceEnable(INT_SOURCE_USART_2_RECEIVE);
        }
        return;
    }

    /* Increment the current queue size*/
    dObj->queueSizeCurrentRead ++;

    /* Check if the queue is empty */
    if(dObj->queueRead == NULL)
    {
        /* This is the first buffer in the
           queue */

        dObj->queueRead = bufferObj;

        /* This is the first item in the queue. Enable
           RX interrupt. */

        SYS_INT_SourceEnable(INT_SOURCE_USART_2_RECEIVE);
    }
    else
    {
        /* This means the read queue is not empty. We must add
           the buffer object to the end of the queue */

        iterator = dObj->queueRead;
        while(iterator->next != NULL)
        {
            /* Get the next buffer object */
            iterator = iterator->next;
        }

        /* At this point, iterator will point to the
           last object in the queue. We add the buffer
           object to the linked list. Note that we
           need to set up the previous pointer as well
           because buffer should be deleted when the
           client closes the driver */

        iterator->next = bufferObj;
        bufferObj->previous = iterator;

        /* We are done. Restore the interrupt enable status
           and return. */

        if(interruptWasEnabled)
        {
            SYS_INT_SourceEnable(INT_SOURCE_USART_2_RECEIVE);
        }
    }
    return;
}


void DRV_USART0_BufferEventHandlerSet(const DRV_USART_BUFFER_EVENT_HANDLER eventHandler,const uintptr_t context)
{
    DRV_USART_OBJ *dObj = (DRV_USART_OBJ*)NULL;

    dObj = &gDrvUSART0Obj;

    /* Register the event handler with the client */
    dObj->eventHandler = eventHandler;
    dObj->context = context;
}

size_t DRV_USART0_BufferProcessedSizeGet( DRV_USART_BUFFER_HANDLE bufferHandle )
{
    DRV_USART_BUFFER_OBJ * bufferObj;

    /* Validate the buffer handle */
    if(DRV_USART_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        return 0;
    }

    /* The buffer index is the contained in the lower 16 bits of the buffer
     * handle */
    bufferObj = &gDrvUSARTBufferObj[bufferHandle & 0xFFFF];

    /* Check if the buffer is being used by the client */
    if(!bufferObj->inUse)
    {
        return 0;
    }

    /* Return the processed number of bytes */
    return(bufferObj->nCurrentBytes);
}

size_t DRV_USART0_BufferCompletedBytesGet(DRV_USART_BUFFER_HANDLE bufferHandle)
{
    DRV_USART_BUFFER_OBJ * bufferObj;
    size_t processedBytes = DRV_USART_BUFFER_HANDLE_INVALID;

    /* Validate the handle */
    if(DRV_USART_BUFFER_HANDLE_INVALID != bufferHandle)
    {
        /* The buffer index is the contained in the lower 16 bits 
         * of the buffer handle */
        bufferObj = &gDrvUSARTBufferObj[bufferHandle & 0xFFFF];
        
        /* Check if the buffer object is being used by any client */
        if(bufferObj->inUse)
        {
            /* Get the number of bytes processed. */
            processedBytes = bufferObj->nCurrentBytes;
        }
    }

    /* Return the processed number of bytes..
     * If the buffer handle is invalid or bufferObj is expired
     * then return DRV_USART_BUFFER_HANDLE_INVALID */
    return processedBytes;
}

DRV_USART_BUFFER_RESULT DRV_USART0_BufferRemove(DRV_USART_BUFFER_HANDLE bufferHandle)
{
    DRV_USART_OBJ *dObj;
    DRV_USART_BUFFER_OBJ * bufferObj;
    bool mutexGrabbed = true;
    bool interruptWasEnabled = false;
    DRV_USART_BUFFER_RESULT bufferResult = DRV_USART_BUFFER_RESULT_REMOVAL_FAILED;

    /* Validate the handle */
    if(DRV_USART_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        bufferResult = DRV_USART_BUFFER_RESULT_HANDLE_INVALID;
    }
    else
    {

        /* The buffer index is the contained in the lower 16 bits of the buffer
         * handle */
        bufferObj = &gDrvUSARTBufferObj[bufferHandle & 0xFFFF];
        
        /* Get the driver object */
        dObj = &gDrvUSART0Obj;
        
        /* We will allow modifications to buffer queue in the interrupt
         * context of this USART driver. But we must make
         * sure that if we are in interrupt, then we should
         * not modify mutexes. */

        if(dObj->interruptNestingCount == 0)
        {   

            /* We will disable interrupts so that the queue
             * status does not get updated asynchronously.
             * This code will always execute. */
            if( DRV_USART_BUFFER_IS_IN_WRITE_QUEUE == bufferObj->currentState)
            {
                interruptWasEnabled = SYS_INT_SourceDisable(INT_SOURCE_USART_2_TRANSMIT);
            }
            else if( DRV_USART_BUFFER_IS_IN_READ_QUEUE == bufferObj->currentState)
            {
                interruptWasEnabled = SYS_INT_SourceDisable(INT_SOURCE_USART_2_RECEIVE);
            }
        }

        /* mutexGrabbed will always be true for non-RTOS case.
         * Will be false when mutex aquisition timed out in RTOS mode */
        if(true == mutexGrabbed)
        {
            /* Check if the buffer object is being used by any client */
            if(!bufferObj->inUse)
            {
                /* Buffer is not present in the queue */
                bufferResult = DRV_USART_BUFFER_RESULT_HANDLE_EXPIRED;
            }
            else
            {
                if( DRV_USART_BUFFER_IS_IN_WRITE_QUEUE == bufferObj->currentState)
                {

                    if(dObj->queueWrite == bufferObj)
                    {
                        /* This is the first buffer in the
                           queue */

                        /* Get the next buffer in the write queue */
                         dObj->queueWrite = bufferObj->next;
                         
                        if (dObj->queueWrite != NULL)
                        {
                            /* Reset the new head's previous pointer */
                            dObj->queueWrite->previous = NULL;
                        }
                        else
                        {
                            /* The queue is empty. We can disable the interrupt */
                            SYS_INT_SourceDisable(INT_SOURCE_USART_2_TRANSMIT);
                        }
                    }
                    else
                    {
                        /* Buffer queue link updates */
                        
                        /* Update previous buffer objects next pointer 
                         * to the next buffer object of current buffer */
                        bufferObj->previous->next = bufferObj->next;
                        
                        /* This may be the last buffer in the queue, if not 
                         * then update the previous pointer of the next one */
                        if(NULL != bufferObj->next)
                        {
                            bufferObj->next->previous = bufferObj->previous;
                        }
                    }
             
                    /* Update the write queue size */
                    dObj->queueSizeCurrentWrite --;
                }
                
                else if( DRV_USART_BUFFER_IS_IN_READ_QUEUE == bufferObj->currentState)
                {
                    if(dObj->queueRead == bufferObj)
                    {
                        /* This is the first buffer in the
                           queue */

                        /* Get the next buffer in the read queue */
                         dObj->queueRead = bufferObj->next;
                         
                        if (dObj->queueRead != NULL)
                        {
                            /* Reset the new head's previous pointer */
                            dObj->queueRead->previous = NULL;
                        }
                    }
                    else
                    {
                        /* Buffer queue link updates */
                        
                        /* Update previous buffer objects next pointer 
                         * to the next buffer object of current buffer */
                        bufferObj->previous->next = bufferObj->next;
                        
                        /* This may be the last buffer in the queue, if not 
                         * then update the previous pointer of the next one */
                        if(NULL != bufferObj->next)
                        {
                            bufferObj->next->previous = bufferObj->previous;
                        }
                    }
             
                    /* Update the read queue size */
                    dObj->queueSizeCurrentRead --;
                }

                /* Reset the current buffers flags and pointers */
                bufferObj->inUse = false;
                bufferObj->next = NULL;
                bufferObj->previous = NULL;
                bufferObj->currentState = DRV_USART_BUFFER_IS_FREE;

                if(interruptWasEnabled)
                {
                    /* Restore interrupts. */
                    if( DRV_USART_BUFFER_IS_IN_WRITE_QUEUE == bufferObj->currentState)
                    {
                        SYS_INT_SourceEnable(INT_SOURCE_USART_2_TRANSMIT);
                    }
                    else if( DRV_USART_BUFFER_IS_IN_READ_QUEUE == bufferObj->currentState)
                    {
                        SYS_INT_SourceEnable(INT_SOURCE_USART_2_RECEIVE);
                    }
                }

                /* Update the buffer processing result */
                bufferResult = DRV_USART_BUFFER_RESULT_REMOVED_SUCCESFULLY;
            }
        }
    }

    /* Return the buffer result */
    return bufferResult;
}
/*******************************************************************************
 End of File
*/
