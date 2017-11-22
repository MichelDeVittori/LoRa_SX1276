/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C) 2014 Semtech
 
Description: -
 
License: Revised BSD License, see LICENSE.TXT file include in the project
 
Maintainers: Miguel Luis, Gregory Cristian and Nicolas Huguenin
*/
#ifndef __SX1276_HAL_H__
#define __SX1276_HAL_H__

#include <stdint.h>
#include <stdbool.h>

#define RADIO_INIT_REGISTERS_VALUE                \
{                                                 \
    { MODEM_FSK , REG_LNA                , 0x23 },\
    { MODEM_FSK , REG_RXCONFIG           , 0x1E },\
    { MODEM_FSK , REG_RSSICONFIG         , 0xD2 },\
    { MODEM_FSK , REG_AFCFEI             , 0x01 },\
    { MODEM_FSK , REG_PREAMBLEDETECT     , 0xAA },\
    { MODEM_FSK , REG_OSC                , 0x07 },\
    { MODEM_FSK , REG_SYNCCONFIG         , 0x12 },\
    { MODEM_FSK , REG_SYNCVALUE1         , 0xC1 },\
    { MODEM_FSK , REG_SYNCVALUE2         , 0x94 },\
    { MODEM_FSK , REG_SYNCVALUE3         , 0xC1 },\
    { MODEM_FSK , REG_PACKETCONFIG1      , 0xD8 },\
    { MODEM_FSK , REG_FIFOTHRESH         , 0x8F },\
    { MODEM_FSK , REG_IMAGECAL           , 0x02 },\
    { MODEM_FSK , REG_DIOMAPPING1        , 0x00 },\
    { MODEM_FSK , REG_DIOMAPPING2        , 0x30 },\
    { MODEM_LORA, REG_LR_PAYLOADMAXLENGTH, 0x40 },\
}                                                 \

#define RF_MID_BAND_THRESH                          525000000

extern const struct Radio_s Radio;

/*!
 * @brief Gets the board PA selection configuration
 *
 * @param [IN] channel Channel frequency in Hz
 * @retval PaSelect RegPaConfig PaSelect value
 */
uint8_t SX1276GetPaSelect( uint32_t channel );

/*!
 * @brief Set the RF Switch I/Os pins in Low Power mode
 *
 * @param [IN] status enable or disable
 */
void SX1276SetAntSwLowPower( bool status );

/*!
 * @brief Initializes the RF Switch I/Os pins interface
 */
void SX1276AntSwInit( void );

/*!
 * @brief De-initializes the RF Switch I/Os pins interface 
 *
 * \remark Needed to decrease the power consumption in MCU lowpower modes
 */
void SX1276AntSwDeInit( void );

/*!
 * @brief Controls the antena switch if necessary.
 *
 * \remark see errata note
 *
 * @param [IN] rxTx [1: Tx, 0: Rx]
 */
void SX1276SetAntSw( uint8_t rxTx );
  
/*!
 * @brief Checks if the given RF frequency is supported by the hardware
 *
 * @param [IN] frequency RF frequency to be checked
 * @retval isSupported [true: supported, false: unsupported]
 */
bool SX1276CheckRfFrequency( uint32_t frequency );

/*!
 * \brief Writes multiple radio registers starting at address
 *
 * \param [IN] addr   First Radio register address
 * \param [IN] buffer Buffer containing the new register's values
 * \param [IN] size   Number of registers to be written
 */
void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads multiple radio registers starting at address
 *
 * \param [IN] addr First Radio register address
 * \param [OUT] buffer Buffer where to copy the registers data
 * \param [IN] size Number of registers to be read
 */
void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );

/*!
 * @brief Reset the SX1276
 */
void SX1276Reset( void );


#endif // __SX1276_HAL_H__