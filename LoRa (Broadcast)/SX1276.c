/* Include files **************************************************************/
#include <xc.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

/* Include my files ***********************************************************/
#include "SX1276.h"
#include "IO.h"
#include "Delay.h"
#include "USART.h"
#include "Settings.h"
#include "driver/spi/static/drv_spi_static.h"

uint8_t sx1272_SPI_WriteData(uint8_t data)
{
    while (!SPI1STATbits.SPITBE); /* Wait for tx buffer empty */
    SPI1BUF = data;
    while (SPI1STATbits.SPIBUSY);
    return SPI1BUF;
}

/* Original source: http://cpham.perso.univ-pau.fr/LORA/RPIgateway.html
 * 
 *  Oct, 28th, 2016 by M. De Vittori
 *      * code adapted to C and modified
 *          - TODO change millis()
 *
 *  Jan, 23rd, 2016 by C. Pham
 *      - the packet format at transmission does not use the original Libelium format anymore
 *      * the retry field is removed therefore all operations using retry will probably not work well, not tested though
 *          - therefore DO NOT use sx1272_sendPacketTimeoutACKRetries()
 *          - the reason is that we do not want to have a reserved byte after the payload
 *      * the length field is removed because it is much better to get the packet length at the reception side
 *      * after the dst field, we inserted a packet type field to better identify the packet type: DATA, ACK, encryption, app key,...
 *          - the format is now dst(1B) ptype(1B) src(1B) seq(1B) payload(xB)
 *          - ptype is decomposed in 2 parts type(4bits) flags(4bits)
 *          - type can take current value of DATA=0001 and ACK=0010
 *          - the flags are from left to right: ack_requested|encrypted|with_appkey|is_binary
 *          - ptype can be set with sx1272_setPacketType(), see constant defined in SX1272.h
 *          - the header length is then 4 instead of 5
 *  Jan, 16th, 2016 by C. Pham
 *      - add support for SX1276, automatic detect
 *      - add LF/HF calibaration copied from LoRaMAC-Node. Don't know if it is really necessary though
 *      - change various radio settings
 *  Dec, 10th, 2015 by C. Pham
 *      - add SyncWord for test with simple LoRaWAN
 *      - add mode 11 that have BW=125, CR=4/5, SF=7 on channel 868.1MHz
 *          - use following in your code if (loraMode==11) { status = sx1272.sx1272_setChannel(CH_18_868); }
 *  Nov, 13th, 2015 by C. Pham
 *      - add sx1272_CarrierSense() to perform some Listen Before Talk procedure
 *      - add dynamic ACK suport
 *          - compile with W_REQUESTED_ACK, retry field is used to indicate at the receiver
 *			  that an ACK should be sent
 *          - receiveWithTimeout() has been modified to send an ACK if retry is 1
 *          - at sender side, sx1272_sendPacketTimeoutACK() has been modified to indicate
 *			  whether retry should be set to 1 or not in sx1272_setPacket()
 *          - receiver should always use receiveWithTimeout() while sender decides to use
 *			  sx1272_sendPacketTimeout() or sx1272_sendPacketTimeoutACK()
 *  Jun, 2015 by C. Pham
 *      - Add time on air computation and CAD features
*/

// Added by C. Pham
// based on SIFS=3CAD
uint8_t sx1272_SIFS_value[11]={0, 183, 94, 44, 47, 23, 24, 12, 12, 7, 4};
uint8_t sx1272_CAD_value[11]={0, 62, 31, 16, 16, 8, 9, 5, 3, 1, 1};
// end

//**********************************************************************/
// Public functions.
//**********************************************************************/

void sx1272(void)
{
    // Initialize class variables
    _bandwidth = BW_125;
    _codingRate = CR_5;
    _spreadingFactor = SF_12;
    _channel = CH_18_868;
    _header = HEADER_ON;
    _CRC = CRC_OFF;
    _modem = LORA;
    _power = 15;
    _packetNumber = 0;
    _reception = CORRECT_PACKET;
    _retries = 0;
    // added by C. Pham
    _defaultSyncWord=0x12;
    _rawFormat=FALSE;
    _extendedIFS=TRUE;
    _RSSIonSend=TRUE;
    // disabled by default
    _enableCarrierSense=FALSE;
    // DIFS by default
    _send_cad_number=9;
    _tempCalibration=8;
    
    #ifdef W_REQUESTED_ACK
        _requestACK = 0;
    #endif

    #ifdef W_NET_KEY
        _my_netkey[0] = net_key_0;
        _my_netkey[1] = net_key_1;
    #endif
        
    _maxRetries = 3;
    packet_sent.retry = _retries;
};

static uint32_t bitRead(uint32_t data, uint8_t pos)
{
    return (data&(1<<pos))?1:0;
}

// added by C. Pham
// copied from LoRaMAC-Node
/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
void sx1272_RxChainCalibration(void)
{
    if (_board==SX1276Chip) {

        SerialPrint("Start SX1276 LF/HF calibration"NL);

        // Cut the PA just in case, RFO output, power = -1 dBm
        sx1272_writeRegister( REG_PA_CONFIG, 0x00 );
    
        // Launch Rx chain calibration for LF band
        sx1272_writeRegister( REG_IMAGE_CAL, ( sx1272_readRegister( REG_IMAGE_CAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
        while( ( sx1272_readRegister( REG_IMAGE_CAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
        {
        }
    
        // Sets a Frequency in HF band
        sx1272_setChannel(CH_17_868);
    
        // Launch Rx chain calibration for HF band
        sx1272_writeRegister( REG_IMAGE_CAL, ( sx1272_readRegister( REG_IMAGE_CAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
        while( ( sx1272_readRegister( REG_IMAGE_CAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING );
    }
}


/*
 Function: Sets the module ON.
 Returns: uint8_t setLORA state
*/
uint8_t sx1272_ON(void)
{    
    sx1272();
    
    uint8_t state = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'ON'"NL);
#endif
    
    /* sx1272 ? */
    pin_Set(RFM95W_RST, PIN_HIGH);
    delay_ms(1000);
    pin_Set(RFM95W_RST, PIN_LOW);    
    delay_ms(100);

    // from single_chan_pkt_fwd by Thomas Telkamp
    uint8_t version = sx1272_readRegister(REG_VERSION);

    if (version == 0x22) {
        /* sx1272 */
        SerialPrint("SX1272 detected, starting..."NL);
        _board = SX1272Chip;
    } else {
        /* Chip-Select */    
        pin_Set(RFM95W_RST, PIN_LOW);
        delay_ms(100);
        pin_Set(RFM95W_RST, PIN_HIGH);
        delay_ms(100);
        
        version = sx1272_readRegister(REG_VERSION);
        if (version == 0x12) {
            // sx1276
            SerialPrint("SX1276 detected, starting..."NL);
            _board = SX1276Chip;
        } else {
            SerialPrint("Unrecognized transceiver"NL);
            return 255;
        }
    }
    // end from single_chan_pkt_fwd by Thomas Telkamp

    // added by C. Pham
    sx1272_RxChainCalibration();

    sx1272_setMaxCurrent(0x1B);
#ifdef SX1272_DEBUG
    SerialPrint("## Setting ON with maximum current supply ##"NL);
#endif

    // set LoRa mode
    state = sx1272_setLORA();

    // Added by C. Pham for ToA computation
    sx1272_getPreambleLength();
#ifdef W_NET_KEY
    //#ifdef SX1272_DEBUG
    SerialPrint("## SX1272 layer has net key##"NL);
    //#endif
#endif

#ifdef W_INITIALIZATION
    // CAUTION
    // doing initialization as proposed by Libelium seems not to work for the SX1276
    // so we decided to leave the default value of the SX127x, then configure the radio when
    // setting to LoRa mode

    //Set initialization values
    sx1272_writeRegister(0x0,0x0);
    // comment by C. Pham
    // still valid for SX1276
    sx1272_writeRegister(0x1,0x81);
    // end
    sx1272_writeRegister(0x2,0x1A);
    sx1272_writeRegister(0x3,0xB);
    sx1272_writeRegister(0x4,0x0);
    sx1272_writeRegister(0x5,0x52);
    sx1272_writeRegister(0x6,0xD8);
    sx1272_writeRegister(0x7,0x99);
    sx1272_writeRegister(0x8,0x99);
    // modified by C. Pham
    // added by C. Pham
    if (_board==SX1272Chip)
        // RFIO_pin RFU OutputPower
        // 0 000 0000
        sx1272_writeRegister(0x9,0x0);
    else
        // RFO_pin MaxP OutputPower
        // 0 100 1111
        // set MaxPower to 0x4 and OutputPower to 0
        sx1272_writeRegister(0x9,0x40);

    sx1272_writeRegister(0xA,0x9);
    sx1272_writeRegister(0xB,0x3B);

    // comment by C. Pham
    // still valid for SX1276
    sx1272_writeRegister(0xC,0x23);

    // REG_RX_CONFIG
    sx1272_writeRegister(0xD,0x1);

    sx1272_writeRegister(0xE,0x80);
    sx1272_writeRegister(0xF,0x0);
    sx1272_writeRegister(0x10,0x0);
    sx1272_writeRegister(0x11,0x0);
    sx1272_writeRegister(0x12,0x0);
    sx1272_writeRegister(0x13,0x0);
    sx1272_writeRegister(0x14,0x0);
    sx1272_writeRegister(0x15,0x0);
    sx1272_writeRegister(0x16,0x0);
    sx1272_writeRegister(0x17,0x0);
    sx1272_writeRegister(0x18,0x10);
    sx1272_writeRegister(0x19,0x0);
    sx1272_writeRegister(0x1A,0x0);
    sx1272_writeRegister(0x1B,0x0);
    sx1272_writeRegister(0x1C,0x0);

    // added by C. Pham
    if (_board==SX1272Chip) {
        // comment by C. Pham
        // 0x4A = 01 001 0 1 0
        // BW=250 CR=4/5 ImplicitH_off RxPayloadCrcOn_on LowDataRateOptimize_off
        sx1272_writeRegister(0x1D,0x4A);
        // 1001 0 1 11
        // SF=9 TxContinuous_off AgcAutoOn SymbTimeOut
        sx1272_writeRegister(0x1E,0x97);
    }
    else {
        // 1000 001 0
        // BW=250 CR=4/5 ImplicitH_off
        sx1272_writeRegister(0x1D,0x82);
        // 1001 0 1 11
        // SF=9 TxContinuous_off RxPayloadCrcOn_on SymbTimeOut
        sx1272_writeRegister(0x1E,0x97);
    }
    // end

    sx1272_writeRegister(0x1F,0xFF);
    sx1272_writeRegister(0x20,0x0);
    sx1272_writeRegister(0x21,0x8);
    sx1272_writeRegister(0x22,0xFF);
    sx1272_writeRegister(0x23,0xFF);
    sx1272_writeRegister(0x24,0x0);
    sx1272_writeRegister(0x25,0x0);

    // added by C. Pham
    if (_board==SX1272Chip)
        sx1272_writeRegister(0x26,0x0);
    else
        // 0000 0 1 00
        // reserved LowDataRateOptimize_off AgcAutoOn reserved
        sx1272_writeRegister(0x26,0x04);

    // REG_SYNC_CONFIG
    sx1272_writeRegister(0x27,0x0);

    sx1272_writeRegister(0x28,0x0);
    sx1272_writeRegister(0x29,0x0);
    sx1272_writeRegister(0x2A,0x0);
    sx1272_writeRegister(0x2B,0x0);
    sx1272_writeRegister(0x2C,0x0);
    sx1272_writeRegister(0x2D,0x50);
    sx1272_writeRegister(0x2E,0x14);
    sx1272_writeRegister(0x2F,0x40);
    sx1272_writeRegister(0x30,0x0);
    sx1272_writeRegister(0x31,0x3);
    sx1272_writeRegister(0x32,0x5);
    sx1272_writeRegister(0x33,0x27);
    sx1272_writeRegister(0x34,0x1C);
    sx1272_writeRegister(0x35,0xA);
    sx1272_writeRegister(0x36,0x0);
    sx1272_writeRegister(0x37,0xA);
    sx1272_writeRegister(0x38,0x42);
    sx1272_writeRegister(0x39,0x12);
    //sx1272_writeRegister(0x3A,0x65);
    //sx1272_writeRegister(0x3B,0x1D);
    //sx1272_writeRegister(0x3C,0x1);
    //sx1272_writeRegister(0x3D,0xA1);
    //sx1272_writeRegister(0x3E,0x0);
    //sx1272_writeRegister(0x3F,0x0);
    //sx1272_writeRegister(0x40,0x0);
    //sx1272_writeRegister(0x41,0x0);
    // commented by C. Pham
    // since now we handle also the SX1276
    //sx1272_writeRegister(0x42,0x22);
#endif
    // added by C. Pham
    // default sync word for non-LoRaWAN
    sx1272_setSyncWord(_defaultSyncWord);
    sx1272_getSyncWord();
    _defaultSyncWord=_syncWord;
    //end

    return state;
}

/*
 Function: Sets the module OFF.
 Returns: Nothing
*/
void sx1272_OFF(void)
{
#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'OFF'"NL);
#endif

    pin_Set(RFM95W_CS, PIN_HIGH);
#ifdef SX1272_DEBUG
    SerialPrint("## Setting OFF ##"NL);    
#endif
}

/*
 Function: Reads the indicated register.
 Returns: The content of the register
 Parameters:
   address: address register to read from
*/
byte sx1272_readRegister(byte address)
{
    byte value;

    pin_Set(RFM95W_CS, PIN_LOW);
    address &= ~0x80;		// Bit 7 cleared to write in registers
    sx1272_SPI_WriteData(address);
    value = sx1272_SPI_WriteData(0x00);
    pin_Set(RFM95W_CS, PIN_HIGH);

#ifdef SX1272_DEBUG
    SerialPrint("## Reading register ");
    SerialPrintNr(address, HEX);
    SerialPrint(": data ");
    SerialPrintNr(value, HEX);
    SerialPrint(NL);
#endif

    return value;
}

/*
 Function: Writes on the indicated register.
 Returns: Nothing
 Parameters:
   address: address register to write in
   data : value to write in the register
*/
void sx1272_writeRegister(byte address, byte data)
{
    pin_Set(RFM95W_CS, PIN_LOW);
    address |= 0x80;			
    sx1272_SPI_WriteData(address);
    sx1272_SPI_WriteData(data);
    pin_Set(RFM95W_CS, PIN_HIGH);

#ifdef SX1272_DEBUG
    SerialPrint("## Writing register ");
    address &= ~0x80;
    SerialPrintNr(address, HEX);
    SerialPrint(": data ");
    SerialPrintNr(data, HEX);
    SerialPrint(NL);
#endif

}

/*
 Function: Clears the interruption flags
 Returns: Nothing
*/
void sx1272_clearFlags(void)
{
    byte st0;

    st0 = sx1272_readRegister(REG_OP_MODE);		// Save the previous status

    if( _modem == LORA )
    { // LoRa mode
        sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Stdby mode to write in registers
        sx1272_writeRegister(REG_IRQ_FLAGS, 0xFF);	// LoRa mode flags register
        sx1272_writeRegister(REG_OP_MODE, st0);		// Getting back to previous status
#ifdef SX1272_DEBUG
        SerialPrint("## LoRa flags cleared ##"NL);
#endif
    }
    else
    { // FSK mode
        sx1272_writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Stdby mode to write in registers
        sx1272_writeRegister(REG_IRQ_FLAGS1, 0xFF); // FSK mode flags1 register
        sx1272_writeRegister(REG_IRQ_FLAGS2, 0xFF); // FSK mode flags2 register
        sx1272_writeRegister(REG_OP_MODE, st0);		// Getting back to previous status
#ifdef SX1272_DEBUG
        SerialPrint("## FSK flags cleared ##"NL);
#endif
    }
}

/*
 Function: Sets the module in LoRa mode.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_setLORA(void)
{
    uint8_t state = 2;
    byte st0;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setLORA'"NL);
#endif

    // modified by C. Pham
    uint8_t retry=0;
    
    SerialPrint("Set LoRa ...");

    do {
        delay_ms(200);
        sx1272_writeRegister(REG_OP_MODE, FSK_SLEEP_MODE);    // Sleep mode (mandatory to set LoRa mode)
        sx1272_writeRegister(REG_OP_MODE, LORA_SLEEP_MODE);    // LoRa sleep mode
        sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
        delay_ms(50+retry*10);
        st0 = sx1272_readRegister(REG_OP_MODE);
        SerialPrint(" ..."NL);

        if ((retry % 2)==0)
            if (retry==20)
                retry=0;
            else
                retry++;
        /*
        if (st0!=LORA_STANDBY_MODE) {
            pinMode(SX1272_RST,OUTPUT);
            digitalWrite(SX1272_RST,HIGH);
            delay_ms(100);
            digitalWrite(SX1272_RST,LOW);
        }
        */

    } while (st0!=LORA_STANDBY_MODE);	// LoRa standby mode

    if( st0 == LORA_STANDBY_MODE)
    { // LoRa mode
        _modem = LORA;
        state = 0;
#ifdef SX1272_DEBUG
        SerialPrint("## LoRa set with success ##"NL);        
#endif
    }
    else
    { // FSK mode
        _modem = FSK;
        state = 1;
#ifdef SX1272_DEBUG
        SerialPrint("** There has been an error while setting LoRa **"NL);        
#endif
    }
    return state;
}

/*
 Function: Sets the module in FSK mode.
 Returns:   Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_setFSK(void)
{
    uint8_t state = 2;
    byte st0;
    byte config1;

    if (_board==SX1276Chip)
        SerialPrint("Warning: FSK has not been tested on SX1276!"NL);

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setFSK'"NL);
#endif

    sx1272_writeRegister(REG_OP_MODE, FSK_SLEEP_MODE);	// Sleep mode (mandatory to change mode)
    sx1272_writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// FSK standby mode
    config1 = sx1272_readRegister(REG_PACKET_CONFIG1);
    config1 = config1 & 0b01111101;		// clears bits 8 and 1 from REG_PACKET_CONFIG1
    config1 = config1 | 0b00000100;		// sets bit 2 from REG_PACKET_CONFIG1
    sx1272_writeRegister(REG_PACKET_CONFIG1,config1);	// AddressFiltering = NodeAddress + BroadcastAddress
    sx1272_writeRegister(REG_FIFO_THRESH, 0x80);	// condition to start packet tx
    config1 = sx1272_readRegister(REG_SYNC_CONFIG);
    config1 = config1 & 0b00111111;
    sx1272_writeRegister(REG_SYNC_CONFIG,config1);

    delay_ms(100);

    st0 = sx1272_readRegister(REG_OP_MODE);	// Reading config mode
    if( st0 == FSK_STANDBY_MODE )
    { // FSK mode
        _modem = FSK;
        state = 0;
#ifdef SX1272_DEBUG
        SerialPrint("## FSK set with success ##"NL);        
#endif
    }
    else
    { // LoRa mode
        _modem = LORA;
        state = 1;
#ifdef SX1272_DEBUG
        SerialPrint("** There has been an error while setting FSK **"NL);        
#endif
    }
    return state;
}

/*
 Function: Gets the bandwidth, coding rate and spreading factor of the LoRa modulation.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_getMode(void)
{
    byte st0;
    int8_t state = 2;
    byte value = 0x00;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'getMode'"NL);
#endif

    st0 = sx1272_readRegister(REG_OP_MODE);		// Save the previous status
    if( _modem == FSK )
    {
        sx1272_setLORA();					// Setting LoRa mode
    }
    value = sx1272_readRegister(REG_MODEM_CONFIG1);
    // added by C. Pham
    if (_board==SX1272Chip) {
        _bandwidth = (value >> 6);   			// Storing 2 MSB from REG_MODEM_CONFIG1 (=_bandwidth)
        // added by C. Pham
        // convert to common bandwidth values used by both SX1272 and SX1276
        _bandwidth += 7;
    }
    else
        _bandwidth = (value >> 4);   			// Storing 4 MSB from REG_MODEM_CONFIG1 (=_bandwidth)

    if (_board==SX1272Chip)
        _codingRate = (value >> 3) & 0x07;  		// Storing third, forth and fifth bits from
    else
        _codingRate = (value >> 1) & 0x07;  		// Storing 3-1 bits REG_MODEM_CONFIG1 (=_codingRate)

    value = sx1272_readRegister(REG_MODEM_CONFIG2);
    _spreadingFactor = (value >> 4) & 0x0F; 	// Storing 4 MSB from REG_MODEM_CONFIG2 (=_spreadingFactor)
    state = 1;

    if( sx1272_isBW(_bandwidth) )		// Checking available values for:
    {								//		_bandwidth
        if( sx1272_isCR(_codingRate) )		//		_codingRate
        {							//		_spreadingFactor
            if( sx1272_isSF(_spreadingFactor) )
            {
                state = 0;
            }
        }
    }

#ifdef SX1272_DEBUG
    SerialPrint("## Parameters from configuration mode are:"NL);
    SerialPrint("Bandwidth: ");
    SerialPrintNr(_bandwidth, HEX);
    SerialPrint(NL);
    SerialPrint("\t Coding Rate: ");
    SerialPrintNr(_codingRate, HEX);
    SerialPrint(NL);
    SerialPrint("\t Spreading Factor: ");
    SerialPrintNr(_spreadingFactor, HEX);
    SerialPrint(NL);
    SerialPrint(" ##"NL);
#endif

    sx1272_writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    delay_ms(100);
    return state;
}

/*
 Function: Sets the bandwidth, coding rate and spreading factor of the LoRa modulation.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   mode: mode number to set the required BW, SF and CR of LoRa modem.
*/
int8_t sx1272_setMode(uint8_t mode)
{
    int8_t state = 2;
    byte st0;
    byte config1 = 0x00;
    byte config2 = 0x00;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setMode'"NL);
#endif

    st0 = sx1272_readRegister(REG_OP_MODE);		// Save the previous status

    if( _modem == FSK )
    {
        sx1272_setLORA();
    }
    sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// LoRa standby mode

    switch (mode)
    {
    // mode 1 (better reach, medium time on air)
    case 1:
        sx1272_setCR(CR_5);        // CR = 4/5
        sx1272_setSF(SF_12);       // SF = 12
        sx1272_setBW(BW_125);      // BW = 125 KHz
        break;

    // mode 2 (medium reach, less time on air)
    case 2:
        sx1272_setCR(CR_5);        // CR = 4/5
        sx1272_setSF(SF_12);       // SF = 12
        sx1272_setBW(BW_250);      // BW = 250 KHz
        break;

    // mode 3 (worst reach, less time on air)
    case 3:
        sx1272_setCR(CR_5);        // CR = 4/5
        sx1272_setSF(SF_10);       // SF = 10
        sx1272_setBW(BW_125);      // BW = 125 KHz
        break;

    // mode 4 (better reach, low time on air)
    case 4:
        sx1272_setCR(CR_5);        // CR = 4/5
        sx1272_setSF(SF_12);       // SF = 12
        sx1272_setBW(BW_500);      // BW = 500 KHz
        break;

    // mode 5 (better reach, medium time on air)
    case 5:
        sx1272_setCR(CR_5);        // CR = 4/5
        sx1272_setSF(SF_10);       // SF = 10
        sx1272_setBW(BW_250);      // BW = 250 KHz
        break;

    // mode 6 (better reach, worst time-on-air)
    case 6:
        sx1272_setCR(CR_5);        // CR = 4/5
        sx1272_setSF(SF_11);       // SF = 11
        sx1272_setBW(BW_500);      // BW = 500 KHz
        break;

    // mode 7 (medium-high reach, medium-low time-on-air)
    case 7:
        sx1272_setCR(CR_5);        // CR = 4/5
        sx1272_setSF(SF_9);        // SF = 9
        sx1272_setBW(BW_250);      // BW = 250 KHz
        break;

        // mode 8 (medium reach, medium time-on-air)
    case 8:     
    	sx1272_setCR(CR_5);        // CR = 4/5
        sx1272_setSF(SF_9);        // SF = 9
        sx1272_setBW(BW_500);      // BW = 500 KHz
        break;

    // mode 9 (medium-low reach, medium-high time-on-air)
    case 9:
        sx1272_setCR(CR_5);        // CR = 4/5
        sx1272_setSF(SF_8);        // SF = 8
        sx1272_setBW(BW_500);      // BW = 500 KHz
        break;

    // mode 10 (worst reach, less time_on_air)
    case 10:
        sx1272_setCR(CR_5);        // CR = 4/5
        sx1272_setSF(SF_7);        // SF = 7
        sx1272_setBW(BW_500);      // BW = 500 KHz
        break;

    // added by C. Pham
    // test for LoRaWAN channel
    case 11:
        sx1272_setCR(CR_5);        // CR = 4/5
        sx1272_setSF(SF_12);        // SF = 12
        sx1272_setBW(BW_125);      // BW = 125 KHz
        // set the sync word to the LoRaWAN sync word which is 0x34
        sx1272_setSyncWord(0x34);
        SerialPrint("** Using sync word of 0x"NL);
        SerialPrintNr(_syncWord, HEX);
        break;

    default:    state = -1; // The indicated mode doesn't exist

    };

    if( state == -1 )	// if state = -1, don't change its value
    {
#ifdef SX1272_DEBUG
        SerialPrint("** The indicated mode doesn't exist, "NL);
        SerialPrint("please select from 1 to 10 **"NL);
#endif
    }
    else
    {
        state = 1;
        config1 = sx1272_readRegister(REG_MODEM_CONFIG1);
        switch (mode)
        {   //      Different way to check for each mode:
        // (config1 >> 3) ---> take out bits 7-3 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
        // (config2 >> 4) ---> take out bits 7-4 from REG_MODEM_CONFIG2 (=_spreadingFactor)

        // mode 1: BW = 125 KHz, CR = 4/5, SF = 12.
        case 1:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x01 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x39 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = sx1272_readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_12 )
                {
                    state = 0;
                }
            }
            break;


            // mode 2: BW = 250 KHz, CR = 4/5, SF = 12.
        case 2:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x09 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x41 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = sx1272_readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_12 )
                {
                    state = 0;
                }
            }
            break;

            // mode 3: BW = 125 KHz, CR = 4/5, SF = 10.
        case 3:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x01 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x39 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = sx1272_readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_10 )
                {
                    state = 0;
                }
            }
            break;

            // mode 4: BW = 500 KHz, CR = 4/5, SF = 12.
        case 4:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x11 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x49 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = sx1272_readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_12 )
                {
                    state = 0;
                }
            }
            break;

            // mode 5: BW = 250 KHz, CR = 4/5, SF = 10.
        case 5:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x09 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x41 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = sx1272_readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_10 )
                {
                    state = 0;
                }
            }
            break;

            // mode 6: BW = 500 KHz, CR = 4/5, SF = 11.
        case 6:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x11 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x49 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = sx1272_readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_11 )
                {
                    state = 0;
                }
            }
            break;

            // mode 7: BW = 250 KHz, CR = 4/5, SF = 9.
        case 7:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x09 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x41 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = sx1272_readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_9 )
                {
                    state = 0;
                }
            }
            break;

            // mode 8: BW = 500 KHz, CR = 4/5, SF = 9.
        case 8:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x11 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x49 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = sx1272_readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_9 )
                {
                    state = 0;
                }
            }
            break;

            // mode 9: BW = 500 KHz, CR = 4/5, SF = 8.
        case 9:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x11 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x49 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = sx1272_readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_8 )
                {
                    state = 0;
                }
            }
            break;

            // mode 10: BW = 500 KHz, CR = 4/5, SF = 7.
        case 10:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x11 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x49 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = sx1272_readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_7 )
                {
                    state = 0;
                }
            }
            break;

            // added by C. Pham
            // test of LoRaWAN channel
            // mode 11: BW = 125 KHz, CR = 4/5, SF = 12.
        case 11:

            //modified by C. Pham
            if (_board==SX1272Chip) {
                if( (config1 >> 3) == 0x01 )
                    state=0;
            }
            else {
                // (config1 >> 1) ---> take out bits 7-1 from REG_MODEM_CONFIG1 (=_bandwidth & _codingRate together)
                if( (config1 >> 1) == 0x39 )
                    state=0;
            }

            if( state==0) {
                state = 1;
                config2 = sx1272_readRegister(REG_MODEM_CONFIG2);

                if( (config2 >> 4) == SF_12 )
                {
                    state = 0;
                }
            }
            break;
        }// end switch

        if (mode!=11) {
            sx1272_setSyncWord(_defaultSyncWord);
#ifdef SX1272_DEBUG
            SerialPrint("** Using sync word of 0x");
            SerialPrintNr(_defaultSyncWord, HEX);
#endif
        }
    }
    // added by C. Pham
    if (state == 0)
        _loraMode=mode;

#ifdef SX1272_DEBUG

    if( state == 0 )
    {
        SerialPrint("## Mode ");
        SerialPrintNr(mode, DEC);
        SerialPrint(" configured with success ##"NL);
    }
    else
    {
        SerialPrint("** There has been an error while configuring mode ");
        SerialPrintNr(mode, DEC);
        SerialPrint(". **"NL);
    }
#endif

    sx1272_writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    delay_ms(100);
    return state;
}

/*
 Function: Indicates if module is configured in implicit or explicit header mode.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	sx1272_getHeader(void)
{
    int8_t state = 2;

#ifdef SX1272_DEBUG
    
    SerialPrint("Starting 'getHeader'"NL);
#endif

    // added by C. Pham
    uint8_t theHeaderBit;

    if (_board==SX1272Chip)
        theHeaderBit=2;
    else
        theHeaderBit=0;

    // take out bit 2 from REG_MODEM_CONFIG1 indicates ImplicitHeaderModeOn
    if( bitRead(REG_MODEM_CONFIG1, theHeaderBit) == 0 )
    { // explicit header mode (ON)
        _header = HEADER_ON;
        state = 1;
    }
    else
    { // implicit header mode (OFF)
        _header = HEADER_OFF;
        state = 1;
    }

    state = 0;

    if( _modem == FSK )
    { // header is not available in FSK mode
#ifdef SX1272_DEBUG
        SerialPrint("## Notice that FSK mode packets hasn't header ##"NL);
        
#endif
    }
    else
    { // header in LoRa mode
#ifdef SX1272_DEBUG
        SerialPrint("## Header is ");
        if( _header == HEADER_ON )
        {
            SerialPrint("in explicit header mode ##"NL);
        }
        else
        {
            SerialPrint("in implicit header mode ##"NL);
        }
        
#endif
    }
    return state;
}

/*
 Function: Sets the module in explicit header mode (header is sent).
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t sx1272_setHeader_ON(void)
{
    int8_t state = 2;
    byte config1;

#ifdef SX1272_DEBUG
    
    SerialPrint("Starting 'setHeaderON'"NL);
#endif

    if( _modem == FSK )
    {
        state = -1;		// header is not available in FSK mode
#ifdef SX1272_DEBUG
        SerialPrint("## FSK mode packets hasn't header ##"NL);
        
#endif
    }
    else
    {
        config1 = sx1272_readRegister(REG_MODEM_CONFIG1);	// Save config1 to modify only the header bit
        if( _spreadingFactor == 6 )
        {
            state = -1;		// Mandatory headerOFF with SF = 6
#ifdef SX1272_DEBUG
            SerialPrint("## Mandatory implicit header mode with spreading factor = 6 ##"NL);
#endif
        }
        else
        {
            // added by C. Pham
            if (_board==SX1272Chip)
                config1 = config1 & 0b11111011;		// clears bit 2 from config1 = headerON
            else
                config1 = config1 & 0b11111110;              // clears bit 0 from config1 = headerON

            sx1272_writeRegister(REG_MODEM_CONFIG1,config1);	// Update config1
        }

        // added by C. Pham
        uint8_t theHeaderBit;

        if (_board==SX1272Chip)
            theHeaderBit=2;
        else
            theHeaderBit=0;

        if( _spreadingFactor != 6 )
        { // checking headerON taking out bit 2 from REG_MODEM_CONFIG1
            config1 = sx1272_readRegister(REG_MODEM_CONFIG1);
            // modified by C. Pham
            if( bitRead(config1, theHeaderBit) == HEADER_ON )
            {
                state = 0;
                _header = HEADER_ON;
#ifdef SX1272_DEBUG
                SerialPrint("## Header has been activated ##"NL);                
#endif
            }
            else
            {
                state = 1;
            }
        }
        return state;
    }
}

/*
 Function: Sets the module in implicit header mode (header is not sent).
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t sx1272_setHeader_OFF(void)
{
    uint8_t state = 2;
    byte config1;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setHeaderOFF'"NL);
#endif

    if( _modem == FSK )
    { // header is not available in FSK mode
        state = -1;
#ifdef SX1272_DEBUG
        SerialPrint("## Notice that FSK mode packets hasn't header ##"NL);        
#endif
    }
    else
    {
        config1 = sx1272_readRegister(REG_MODEM_CONFIG1);	// Save config1 to modify only the header bit

        // modified by C. Pham
        if (_board==SX1272Chip)
            config1 = config1 | 0b00000100;			// sets bit 2 from REG_MODEM_CONFIG1 = headerOFF
        else
            config1 = config1 | 0b00000001;                      // sets bit 0 from REG_MODEM_CONFIG1 = headerOFF

        sx1272_writeRegister(REG_MODEM_CONFIG1,config1);		// Update config1

        config1 = sx1272_readRegister(REG_MODEM_CONFIG1);

        // added by C. Pham
        uint8_t theHeaderBit;

        if (_board==SX1272Chip)
            theHeaderBit=2;
        else
            theHeaderBit=0;

        if( bitRead(config1, theHeaderBit) == HEADER_OFF )
        { // checking headerOFF taking out bit 2 from REG_MODEM_CONFIG1
            state = 0;
            _header = HEADER_OFF;

#ifdef SX1272_DEBUG
            SerialPrint("## Header has been desactivated ##"NL);            
#endif
        }
        else
        {
            state = 1;
#ifdef SX1272_DEBUG
            SerialPrint("** Header hasn't been desactivated ##"NL);            
#endif
        }
    }
    return state;
}

/*
 Function: Indicates if module is configured with or without checking CRC.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	sx1272_getCRC(void)
{
    int8_t state = 2;
    byte value;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'getCRC'"NL);
#endif

    if( _modem == LORA )
    { // LoRa mode

        // added by C. Pham
        uint8_t theRegister;
        uint8_t theCrcBit;

        if (_board==SX1272Chip) {
            theRegister=REG_MODEM_CONFIG1;
            theCrcBit=1;
        }
        else {
            theRegister=REG_MODEM_CONFIG2;
            theCrcBit=2;
        }

        // take out bit 1 from REG_MODEM_CONFIG1 indicates RxPayloadCrcOn
        value = sx1272_readRegister(theRegister);
        if( bitRead(value, theCrcBit) == CRC_OFF )
        { // CRCoff
            _CRC = CRC_OFF;
#ifdef SX1272_DEBUG
            SerialPrint("## CRC is desactivated ##"NL);            
#endif
            state = 0;
        }
        else
        { // CRCon
            _CRC = CRC_ON;
#ifdef SX1272_DEBUG
            SerialPrint("## CRC is activated ##"NL);            
#endif
            state = 0;
        }
    }
    else
    { // FSK mode

        // take out bit 2 from REG_PACKET_CONFIG1 indicates CrcOn
        value = sx1272_readRegister(REG_PACKET_CONFIG1);
        if( bitRead(value, 4) == CRC_OFF )
        { // CRCoff
            _CRC = CRC_OFF;
#ifdef SX1272_DEBUG
            SerialPrint("## CRC is desactivated ##"NL);            
#endif
            state = 0;
        }
        else
        { // CRCon
            _CRC = CRC_ON;
#ifdef SX1272_DEBUG
            SerialPrint("## CRC is activated ##"NL);            
#endif
            state = 0;
        }
    }
    if( state != 0 )
    {
        state = 1;
#ifdef SX1272_DEBUG
        SerialPrint("** There has been an error while getting configured CRC **"NL);        
#endif
    }
    return state;
}

/*
 Function: Sets the module with CRC on.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	sx1272_setCRC_ON(void)
{
    uint8_t state = 2;
    byte config1;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setCRC_ON'"NL);
#endif

    if( _modem == LORA )
    { // LORA mode

        // added by C. Pham
        uint8_t theRegister;
        uint8_t theCrcBit;

        if (_board==SX1272Chip) {
            theRegister=REG_MODEM_CONFIG1;
            theCrcBit=1;
        }
        else {
            theRegister=REG_MODEM_CONFIG2;
            theCrcBit=2;
        }

        config1 = sx1272_readRegister(theRegister);	// Save config1 to modify only the CRC bit

        if (_board==SX1272Chip)
            config1 = config1 | 0b00000010;				// sets bit 1 from REG_MODEM_CONFIG1 = CRC_ON
        else
            config1 = config1 | 0b00000100;                               // sets bit 2 from REG_MODEM_CONFIG2 = CRC_ON

        sx1272_writeRegister(theRegister,config1);

        state = 1;

        config1 = sx1272_readRegister(theRegister);

        if( bitRead(config1, theCrcBit) == CRC_ON )
        { // take out bit 1 from REG_MODEM_CONFIG1 indicates RxPayloadCrcOn
            state = 0;
            _CRC = CRC_ON;
#ifdef SX1272_DEBUG
            SerialPrint("## CRC has been activated ##"NL);            
#endif
        }
    }
    else
    { // FSK mode
        config1 = sx1272_readRegister(REG_PACKET_CONFIG1);	// Save config1 to modify only the CRC bit
        config1 = config1 | 0b00010000;				// set bit 4 and 3 from REG_MODEM_CONFIG1 = CRC_ON
        sx1272_writeRegister(REG_PACKET_CONFIG1,config1);

        state = 1;

        config1 = sx1272_readRegister(REG_PACKET_CONFIG1);
        if( bitRead(config1, 4) == CRC_ON )
        { // take out bit 4 from REG_PACKET_CONFIG1 indicates CrcOn
            state = 0;
            _CRC = CRC_ON;
#ifdef SX1272_DEBUG
            SerialPrint("## CRC has been activated ##"NL);            
#endif
        }
    }
    if( state != 0 )
    {
        state = 1;
#ifdef SX1272_DEBUG
        SerialPrint("** There has been an error while setting CRC ON **"NL);        
#endif
    }
    return state;
}

/*
 Function: Sets the module with CRC off.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	sx1272_setCRC_OFF(void)
{
    int8_t state = 2;
    byte config1;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setCRC_OFF'"NL);
#endif

    if( _modem == LORA )
    { // LORA mode

        // added by C. Pham
        uint8_t theRegister;
        uint8_t theCrcBit;

        if (_board==SX1272Chip) {
            theRegister=REG_MODEM_CONFIG1;
            theCrcBit=1;
        }
        else {
            theRegister=REG_MODEM_CONFIG2;
            theCrcBit=2;
        }

        config1 = sx1272_readRegister(theRegister);	// Save config1 to modify only the CRC bit
        if (_board==SX1272Chip)
            config1 = config1 & 0b11111101;				// clears bit 1 from config1 = CRC_OFF
        else
            config1 = config1 & 0b11111011;				// clears bit 2 from config1 = CRC_OFF

        sx1272_writeRegister(theRegister,config1);

        config1 = sx1272_readRegister(theRegister);
        if( (bitRead(config1, theCrcBit)) == CRC_OFF )
        { // take out bit 1 from REG_MODEM_CONFIG1 indicates RxPayloadCrcOn
            state = 0;
            _CRC = CRC_OFF;
#ifdef SX1272_DEBUG
            SerialPrint("## CRC has been desactivated ##"NL);            
#endif
        }
    }
    else
    { // FSK mode
        config1 = sx1272_readRegister(REG_PACKET_CONFIG1);	// Save config1 to modify only the CRC bit
        config1 = config1 & 0b11101111;				// clears bit 4 from config1 = CRC_OFF
        sx1272_writeRegister(REG_PACKET_CONFIG1,config1);

        config1 = sx1272_readRegister(REG_PACKET_CONFIG1);
        if( bitRead(config1, 4) == CRC_OFF )
        { // take out bit 4 from REG_PACKET_CONFIG1 indicates RxPayloadCrcOn
            state = 0;
            _CRC = CRC_OFF;
#ifdef SX1272_DEBUG
            SerialPrint("## CRC has been desactivated ##"NL);            
#endif
        }
    }
    if( state != 0 )
    {
        state = 1;
#ifdef SX1272_DEBUG
        SerialPrint("** There has been an error while setting CRC OFF **"NL);        
#endif
    }
    return state;
}

/*
 Function: Checks if SF is a valid value.
 Returns: Boolean that's 'TRUE' if the SF value exists and
          it's 'FALSE' if the SF value does not exist.
 Parameters:
   spr: spreading factor value to check.
*/
boolean	sx1272_isSF(uint8_t spr)
{
#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'isSF'"NL);
#endif

    // Checking available values for _spreadingFactor
    switch(spr)
    {
    case SF_6:
    case SF_7:
    case SF_8:
    case SF_9:
    case SF_10:
    case SF_11:
    case SF_12:
        return TRUE;
        break;

    default:
        return FALSE;
    }
#ifdef SX1272_DEBUG
    SerialPrint("## Finished 'isSF' ##"NL);    
#endif
}

/*
 Function: Gets the SF within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t sx1272_getSF(void)
{
    int8_t state = 2;
    byte config2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'getSF'"NL);
#endif

    if( _modem == FSK )
    {
        state = -1;		// SF is not available in FSK mode
#ifdef SX1272_DEBUG
        SerialPrint("** FSK mode hasn't spreading factor **"NL);        
#endif
    }
    else
    {
        // take out bits 7-4 from REG_MODEM_CONFIG2 indicates _spreadingFactor
        config2 = (sx1272_readRegister(REG_MODEM_CONFIG2)) >> 4;
        _spreadingFactor = config2;
        state = 1;

        if( (config2 == _spreadingFactor) && sx1272_isSF(_spreadingFactor) )
        {
            state = 0;
#ifdef SX1272_DEBUG
            SerialPrint("## Spreading factor is ");
            SerialPrintNr(_spreadingFactor, HEX);
            SerialPrint(" ##"NL);
            
#endif
        }
    }
    return state;
}

/*
 Function: Sets the indicated SF in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   spr: spreading factor value to set in LoRa modem configuration.
*/
uint8_t	sx1272_setSF(uint8_t spr)
{
    byte st0;
    int8_t state = 2;
    byte config1;
    byte config2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setSF'"NL);
#endif

    st0 = sx1272_readRegister(REG_OP_MODE);	// Save the previous status

    if( _modem == FSK )
    {
#ifdef SX1272_DEBUG
        SerialPrint("## Notice that FSK hasn't Spreading Factor parameter, "
            "so you are configuring it in LoRa mode ##"NL);
#endif
        state = sx1272_setLORA();				// Setting LoRa mode
    }
    else
    { // LoRa mode
        sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// LoRa standby mode
        config2 = (sx1272_readRegister(REG_MODEM_CONFIG2));	// Save config2 to modify SF value (bits 7-4)
        switch(spr)
        {
        case SF_6: 	config2 = config2 & 0b01101111;	// clears bits 7 & 4 from REG_MODEM_CONFIG2
            config2 = config2 | 0b01100000;	// sets bits 6 & 5 from REG_MODEM_CONFIG2
            sx1272_setHeader_OFF();		// Mandatory headerOFF with SF = 6
            break;
        case SF_7: 	config2 = config2 & 0b01111111;	// clears bits 7 from REG_MODEM_CONFIG2
            config2 = config2 | 0b01110000;	// sets bits 6, 5 & 4
            break;
        case SF_8: 	config2 = config2 & 0b10001111;	// clears bits 6, 5 & 4 from REG_MODEM_CONFIG2
            config2 = config2 | 0b10000000;	// sets bit 7 from REG_MODEM_CONFIG2
            break;
        case SF_9: 	config2 = config2 & 0b10011111;	// clears bits 6, 5 & 4 from REG_MODEM_CONFIG2
            config2 = config2 | 0b10010000;	// sets bits 7 & 4 from REG_MODEM_CONFIG2
            break;
        case SF_10:	config2 = config2 & 0b10101111;	// clears bits 6 & 4 from REG_MODEM_CONFIG2
            config2 = config2 | 0b10100000;	// sets bits 7 & 5 from REG_MODEM_CONFIG2
            break;
        case SF_11:	config2 = config2 & 0b10111111;	// clears bit 6 from REG_MODEM_CONFIG2
            config2 = config2 | 0b10110000;	// sets bits 7, 5 & 4 from REG_MODEM_CONFIG2
            sx1272_getBW();

            // modified by C. Pham
            if( _bandwidth == BW_125)
            { // LowDataRateOptimize (Mandatory with SF_11 if BW_125)
                if (_board==SX1272Chip) {
                    config1 = (sx1272_readRegister(REG_MODEM_CONFIG1));	// Save config1 to modify only the LowDataRateOptimize
                    config1 = config1 | 0b00000001;
                    sx1272_writeRegister(REG_MODEM_CONFIG1,config1);
                }
                else {
                    byte config3=sx1272_readRegister(REG_MODEM_CONFIG3);
                    config3 = config3 | 0b00001000;
                    sx1272_writeRegister(REG_MODEM_CONFIG3,config3);
                }
            }
            break;
        case SF_12: config2 = config2 & 0b11001111;	// clears bits 5 & 4 from REG_MODEM_CONFIG2
            config2 = config2 | 0b11000000;	// sets bits 7 & 6 from REG_MODEM_CONFIG2
            if( _bandwidth == BW_125)
            { // LowDataRateOptimize (Mandatory with SF_12 if BW_125)
                // modified by C. Pham
                if (_board==SX1272Chip) {
                    config1 = (sx1272_readRegister(REG_MODEM_CONFIG1));	// Save config1 to modify only the LowDataRateOptimize
                    config1 = config1 | 0b00000001;
                    sx1272_writeRegister(REG_MODEM_CONFIG1,config1);
                }
                else {
                    byte config3=sx1272_readRegister(REG_MODEM_CONFIG3);
                    config3 = config3 | 0b00001000;
                    sx1272_writeRegister(REG_MODEM_CONFIG3,config3);
                }
            }
            break;
        }

        // Check if it is neccesary to set special settings for SF=6
        if( spr == SF_6 )
        {
            // Mandatory headerOFF with SF = 6 (Implicit mode)
            sx1272_setHeader_OFF();

            // Set the bit field DetectionOptimize of
            // register RegLoRaDetectOptimize to value "0b101".
            sx1272_writeRegister(REG_DETECT_OPTIMIZE, 0x05);

            // Write 0x0C in the register RegDetectionThreshold.
            sx1272_writeRegister(REG_DETECTION_THRESHOLD, 0x0C);
        }
        else
        {
            // added by C. Pham
            sx1272_setHeader_ON();

            // LoRa detection Optimize: 0x03 --> SF7 to SF12
            sx1272_writeRegister(REG_DETECT_OPTIMIZE, 0x03);

            // LoRa detection threshold: 0x0A --> SF7 to SF12
            sx1272_writeRegister(REG_DETECTION_THRESHOLD, 0x0A);
        }

        // added by C. Pham
        if (_board==SX1272Chip) {
            // comment by C. Pham
            // bit 9:8 of SymbTimeout are then 11
            // single_chan_pkt_fwd uses 00 and then 00001000
            // why?
            // sets bit 2-0 (AgcAutoOn and SymbTimout) for any SF value
            //config2 = config2 | 0b00000111;
            // modified by C. Pham
            config2 = config2 | 0b00000100;
            sx1272_writeRegister(REG_MODEM_CONFIG1, config1);		// Update config1
        }
        else {
            // set the AgcAutoOn in bit 2 of REG_MODEM_CONFIG3
            uint8_t config3 = (sx1272_readRegister(REG_MODEM_CONFIG3));
            config3=config3 | 0b00000100;
            sx1272_writeRegister(REG_MODEM_CONFIG3, config3);
        }

        // here we write the new SF
        sx1272_writeRegister(REG_MODEM_CONFIG2, config2);		// Update config2

        delay_ms(100);

        // added by C. Pham
        byte configAgc;
        uint8_t theLDRBit;

        if (_board==SX1272Chip) {
            config1 = (sx1272_readRegister(REG_MODEM_CONFIG1));	// Save config1 to check update
            config2 = (sx1272_readRegister(REG_MODEM_CONFIG2));	// Save config2 to check update
            // comment by C. Pham
            // (config2 >> 4) ---> take out bits 7-4 from REG_MODEM_CONFIG2 (=_spreadingFactor)
            // bitRead(config1, 0) ---> take out bits 1 from config1 (=LowDataRateOptimize)
            // config2 is only for the AgcAutoOn
            configAgc=config2;
            theLDRBit=0;
        }
        else {
            config1 = (sx1272_readRegister(REG_MODEM_CONFIG3));	// Save config1 to check update
            config2 = (sx1272_readRegister(REG_MODEM_CONFIG2));
            // LowDataRateOptimize is in REG_MODEM_CONFIG3
            // AgcAutoOn is in REG_MODEM_CONFIG3
            configAgc=config1;
            theLDRBit=3;
        }


        switch(spr)
        {
        case SF_6:	if(		((config2 >> 4) == spr)
                            && 	(bitRead(configAgc, 2) == 1)
                            && 	(_header == HEADER_OFF))
            {
                state = 0;
            }
            break;
        case SF_7:	if(		((config2 >> 4) == 0x07)
                            && (bitRead(configAgc, 2) == 1))
            {
                state = 0;
            }
            break;
        case SF_8:	if(		((config2 >> 4) == 0x08)
                            && (bitRead(configAgc, 2) == 1))
            {
                state = 0;
            }
            break;
        case SF_9:	if(		((config2 >> 4) == 0x09)
                            && (bitRead(configAgc, 2) == 1))
            {
                state = 0;
            }
            break;
        case SF_10:	if(		((config2 >> 4) == 0x0A)
                            && (bitRead(configAgc, 2) == 1))
            {
                state = 0;
            }
            break;
        case SF_11:	if(		((config2 >> 4) == 0x0B)
                            && (bitRead(configAgc, 2) == 1)
                            && (bitRead(config1, theLDRBit) == 1))
            {
                state = 0;
            }
            break;
        case SF_12:	if(		((config2 >> 4) == 0x0C)
                            && (bitRead(configAgc, 2) == 1)
                            && (bitRead(config1, theLDRBit) == 1))
            {
                state = 0;
            }
            break;
        default:	state = 1;
        }
    }

    sx1272_writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    delay_ms(100);

    if( sx1272_isSF(spr) )
    { // Checking available value for _spreadingFactor
        state = 0;
        _spreadingFactor = spr;
#ifdef SX1272_DEBUG
        SerialPrint("## Spreading factor ");
        SerialPrintNr(_spreadingFactor, DEC);
        SerialPrint(" has been successfully set ##"NL);        
#endif
    }
    else
    {
        if( state != 0 )
        {
#ifdef SX1272_DEBUG
            SerialPrint("** There has been an error while setting the spreading factor **"NL);            
#endif
        }
    }
    return state;
}

/*
 Function: Checks if BW is a valid value.
 Returns: Boolean that's 'TRUE' if the BW value exists and
          it's 'FALSE' if the BW value does not exist.
 Parameters:
   band: bandwidth value to check.
*/
boolean	sx1272_isBW(uint16_t band)
{
#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'isBW'"NL);
#endif

    // Checking available values for _bandwidth
    // added by C. Pham
    if (_board==SX1272Chip) {
        switch(band)
        {
        case BW_125:
        case BW_250:
        case BW_500:
            return TRUE;
            break;

        default:
            return FALSE;
        }
    }
    else {
        switch(band)
        {
        case BW_7_8:
        case BW_10_4:
        case BW_15_6:
        case BW_20_8:
        case BW_31_25:
        case BW_41_7:
        case BW_62_5:
        case BW_125:
        case BW_250:
        case BW_500:
            return TRUE;
            break;

        default:
            return FALSE;
        }
    }

#ifdef SX1272_DEBUG
    SerialPrint("## Finished 'isBW' ##"NL);    
#endif
}

/*
 Function: Gets the BW within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t sx1272_getBW(void)
{
    uint8_t state = 2;
    byte config1;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'getBW'"NL);
#endif

    if( _modem == FSK )
    {
        state = -1;		// BW is not available in FSK mode
#ifdef SX1272_DEBUG
        SerialPrint("** FSK mode hasn't bandwidth **"NL);        
#endif
    }
    else
    {
        // added by C. Pham
        if (_board==SX1272Chip) {
            // take out bits 7-6 from REG_MODEM_CONFIG1 indicates _bandwidth
            config1 = (sx1272_readRegister(REG_MODEM_CONFIG1)) >> 6;
        }
        else {
            // take out bits 7-4 from REG_MODEM_CONFIG1 indicates _bandwidth
            config1 = (sx1272_readRegister(REG_MODEM_CONFIG1)) >> 4;
        }

        _bandwidth = config1;

        if( (config1 == _bandwidth) && sx1272_isBW(_bandwidth) )
        {
            state = 0;
#ifdef SX1272_DEBUG
            SerialPrint("## Bandwidth is ");
            SerialPrintNr(_bandwidth,HEX);
            SerialPrint(" ##"NL);
            
#endif
        }
        else
        {
            state = 1;
#ifdef SX1272_DEBUG
            SerialPrint("** There has been an error while getting bandwidth **"NL);
            
#endif
        }
    }
    return state;
}

/*
 Function: Sets the indicated BW in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   band: bandwith value to set in LoRa modem configuration.
*/
int8_t sx1272_setBW(uint16_t band)
{
    byte st0;
    int8_t state = 2;
    byte config1;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setBW'"NL);
#endif

    if(!sx1272_isBW(band) )
    {
        state = 1;
#ifdef SX1272_DEBUG
        SerialPrint("** Bandwidth ");
        SerialPrintNr(band, HEX);
        SerialPrint(" is not a correct value **"NL);
        
#endif
        return state;
    }

    st0 = sx1272_readRegister(REG_OP_MODE);	// Save the previous status

    if( _modem == FSK )
    {
#ifdef SX1272_DEBUG
        SerialPrint("## Notice that FSK hasn't Bandwidth parameter, "
            "so you are configuring it in LoRa mode ##"NL);
#endif
        state = sx1272_setLORA();
    }
    sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// LoRa standby mode
    config1 = (sx1272_readRegister(REG_MODEM_CONFIG1));	// Save config1 to modify only the BW

    // added by C. Pham for SX1276
    if (_board==SX1272Chip) {
        switch(band)
        {
        case BW_125:  config1 = config1 & 0b00111111;	// clears bits 7 & 6 from REG_MODEM_CONFIG1
            sx1272_getSF();
            if( _spreadingFactor == 11 )
            { // LowDataRateOptimize (Mandatory with BW_125 if SF_11)
                config1 = config1 | 0b00000001;
            }
            if( _spreadingFactor == 12 )
            { // LowDataRateOptimize (Mandatory with BW_125 if SF_12)
                config1 = config1 | 0b00000001;
            }
            break;
        case BW_250:  config1 = config1 & 0b01111111;	// clears bit 7 from REG_MODEM_CONFIG1
            config1 = config1 | 0b01000000;	// sets bit 6 from REG_MODEM_CONFIG1
            break;
        case BW_500:  config1 = config1 & 0b10111111;	//clears bit 6 from REG_MODEM_CONFIG1
            config1 = config1 | 0b10000000;	//sets bit 7 from REG_MODEM_CONFIG1
            break;
        }
    }
    else {
        // SX1276
        config1 = config1 & 0b00001111;	// clears bits 7 - 4 from REG_MODEM_CONFIG1
        switch(band)
        {
        case BW_125:
            // 0111
            config1 = config1 | 0b01110000;
            sx1272_getSF();
            if( _spreadingFactor == 11 || _spreadingFactor == 12)
            { // LowDataRateOptimize (Mandatory with BW_125 if SF_11 or SF_12)
                byte config3=sx1272_readRegister(REG_MODEM_CONFIG3);
                config3 = config3 | 0b00001000;
                sx1272_writeRegister(REG_MODEM_CONFIG3,config3);
            }
            break;
        case BW_250:
            // 1000
            config1 = config1 | 0b10000000;
            break;
        case BW_500:
            // 1001
            config1 = config1 | 0b10010000;
            break;
        }
    }
    // end

    sx1272_writeRegister(REG_MODEM_CONFIG1,config1);		// Update config1

    delay_ms(100);

    config1 = (sx1272_readRegister(REG_MODEM_CONFIG1));

    // added by C. Pham
    if (_board==SX1272Chip) {
        // (config1 >> 6) ---> take out bits 7-6 from REG_MODEM_CONFIG1 (=_bandwidth)
        switch(band)
        {
        case BW_125: if( (config1 >> 6) == SX1272_BW_125 )
            {
                state = 0;
                if( _spreadingFactor == 11 )
                {
                    if( bitRead(config1, 0) == 1 )
                    { // LowDataRateOptimize
                        state = 0;
                    }
                    else
                    {
                        state = 1;
                    }
                }
                if( _spreadingFactor == 12 )
                {
                    if( bitRead(config1, 0) == 1 )
                    { // LowDataRateOptimize
                        state = 0;
                    }
                    else
                    {
                        state = 1;
                    }
                }
            }
            break;
        case BW_250: if( (config1 >> 6) == SX1272_BW_250 )
            {
                state = 0;
            }
            break;
        case BW_500: if( (config1 >> 6) == SX1272_BW_500 )
            {
                state = 0;
            }
            break;
        }
    }
    else {
        // (config1 >> 4) ---> take out bits 7-4 from REG_MODEM_CONFIG1 (=_bandwidth)
        switch(band)
        {
        case BW_125: if( (config1 >> 4) == BW_125 )
            {
                state = 0;

                byte config3 = (sx1272_readRegister(REG_MODEM_CONFIG3));

                if( _spreadingFactor == 11 )
                {
                    if( bitRead(config3, 3) == 1 )
                    { // LowDataRateOptimize
                        state = 0;
                    }
                    else
                    {
                        state = 1;
                    }
                }
                if( _spreadingFactor == 12 )
                {
                    if( bitRead(config3, 3) == 1 )
                    { // LowDataRateOptimize
                        state = 0;
                    }
                    else
                    {
                        state = 1;
                    }
                }
            }
            break;
        case BW_250: if( (config1 >> 4) == BW_250 )
            {
                state = 0;
            }
            break;
        case BW_500: if( (config1 >> 4) == BW_500 )
            {
                state = 0;
            }
            break;
        }
    }

    if(state==0)
    {
        _bandwidth = band;
#ifdef SX1272_DEBUG
        SerialPrint("## Bandwidth ");
        SerialPrintNr(band, HEX);
        SerialPrint(" has been successfully set ##"NL);
        
#endif
    }
    sx1272_writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    delay_ms(100);
    return state;
}

/*
 Function: Checks if CR is a valid value.
 Returns: Boolean that's 'TRUE' if the CR value exists and
          it's 'FALSE' if the CR value does not exist.
 Parameters:
   cod: coding rate value to check.
*/
boolean	sx1272_isCR(uint8_t cod)
{
#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'isCR'"NL);
#endif

    // Checking available values for _codingRate
    switch(cod)
    {
    case CR_5:
    case CR_6:
    case CR_7:
    case CR_8:
        return TRUE;
        break;

    default:
        return FALSE;
    }
#ifdef SX1272_DEBUG
    SerialPrint("## Finished 'isCR' ##"NL);    
#endif
}

/*
 Function: Indicates the CR within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t sx1272_getCR(void)
{
    int8_t state = 2;
    byte config1;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'getCR'"NL);
#endif

    if( _modem == FSK )
    {
        state = -1;		// CR is not available in FSK mode
#ifdef SX1272_DEBUG
        SerialPrint("** FSK mode hasn't coding rate **"NL);        
#endif
    }
    else
    {
        // added by C. Pham
        if (_board==SX1272Chip) {
            // take out bits 7-3 from REG_MODEM_CONFIG1 indicates _bandwidth & _codingRate
            config1 = (sx1272_readRegister(REG_MODEM_CONFIG1)) >> 3;
            config1 = config1 & 0b00000111;	// clears bits 7-3 ---> clears _bandwidth
        }
        else {
            // take out bits 7-1 from REG_MODEM_CONFIG1 indicates _bandwidth & _codingRate
            config1 = (sx1272_readRegister(REG_MODEM_CONFIG1)) >> 1;
            config1 = config1 & 0b00000111;	// clears bits 7-3 ---> clears _bandwidth
        }

        _codingRate = config1;
        state = 1;

        if( (config1 == _codingRate) && sx1272_isCR(_codingRate) )
        {
            state = 0;
#ifdef SX1272_DEBUG
            SerialPrint("## Coding rate is ");
            SerialPrintNr(_codingRate, HEX);
            SerialPrint(" ##"NL);
            
#endif
        }
    }
    return state;
}

/*
 Function: Sets the indicated CR in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   cod: coding rate value to set in LoRa modem configuration.
*/
int8_t sx1272_setCR(uint8_t cod)
{
    byte st0;
    int8_t state = 2;
    byte config1;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setCR'"NL);
#endif

    st0 = sx1272_readRegister(REG_OP_MODE);		// Save the previous status

    if( _modem == FSK )
    {
#ifdef SX1272_DEBUG
        SerialPrint("## Notice that FSK hasn't Coding Rate parameter, "
            "so you are configuring it in LoRa mode ##"NL);
#endif
        state = sx1272_setLORA();
    }
    sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);		// Set Standby mode to write in registers

    config1 = sx1272_readRegister(REG_MODEM_CONFIG1);	// Save config1 to modify only the CR

    // added by C. Pham
    if (_board==SX1272Chip) {
        switch(cod)
        {
        case CR_5: config1 = config1 & 0b11001111;	// clears bits 5 & 4 from REG_MODEM_CONFIG1
            config1 = config1 | 0b00001000;	// sets bit 3 from REG_MODEM_CONFIG1
            break;
        case CR_6: config1 = config1 & 0b11010111;	// clears bits 5 & 3 from REG_MODEM_CONFIG1
            config1 = config1 | 0b00010000;	// sets bit 4 from REG_MODEM_CONFIG1
            break;
        case CR_7: config1 = config1 & 0b11011111;	// clears bit 5 from REG_MODEM_CONFIG1
            config1 = config1 | 0b00011000;	// sets bits 4 & 3 from REG_MODEM_CONFIG1
            break;
        case CR_8: config1 = config1 & 0b11100111;	// clears bits 4 & 3 from REG_MODEM_CONFIG1
            config1 = config1 | 0b00100000;	// sets bit 5 from REG_MODEM_CONFIG1
            break;
        }
    }
    else {
        // SX1276
        config1 = config1 & 0b11110001;	// clears bits 3 - 1 from REG_MODEM_CONFIG1
        switch(cod)
        {
        case CR_5:
            config1 = config1 | 0b00000010;
            break;
        case CR_6:
            config1 = config1 | 0b00000100;
            break;
        case CR_7:
            config1 = config1 | 0b00000110;
            break;
        case CR_8:
            config1 = config1 | 0b00001000;
            break;
        }
    }
    sx1272_writeRegister(REG_MODEM_CONFIG1, config1);		// Update config1

    delay_ms(100);

    config1 = sx1272_readRegister(REG_MODEM_CONFIG1);

    // added by C. Pham
    uint8_t nshift=3;

    // only 1 right shift for SX1276
    if (_board==SX1276Chip)
        nshift=1;

    // ((config1 >> 3) & B0000111) ---> take out bits 5-3 from REG_MODEM_CONFIG1 (=_codingRate)
    switch(cod)
    {
    case CR_5: if( ((config1 >> nshift) & 0b0000111) == 0x01 )
        {
            state = 0;
        }
        break;
    case CR_6: if( ((config1 >> nshift) & 0b0000111) == 0x02 )
        {
            state = 0;
        }
        break;
    case CR_7: if( ((config1 >> nshift) & 0b0000111) == 0x03 )
        {
            state = 0;
        }
        break;
    case CR_8: if( ((config1 >> nshift) & 0b0000111) == 0x04 )
        {
            state = 0;
        }
        break;
    }


    if( sx1272_isCR(cod) )
    {
        _codingRate = cod;
#ifdef SX1272_DEBUG
        SerialPrint("## Coding Rate ");
        SerialPrintNr(cod, HEX);
        SerialPrint(" has been successfully set ##"NL);        
#endif
    }
    else
    {
        state = 1;
#ifdef SX1272_DEBUG
        SerialPrint("** There has been an error while configuring Coding Rate parameter **"NL);        
#endif
    }
    sx1272_writeRegister(REG_OP_MODE,st0);	// Getting back to previous status
    delay_ms(100);
    return state;
}

/*
 Function: Checks if channel is a valid value.
 Returns: Boolean that's 'TRUE' if the CR value exists and
          it's 'FALSE' if the CR value does not exist.
 Parameters:
   ch: frequency channel value to check.
*/
boolean	sx1272_isChannel(uint32_t ch)
{
#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'isChannel'"NL);
#endif

    // Checking available values for _channel
    switch(ch)
    {
    case CH_10_868:
    case CH_11_868:
    case CH_12_868:
    case CH_13_868:
    case CH_14_868:
    case CH_15_868:
    case CH_16_868:
    case CH_17_868:
        //added by C. Pham
    case CH_18_868:
        //end
    case CH_00_900:
    case CH_01_900:
    case CH_02_900:
    case CH_03_900:
    case CH_04_900:
    case CH_05_900:
    case CH_06_900:
    case CH_07_900:
    case CH_08_900:
    case CH_09_900:
    case CH_10_900:
    case CH_11_900:
        //added by C. Pham
    case CH_12_900:
    case CH_00_433:
        //end
        return TRUE;
        break;

    default:
        return FALSE;
    }
#ifdef SX1272_DEBUG
    SerialPrint("## Finished 'isChannel' ##"NL);    
#endif
}

/*
 Function: Indicates the frequency channel within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_getChannel(void)
{
    uint8_t state = 2;
    uint32_t ch;
    uint8_t freq3;
    uint8_t freq2;
    uint8_t freq1;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'getChannel'"NL);
#endif

    freq3 = sx1272_readRegister(REG_FRF_MSB);	// frequency channel MSB
    freq2 = sx1272_readRegister(REG_FRF_MID);	// frequency channel MID
    freq1 = sx1272_readRegister(REG_FRF_LSB);	// frequency channel LSB
    ch = ((uint32_t)freq3 << 16) + ((uint32_t)freq2 << 8) + (uint32_t)freq1;
    _channel = ch;						// frequency channel

    if( (_channel == ch) && sx1272_isChannel(_channel) )
    {
        state = 0;
#ifdef SX1272_DEBUG
        SerialPrint("## Frequency channel is ");
        SerialPrintNr(_channel, HEX);
        SerialPrint(" ##"NL);
        
#endif
    }
    else
    {
        state = 1;
    }
    return state;
}

/*
 Function: Sets the indicated channel in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   ch: frequency channel value to set in configuration.
*/
int8_t sx1272_setChannel(uint32_t ch)
{
    byte st0;
    int8_t state = 2;
    unsigned int freq3;
    unsigned int freq2;
    uint8_t freq1;
    uint32_t freq;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setChannel'"NL);
#endif

    // added by C. Pham
    _starttime = millis();

    st0 = sx1272_readRegister(REG_OP_MODE);	// Save the previous status
    if( _modem == LORA )
    {
        // LoRa Stdby mode in order to write in registers
        sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
    }
    else
    {
        // FSK Stdby mode in order to write in registers
        sx1272_writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
    }

    freq3 = ((ch >> 16) & 0x0FF);		// frequency channel MSB
    freq2 = ((ch >> 8) & 0x0FF);		// frequency channel MIB
    freq1 = (ch & 0xFF);				// frequency channel LSB

    sx1272_writeRegister(REG_FRF_MSB, freq3);
    sx1272_writeRegister(REG_FRF_MID, freq2);
    sx1272_writeRegister(REG_FRF_LSB, freq1);

    // added by C. Pham
    _stoptime = millis();

    delay_ms(100);

    // storing MSB in freq channel value
    freq3 = (sx1272_readRegister(REG_FRF_MSB));
    freq = (freq3 << 8) & 0xFFFFFF;

    // storing MID in freq channel value
    freq2 = (sx1272_readRegister(REG_FRF_MID));
    freq = (freq << 8) + ((freq2 << 8) & 0xFFFFFF);

    // storing LSB in freq channel value
    freq = freq + ((sx1272_readRegister(REG_FRF_LSB)) & 0xFFFFFF);

    if( freq == ch )
    {
        state = 0;
        _channel = ch;
#ifdef SX1272_DEBUG
        SerialPrint("## Frequency channel ");
        SerialPrintNr(ch, HEX);
        SerialPrint(" has been successfully set ##"NL);        
#endif
    }
    else
    {
        state = 1;
    }

    if(!sx1272_isChannel(ch) )
    {
        state = -1;
#ifdef SX1272_DEBUG
        SerialPrint("** Frequency channel ");
        SerialPrintNr(ch, HEX);
        SerialPrint("is not a correct value **"NL);        
#endif
    }

    sx1272_writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    delay_ms(100);
    return state;
}

/*
 Function: Gets the signal power within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_getPower(void)
{
    uint8_t state = 2;
    byte value = 0x00;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'getPower'"NL);
#endif

    value = sx1272_readRegister(REG_PA_CONFIG);
    state = 1;

    // modified by C. Pham
    // get only the OutputPower
    _power = value & 0b00001111;

    if( (value > -1) & (value < 16) )
    {
        state = 0;
#ifdef SX1272_DEBUG
        SerialPrint("## Output power is ");
        SerialPrintNr(_power, HEX);
        SerialPrint(" ##"NL);
        
#endif
    }

    return state;
}

/*
 Function: Sets the signal power indicated in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   p: power option to set in configuration.
*/
int8_t sx1272_setPower(char p)
{
    byte st0;
    int8_t state = 2;
    byte value = 0x00;

    byte RegPaDacReg=(_board==SX1272Chip)?0x5A:0x4D;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setPower'"NL);
#endif

    st0 = sx1272_readRegister(REG_OP_MODE);	  // Save the previous status
    if( _modem == LORA )
    { // LoRa Stdby mode to write in registers
        sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
    }
    else
    { // FSK Stdby mode to write in registers
        sx1272_writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
    }

    switch (p)
    {
    // L = Low. On SX1272/76: PA0 on RFO setting
    // H = High. On SX1272/76: PA0 on RFO setting
    // M = MAX. On SX1272/76: PA0 on RFO setting

    // x = extreme; added by C. Pham. On SX1272/76: PA1&PA2 PA_BOOST setting
    // X = eXtreme; added by C. Pham. On SX1272/76: PA1&PA2 PA_BOOST setting + 20dBm settings

    // added by C. Pham
    //
    case 'x':
    case 'X':
    case 'M':  value = 0x0F;
        // SX1272/76: 14dBm
        break;

    // modified by C. Pham, set to 0x03 instead of 0x00
    case 'L':  value = 0x03;
        // SX1272/76: 2dBm
        break;

    case 'H':  value = 0x07;
        // SX1272/76: 6dBm
        break;

    default:   state = -1;
        break;
    }

    // 100mA
    sx1272_setMaxCurrent(0x0B);

    if (p=='x') {
        // we set only the PA_BOOST pin
        // limit to 14dBm
        value = 0x0C;
        value = value | 0b10000000;
        // set RegOcp for OcpOn and OcpTrim
        // 130mA
        sx1272_setMaxCurrent(0x10);
    }
    
    if (p=='X') {
        // normally value = 0x0F;
        // we set the PA_BOOST pin
        value = value | 0b10000000;
        // and then set the high output power config with register REG_PA_DAC
        sx1272_writeRegister(RegPaDacReg, 0x87);
        // set RegOcp for OcpOn and OcpTrim
        // 150mA
        sx1272_setMaxCurrent(0x12);
    }
    else {
        // disable high power output in all other cases
        sx1272_writeRegister(RegPaDacReg, 0x84);
    }

    // added by C. Pham
    if (_board==SX1272Chip) {
        // Pout = -1 + _power[3:0] on RFO
        // Pout = 2 + _power[3:0] on PA_BOOST
        // so: L=2dBm; H=6dBm, M=14dBm, x=14dBm (PA), X=20dBm(PA+PADAC)
        sx1272_writeRegister(REG_PA_CONFIG, value);	// Setting output power value
    }
    else {
        // for the SX1276

        // set MaxPower to 7 -> Pmax=10.8+0.6*MaxPower [dBm] = 15
        value = value | 0b01110000;

        // then Pout = Pmax-(15-_power[3:0]) if  PaSelect=0 (RFO pin for +14dBm)
        // so L=3dBm; H=7dBm; M=15dBm (but should be limited to 14dBm by RFO pin)

        // and Pout = 17-(15-_power[3:0]) if  PaSelect=1 (PA_BOOST pin for +14dBm)
        // so x= 14dBm (PA);
        // when p=='X' for 20dBm, value is 0x0F and RegPaDacReg=0x87 so 20dBm is enabled

        sx1272_writeRegister(REG_PA_CONFIG, value);
    }

    _power=value;

    value = sx1272_readRegister(REG_PA_CONFIG);

    if( value == _power )
    {
        state = 0;
#ifdef SX1272_DEBUG
        SerialPrint("## Output power has been successfully set ##"NL);        
#endif
    }
    else
    {
        state = 1;
    }

    sx1272_writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    delay_ms(100);
    return state;
}

/*
 Function: Sets the signal power indicated in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   p: power option to set in configuration.
*/
int8_t sx1272_setPowerNum(uint8_t pow)
{
    byte st0;
    int8_t state = 2;
    byte value = 0x00;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setPower'"NL);
#endif

    st0 = sx1272_readRegister(REG_OP_MODE);	  // Save the previous status
    if( _modem == LORA )
    { // LoRa Stdby mode to write in registers
        sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
    }
    else
    { // FSK Stdby mode to write in registers
        sx1272_writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
    }

    if ( (pow >= 0) & (pow < 15) )
    {
        _power = pow;
    }
    else
    {
        state = -1;
#ifdef SX1272_DEBUG
        SerialPrint("## Power value is not valid ##"NL);        
#endif
    }

    // added by C. Pham
    if (_board==SX1276Chip) {
        value=sx1272_readRegister(REG_PA_CONFIG);
        // clear OutputPower, but keep current value of PaSelect and MaxPower
        value=value & 0b11110000;
        value=value + _power;
        _power=value;
    }
    sx1272_writeRegister(REG_PA_CONFIG, _power);	// Setting output power value
    value = sx1272_readRegister(REG_PA_CONFIG);

    if( value == _power )
    {
        state = 0;
#ifdef SX1272_DEBUG
        SerialPrint("## Output power has been successfully set ##"NL);        
#endif
    }
    else
    {
        state = 1;
    }

    sx1272_writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    delay_ms(100);
    return state;
}


/*
 Function: Gets the preamble length from the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_getPreambleLength(void)
{
    int8_t state = 2;
    uint8_t p_length;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'getPreambleLength'"NL);
#endif

    state = 1;
    if( _modem == LORA )
    { // LORA mode
        p_length = sx1272_readRegister(REG_PREAMBLE_MSB_LORA);
        // Saving MSB preamble length in LoRa mode
        _preamblelength = (p_length << 8) & 0xFFFF;
        p_length = sx1272_readRegister(REG_PREAMBLE_LSB_LORA);
        // Saving LSB preamble length in LoRa mode
        _preamblelength = _preamblelength + (p_length & 0xFFFF);
#ifdef SX1272_DEBUG
        SerialPrint("## Preamble length configured is ");
        SerialPrintNr(_preamblelength, HEX);
        SerialPrint(" ##"NL);
        
#endif
    }
    else
    { // FSK mode
        p_length = sx1272_readRegister(REG_PREAMBLE_MSB_FSK);
        // Saving MSB preamble length in FSK mode
        _preamblelength = (p_length << 8) & 0xFFFF;
        p_length = sx1272_readRegister(REG_PREAMBLE_LSB_FSK);
        // Saving LSB preamble length in FSK mode
        _preamblelength = _preamblelength + (p_length & 0xFFFF);
#ifdef SX1272_DEBUG
        SerialPrint("## Preamble length configured is ");
        SerialPrintNr(_preamblelength, HEX);
        SerialPrint(" ##"NL);
        
#endif
    }
    state = 0;
    return state;
}

/*
 Function: Sets the preamble length in the module
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   l: length value to set as preamble length.
*/
uint8_t sx1272_setPreambleLength(uint16_t l)
{
    byte st0;
    uint8_t p_length;
    int8_t state = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setPreambleLength'"NL);
#endif

    st0 = sx1272_readRegister(REG_OP_MODE);	// Save the previous status
    state = 1;
    if( _modem == LORA )
    { // LoRa mode
        sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);    // Set Standby mode to write in registers
        p_length = ((l >> 8) & 0x0FF);
        // Storing MSB preamble length in LoRa mode
        sx1272_writeRegister(REG_PREAMBLE_MSB_LORA, p_length);
        p_length = (l & 0x0FF);
        // Storing LSB preamble length in LoRa mode
        sx1272_writeRegister(REG_PREAMBLE_LSB_LORA, p_length);
    }
    else
    { // FSK mode
        sx1272_writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);    // Set Standby mode to write in registers
        p_length = ((l >> 8) & 0x0FF);
        // Storing MSB preamble length in FSK mode
        sx1272_writeRegister(REG_PREAMBLE_MSB_FSK, p_length);
        p_length = (l & 0x0FF);
        // Storing LSB preamble length in FSK mode
        sx1272_writeRegister(REG_PREAMBLE_LSB_FSK, p_length);
    }

    state = 0;
#ifdef SX1272_DEBUG
    SerialPrint("## Preamble length ");
    SerialPrintNr(l, HEX);
    SerialPrint(" has been successfully set ##"NL);    
#endif

    sx1272_writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    delay_ms(100);
    return state;
}

/*
 Function: Gets the payload length from the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_getPayloadLength(void)
{
    uint8_t state = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'getPayloadLength'"NL);
#endif

    if( _modem == LORA )
    { // LORA mode
        // Saving payload length in LoRa mode
        _payloadlength = sx1272_readRegister(REG_PAYLOAD_LENGTH_LORA);
        state = 1;
    }
    else
    { // FSK mode
        // Saving payload length in FSK mode
        _payloadlength = sx1272_readRegister(REG_PAYLOAD_LENGTH_FSK);
        state = 1;
    }

#ifdef SX1272_DEBUG
    SerialPrint("## Payload length configured is ");
    SerialPrintNr(_payloadlength, HEX);
    SerialPrint(" ##"NL);    
#endif

    state = 0;
    return state;
}

/*
 Function: Sets the packet length in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   l: length value to set as payload length.
*/
int8_t sx1272_setPacketLength(uint8_t l)
{
    byte st0;
    byte value = 0x00;
    int8_t state = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setPacketLength'"NL);
#endif

    st0 = sx1272_readRegister(REG_OP_MODE);	// Save the previous status
    packet_sent.length = l;

    if( _modem == LORA )
    { // LORA mode
        sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);    // Set LoRa Standby mode to write in registers
        sx1272_writeRegister(REG_PAYLOAD_LENGTH_LORA, packet_sent.length);
        // Storing payload length in LoRa mode
        value = sx1272_readRegister(REG_PAYLOAD_LENGTH_LORA);
    }
    else
    { // FSK mode
        sx1272_writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);    //  Set FSK Standby mode to write in registers
        sx1272_writeRegister(REG_PAYLOAD_LENGTH_FSK, packet_sent.length);
        // Storing payload length in FSK mode
        value = sx1272_readRegister(REG_PAYLOAD_LENGTH_FSK);
    }

    if( packet_sent.length == value )
    {
        state = 0;
#ifdef SX1272_DEBUG
        SerialPrint("## Packet length ");
        SerialPrintNr(packet_sent.length, DEC);
        SerialPrint(" has been successfully set ##"NL);        
#endif
    }
    else
    {
        state = 1;
    }

    sx1272_writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    // comment by C. Pham
    // this delay_ms is included in the send delay_ms overhead
    // TODO: do we really need this delay_ms?
    delay_ms(250);
    return state;
}

/*
 Function: Gets the node address in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_getNodeAddress(void)
{
    byte st0 = 0;
    uint8_t state = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'getNodeAddress'"NL);
#endif

    if( _modem == LORA )
    { // LoRa mode
        st0 = sx1272_readRegister(REG_OP_MODE);	// Save the previous status
        // Allowing access to FSK registers while in LoRa standby mode
        sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_FSK_REGS_MODE);
    }

    // Saving node address
    _nodeAddress = sx1272_readRegister(REG_NODE_ADRS);
    state = 1;

    if( _modem == LORA )
    {
        sx1272_writeRegister(REG_OP_MODE, st0);		// Getting back to previous status
    }

    state = 0;
#ifdef SX1272_DEBUG
    SerialPrint("## Node address configured is ");
    SerialPrintNr(_nodeAddress, HEX);
    SerialPrint(" ##"NL);    
#endif
    return state;
}

/*
 Function: Sets the node address in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   addr: address value to set as node address.
*/
int8_t sx1272_setNodeAddress(uint8_t addr)
{
    byte st0;
    byte value;
    uint8_t state = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setNodeAddress'"NL);
#endif

    if( addr > 255 )
    {
        state = -1;
#ifdef SX1272_DEBUG
        SerialPrint("** Node address must be less than 255 **"NL);        
#endif
    }
    else
    {
        // Saving node address
        _nodeAddress = addr;
        st0 = sx1272_readRegister(REG_OP_MODE);	  // Save the previous status

        if( _modem == LORA )
        { // Allowing access to FSK registers while in LoRa standby mode
            sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_FSK_REGS_MODE);
        }
        else
        { //Set FSK Standby mode to write in registers
            sx1272_writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
        }

        // Storing node and broadcast address
        sx1272_writeRegister(REG_NODE_ADRS, addr);
        sx1272_writeRegister(REG_BROADCAST_ADRS, BROADCAST_0);

        value = sx1272_readRegister(REG_NODE_ADRS);
        sx1272_writeRegister(REG_OP_MODE, st0);		// Getting back to previous status

        if( value == _nodeAddress )
        {
            state = 0;
#ifdef SX1272_DEBUG
            SerialPrint("## Node address ");
            SerialPrintNr(addr, HEX);
            SerialPrint(" has been successfully set ##"NL);            
#endif
        }
        else
        {
            state = 1;
#ifdef SX1272_DEBUG
            SerialPrint("** There has been an error while setting address ##"NL);
            
#endif
        }
    }
    return state;
}

/*
 Function: Gets the SNR value in LoRa mode.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t sx1272_getSNR(void)
{	// getSNR exists only in LoRa mode
    int8_t state = 2;
    byte value;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'getSNR'"NL);
#endif

    if( _modem == LORA )
    { // LoRa mode
        state = 1;
        value = sx1272_readRegister(REG_PKT_SNR_VALUE);
        if( value & 0x80 ) // The SNR sign bit is 1
        {
            // Invert and divide by 4
            value = ( ( ~value + 1 ) & 0xFF ) >> 2;
            _SNR = -value;
        }
        else
        {
            // Divide by 4
            _SNR = ( value & 0xFF ) >> 2;
        }
        state = 0;
#ifdef SX1272_DEBUG
        SerialPrint("## SNR value is ");
        SerialPrintNr(_SNR, DEC);
        SerialPrint(" ##"NL);        
#endif
    }
    else
    { // forbidden command if FSK mode
        state = -1;
#ifdef SX1272_DEBUG
        SerialPrint("** SNR does not exist in FSK mode **"NL);        
#endif
    }
    return state;
}

/*
 Function: Gets the current value of RSSI.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_getRSSI(void)
{
    uint8_t state = 2;
    int rssi_mean = 0;
    int total = 5;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'getRSSI'"NL);
#endif

    if( _modem == LORA )
    {
        /// LoRa mode
        // get mean value of RSSI
        int i;
        for(i = 0; i < total; i++)
        {
            // modified by C. Pham
            // with SX1276 we have to add 20 to OFFSET_RSSI
            _RSSI = -(OFFSET_RSSI+(_board==SX1276Chip?20:0)) + sx1272_readRegister(REG_RSSI_VALUE_LORA);
            rssi_mean += _RSSI;
        }

        rssi_mean = rssi_mean / total;
        _RSSI = rssi_mean;

        state = 0;
#ifdef SX1272_DEBUG
        SerialPrint("## RSSI value is ");
        SerialPrintNr(_RSSI, DEC);
        SerialPrint(" ##"NL);        
#endif
    }
    else
    {
        /// FSK mode
        // get mean value of RSSI
        int i;
        for(i = 0; i < total; i++)
        {
            _RSSI = (sx1272_readRegister(REG_RSSI_VALUE_FSK) >> 1);
            rssi_mean += _RSSI;
        }
        rssi_mean = rssi_mean / total;
        _RSSI = rssi_mean;

        state = 0;

#ifdef SX1272_DEBUG
        SerialPrint("## RSSI value is ");
        SerialPrintNr(_RSSI, DEC);
        SerialPrint(" ##"NL);        
#endif
    }
    return state;
}

/*
 Function: Gets the RSSI of the last packet received in LoRa mode.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int16_t sx1272_getRSSIpacket(void)
{	// RSSIpacket only exists in LoRa
    int8_t state = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'getRSSIpacket'");
#endif

    state = 1;
    if( _modem == LORA )
    { // LoRa mode
        state = sx1272_getSNR();
        if( state == 0 )
        {
            // added by C. Pham
            _RSSIpacket = sx1272_readRegister(REG_PKT_RSSI_VALUE);

            if( _SNR < 0 )
            {
                // commented by C. Pham
                //_RSSIpacket = -NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[_bandwidth] + NOISE_FIGURE + ( double )_SNR;

                // added by C. Pham, using Semtech SX1272 rev3 March 2015
                _RSSIpacket = -(OFFSET_RSSI+(_board==SX1276Chip?20:0)) + (double)_RSSIpacket + (double)_SNR*0.25;
                state = 0;
            }
            else
            {
                // commented by C. Pham
                //_RSSIpacket = sx1272_readRegister(REG_PKT_RSSI_VALUE);
                _RSSIpacket = -(OFFSET_RSSI+(_board==SX1276Chip?20:0)) + (double)_RSSIpacket;
                //end
                state = 0;
            }
#ifdef SX1272_DEBUG
            SerialPrint("## RSSI packet value is ");
            SerialPrintNr(_RSSIpacket, DEC);
            SerialPrint(" ##"NL);            
#endif
        }
    }
    else
    { // RSSI packet doesn't exist in FSK mode
        state = -1;
#ifdef SX1272_DEBUG
        SerialPrint("** RSSI packet does not exist in FSK mode **"NL);        
#endif
    }
    return state;
}

/*
 Function: It sets the maximum number of retries.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 -->
*/
uint8_t sx1272_setRetries(uint8_t ret)
{
    uint8_t state = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setRetries'"NL);
#endif

    state = 1;
    if( ret > MAX_RETRIES )
    {
        state = -1;
#ifdef SX1272_DEBUG
        SerialPrint("** Retries value can't be greater than ");
        SerialPrintNr(MAX_RETRIES, DEC);
        SerialPrint(" **"NL);        
#endif
    }
    else
    {
        _maxRetries = ret;
        state = 0;
#ifdef SX1272_DEBUG
        SerialPrint("## Maximum retries value = ");
        SerialPrintNr(_maxRetries, DEC);
        SerialPrint(" ##"NL);        
#endif
    }
    return state;
}

/*
 Function: Gets the current supply limit of the power amplifier, protecting battery chemistries.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   rate: value to compute the maximum current supply. Maximum current is 45+5*'rate' [mA]
*/
uint8_t sx1272_getMaxCurrent(void)
{
    int8_t state = 2;
    byte value;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'getMaxCurrent'"NL);
#endif

    state = 1;
    _maxCurrent = sx1272_readRegister(REG_OCP);

    // extract only the OcpTrim value from the OCP register
    _maxCurrent &= 0b00011111;

    if( _maxCurrent <= 15 )
    {
        value = (45 + (5 * _maxCurrent));
    }
    else if( _maxCurrent <= 27 )
    {
        value = (-30 + (10 * _maxCurrent));
    }
    else
    {
        value = 240;
    }

    _maxCurrent = value;
#ifdef SX1272_DEBUG
    SerialPrint("## Maximum current supply configured is ");
    SerialPrintNr(value, DEC);
    SerialPrint(" mA ##"NL);    
#endif
    state = 0;
    return state;
}

/*
 Function: Limits the current supply of the power amplifier, protecting battery chemistries.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden parameter value for this function
 Parameters:
   rate: value to compute the maximum current supply. Maximum current is 45+5*'rate' [mA]
*/
int8_t sx1272_setMaxCurrent(uint8_t rate)
{
    int8_t state = 2;
    byte st0;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setMaxCurrent'"NL);
#endif

    // Maximum rate value = 0x1B, because maximum current supply = 240 mA
    if (rate > 0x1B)
    {
        state = -1;
#ifdef SX1272_DEBUG
        SerialPrint("** Maximum current supply is 240 mA, "NL);
        SerialPrint("so maximum parameter value must be 27 (DEC) or 0x1B (HEX) **"NL);        
#endif
    }
    else
    {
        // Enable Over Current Protection
        rate |= 0b00100000;

        state = 1;
        st0 = sx1272_readRegister(REG_OP_MODE);	// Save the previous status
        if( _modem == LORA )
        { // LoRa mode
            sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Set LoRa Standby mode to write in registers
        }
        else
        { // FSK mode
            sx1272_writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Set FSK Standby mode to write in registers
        }
        sx1272_writeRegister(REG_OCP, rate);		// Modifying maximum current supply
        sx1272_writeRegister(REG_OP_MODE, st0);		// Getting back to previous status
        state = 0;
    }
    return state;
}

/*
 Function: Gets the content of different registers.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_getRegs(void)
{
    int8_t state = 2;
    uint8_t state_f = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'getRegs'"NL);
#endif

    state_f = 1;
    state = sx1272_getMode();			// Stores the BW, CR and SF.
    if( state == 0 )
    {
        state = sx1272_getPower();		// Stores the power.
    }
    else
    {
        state_f = 1;
#ifdef SX1272_DEBUG
        SerialPrint("** Error getting mode **"NL);
#endif
    }
    if( state == 0 )
    {
        state = sx1272_getChannel();	// Stores the channel.
    }
    else
    {
        state_f = 1;
#ifdef SX1272_DEBUG
        SerialPrint("** Error getting power **"NL);
#endif
    }
    if( state == 0 )
    {
        state = sx1272_getCRC();		// Stores the CRC configuration.
    }
    else
    {
        state_f = 1;
#ifdef SX1272_DEBUG
        SerialPrint("** Error getting channel **"NL);
#endif
    }
    if( state == 0 )
    {
        state = sx1272_getHeader();	// Stores the header configuration.
    }
    else
    {
        state_f = 1;
#ifdef SX1272_DEBUG
        SerialPrint("** Error getting CRC **"NL);
#endif
    }
    if( state == 0 )
    {
        state = sx1272_getPreambleLength();	// Stores the preamble length.
    }
    else
    {
        state_f = 1;
#ifdef SX1272_DEBUG
        SerialPrint("** Error getting header **"NL);
#endif
    }
    if( state == 0 )
    {
        state = sx1272_getPayloadLength();		// Stores the payload length.
    }
    else
    {
        state_f = 1;
#ifdef SX1272_DEBUG
        SerialPrint("** Error getting preamble length **"NL);
#endif
    }
    if( state == 0 )
    {
        state = sx1272_getNodeAddress();		// Stores the node address.
    }
    else
    {
        state_f = 1;
#ifdef SX1272_DEBUG
        SerialPrint("** Error getting payload length **"NL);
#endif
    }
    if( state == 0 )
    {
        state = sx1272_getMaxCurrent();		// Stores the maximum current supply.
    }
    else
    {
        state_f = 1;
#ifdef SX1272_DEBUG
        SerialPrint("** Error getting node address **"NL);
#endif
    }
    if( state == 0 )
    {
        state_f = sx1272_getTemp();		// Stores the module temperature.
    }
    else
    {
        state_f = 1;
#ifdef SX1272_DEBUG
        SerialPrint("** Error getting maximum current supply **"NL);
#endif
    }
    if( state_f != 0 )
    {
#ifdef SX1272_DEBUG
        SerialPrint("** Error getting temperature **"NL);        
#endif
    }
    return state_f;
}

/*
 Function: It truncs the payload length if it is greater than 0xFF.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_truncPayload(uint16_t length16)
{
    uint8_t state = 2;

    state = 1;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'truncPayload'"NL);
#endif

    if( length16 > MAX_PAYLOAD )
    {
        _payloadlength = MAX_PAYLOAD;
    }
    else
    {
        _payloadlength = (length16 & 0xFF);
    }
    state = 0;

    return state;
}

/*
 Function: It sets an ACK in FIFO in order to send it.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_setACK(void)
{
    uint8_t state = 2;

    //#ifdef SX1272_DEBUG
    
    SerialPrint("Starting 'setACK'"NL);
    //#endif

    // delay_ms(1000);

    sx1272_clearFlags();	// Initializing flags

    if( _modem == LORA )
    { // LoRa mode
        sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Stdby LoRa mode to write in FIFO
    }
    else
    { // FSK mode
        sx1272_writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Stdby FSK mode to write in FIFO
    }

    // Setting ACK length in order to send it
    state = sx1272_setPacketLength(ACK_LENGTH);
    if( state == 0 )
    {
        // Setting ACK
        ACK.dst = packet_received.src; // ACK destination is packet source
        ACK.type = PKT_TYPE_ACK;
        ACK.src = packet_received.dst; // ACK source is packet destination
        ACK.packnum = packet_received.packnum; // packet number that has been correctly received
        ACK.length = 2;
        ACK.data[0] = _reception;	// CRC of the received packet
        // added by C. Pham
        // store the SNR
        ACK.data[1]= sx1272_readRegister(REG_PKT_SNR_VALUE);

        // Setting address pointer in FIFO data buffer
        sx1272_writeRegister(REG_FIFO_ADDR_PTR, 0x80);

        state = 1;

        // Writing ACK to send in FIFO
        sx1272_writeRegister(REG_FIFO, ACK.dst); 		// Writing the destination in FIFO
        sx1272_writeRegister(REG_FIFO, ACK.type);
        sx1272_writeRegister(REG_FIFO, ACK.src);		// Writing the source in FIFO
        sx1272_writeRegister(REG_FIFO, ACK.packnum);	// Writing the packet number in FIFO
        sx1272_writeRegister(REG_FIFO, ACK.length); 	// Writing the packet length in FIFO
        sx1272_writeRegister(REG_FIFO, ACK.data[0]);	// Writing the ACK in FIFO
        sx1272_writeRegister(REG_FIFO, ACK.data[1]);	// Writing the ACK in FIFO

        //#ifdef SX1272_DEBUG
        SerialPrint("## ACK set and written in FIFO ##"NL);
        // Print the complete ACK if debug_mode
        SerialPrint("## ACK to send:"NL);
        SerialPrint("Destination: ");
        SerialPrintNr(ACK.dst, HEX);			 	// Printing destination
        SerialPrint(NL"Source: ");
        SerialPrintNr(ACK.src, HEX);			 	// Printing source
        SerialPrint(NL"ACK number: ");
        SerialPrintNr(ACK.packnum, DEC);			// Printing ACK number
        SerialPrint(NL"ACK length: ");
        SerialPrintNr(ACK.length, DEC);				// Printing ACK length
        SerialPrint(NL"ACK payload: ");
        SerialPrintNr(ACK.data[0], HEX);			// Printing ACK payload
        SerialPrint(NL"ACK SNR last rcv pkt: "NL);
        SerialPrintNr(_SNR, DEC);
        SerialPrint("##"NL);        
        //#endif

        state = 0;
        _reception = CORRECT_PACKET;		// Updating value to next packet

        // comment by C. Pham
        // TODO: do we really need this delay_ms?
        delay_ms(500);
    }
    return state;
}

/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_receive(void)
{
    uint8_t state = 1;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'receive'"NL);
#endif

    // Initializing packet_received struct
    memset( &packet_received, 0x00, sizeof(packet_received) );

    // Setting Testmode
    // commented by C. Pham
    //sx1272_writeRegister(0x31,0x43);

    // Set LowPnTxPllOff
    // modified by C. Pham from 0x09 to 0x08
    sx1272_writeRegister(REG_PA_RAMP, 0x08);

    //sx1272_writeRegister(REG_LNA, 0x23);			// Important in reception
    // modified by C. Pham
    sx1272_writeRegister(REG_LNA, LNA_MAX_GAIN);
    sx1272_writeRegister(REG_FIFO_ADDR_PTR, 0x00);  // Setting address pointer in FIFO data buffer
    // change RegSymbTimeoutLsb
    // comment by C. Pham
    // single_chan_pkt_fwd uses 00 00001000
    // why here we have 11 11111111
    // change RegSymbTimeoutLsb
    //sx1272_writeRegister(REG_SYMB_TIMEOUT_LSB, 0xFF);

    // modified by C. Pham
    if (_spreadingFactor == SF_10 || _spreadingFactor == SF_11 || _spreadingFactor == SF_12) {
        sx1272_writeRegister(REG_SYMB_TIMEOUT_LSB,0x05);
    } else {
        sx1272_writeRegister(REG_SYMB_TIMEOUT_LSB,0x08);
    }
    //end

    sx1272_writeRegister(REG_FIFO_RX_BYTE_ADDR, 0x00); // Setting current value of reception buffer pointer
    //sx1272_clearFlags();						// Initializing flags
    //state = 1;
    if( _modem == LORA )
    { // LoRa mode
        state = sx1272_setPacketLength(MAX_LENGTH);	// With MAX_LENGTH gets all packets with length < MAX_LENGTH
        sx1272_writeRegister(REG_OP_MODE, LORA_RX_MODE);  	  // LORA mode - Rx
#ifdef SX1272_DEBUG
        SerialPrint("## Receiving LoRa mode activated with success ##"NL);        
#endif
    }
    else
    { // FSK mode
        state = sx1272_setPacketLength(_rawFormat ? _payloadlength : _payloadlength+OFFSET_PAYLOADLENGTH);
        sx1272_writeRegister(REG_OP_MODE, FSK_RX_MODE);  // FSK mode - Rx
#ifdef SX1272_DEBUG
        SerialPrint("## Receiving FSK mode activated with success ##"NL);        
#endif
    }
    return state;
}

/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_receivePacketMAXTimeout(void)
{
    return sx1272_sx1272_receivePacketTimeout(MAX_TIMEOUT);
}

/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_receivePacketTimeout2(void)
{
    sx1272_setTimeout();
    return sx1272_receivePacketTimeout(_sendTime);
}

/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/

#ifdef W_REQUESTED_ACK
// added by C. Pham
// receiver always use sx1272_receivePacketTimeout2()
// sender should either use sx1272_sendPacketTimeout() or sx1272_sendPacketTimeoutACK()
uint8_t sx1272_receivePacketTimeout(uint16_t wait)
{
    uint8_t state = 2;
    uint8_t state_f = 2;


#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'receivePacketTimeoutACK'"NL);
#endif

    state = sx1272_receive();
    if( state == 0 )
    {
        if( sx1272_availableData(wait) )
        {
            state = sx1272_getPacket(MAX_TIMEOUT);
        }
        else
        {
            state = 1;
            state_f = 3;  // There is no packet received
        }
    }
    else
    {
        state = 1;
        state_f = 1; // There has been an error with the 'receive' function
    }

    if( (state == 0) || (state == 3) || (state == 5) )
    {
        if( _reception == INCORRECT_PACKET )
        {
            state_f = 4;  // The packet has been incorrectly received
        }
        else
        {
            state_f = 0;  // The packet has been correctly received
            // added by C. Pham
            // we get the SNR and RSSI of the received packet for future usage
            sx1272_getSNR();
            sx1272_getRSSIpacket();
        }

        // need to send an ACK
        if ( state == 5 && state_f == 0) {

            state = sx1272_setACK();

            if( state == 0 )
            {
                state = sx1272_sendWithTimeout2();
                if( state == 0 )
                {
                    state_f = 0;
#ifdef SX1272_DEBUG
                    SerialPrint("This last packet was an ACK, so ..."NL);
                    SerialPrint("ACK successfully sent"NL);                    
#endif
                }
                else
                {
                    state_f = 1; // There has been an error with the 'sendWithTimeout' function
                }
            }
            else
            {
                state_f = 1; // There has been an error with the 'setACK' function
            }
        }
    }
    else
    {
        state_f = 1;
    }
    return state_f;
}
#else

uint8_t sx1272_receivePacketTimeout(uint16_t wait)
{
    uint8_t state = 2;
    uint8_t state_f = 2;

#ifdef SX1272_DEBUG
    
    SerialPrint("Starting 'receivePacketTimeout'"NL);
#endif

    state = sx1272_receive();
    if( state == 0 )
    {
        if( sx1272_availableData(wait) )
        {
            // If packet received, getPacket
            state_f = sx1272_getPacket(MAX_TIMEOUT);
        }
        else
        {
            state_f = 1;
        }
    }
    else
    {
        state_f = state;
    }
    return state_f;
}
#endif

/*
 Function: Configures the module to receive information and send an ACK.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_receivePacketMAXTimeoutACK(void)
{
    return sx1272_receivePacketTimeoutACK(MAX_TIMEOUT);
}

/*
 Function: Configures the module to receive information and send an ACK.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_receivePacketTimeoutACK2(void)
{
    sx1272_setTimeout();
    return sx1272_receivePacketTimeoutACK(_sendTime);
}

/*
 Function: Configures the module to receive information and send an ACK.
 Returns: Integer that determines if there has been any error
   state = 4  --> The command has been executed but the packet received is incorrect
   state = 3  --> The command has been executed but there is no packet received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_receivePacketTimeoutACK(uint16_t wait)
{
    uint8_t state = 2;
    uint8_t state_f = 2;


#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'receivePacketTimeoutACK'"NL);
#endif

    state = sx1272_receive();
    if( state == 0 )
    {
        if( sx1272_availableData(wait) )
        {
            state = sx1272_getPacket(MAX_TIMEOUT);
        }
        else
        {
            state = 1;
            state_f = 3;  // There is no packet received
        }
    }
    else
    {
        state = 1;
        state_f = 1; // There has been an error with the 'receive' function
    }
    if( (state == 0) || (state == 3) )
    {
        if( _reception == INCORRECT_PACKET )
        {
            state_f = 4;  // The packet has been incorrectly received
        }
        else
        {
            state_f = 1;  // The packet has been correctly received
        }
        state = sx1272_setACK();
        if( state == 0 )
        {
            state = sx1272_sendWithTimeout2();
            if( state == 0 )
            {
                state_f = 0;
#ifdef SX1272_DEBUG
                SerialPrint("This last packet was an ACK, so ..."NL);
                SerialPrint("ACK successfully sent"NL);                
#endif
            }
            else
            {
                state_f = 1; // There has been an error with the 'sendWithTimeout' function
            }
        }
        else
        {
            state_f = 1; // There has been an error with the 'setACK' function
        }
    }
    else
    {
        state_f = 1;
    }
    return state_f;
}

/*
 Function: Configures the module to receive all the information on air.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_receiveAll(uint16_t wait)
{
    uint8_t state = 2;
    byte config1;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'receiveAll'"NL);
#endif

    if( _modem == FSK )
    { // FSK mode
        sx1272_writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);		// Setting standby FSK mode
        config1 = sx1272_readRegister(REG_PACKET_CONFIG1);
        config1 = config1 & 0b11111001;			// clears bits 2-1 from REG_PACKET_CONFIG1
        sx1272_writeRegister(REG_PACKET_CONFIG1, config1);		// AddressFiltering = None
    }
#ifdef SX1272_DEBUG
    SerialPrint("## Address filtering desactivated ##"NL);    
#endif
    state = sx1272_receive();	// Setting Rx mode
    if( state == 0 )
    {
        state = sx1272_getPacket(wait);	// Getting all packets received in wait
    }
    return state;
}

/*
 Function: If a packet is received, checks its destination.
 Returns: Boolean that's 'TRUE' if the packet is for the module and
          it's 'FALSE' if the packet is not for the module.
 Parameters:
   wait: time to wait while there is no a valid header received.
*/
boolean	sx1272_availableData(uint16_t wait)
{
    byte value;
    byte header = 0;
    boolean forme = FALSE;
    boolean	_hreceived = FALSE;
    unsigned long previous;


#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'availableData'"NL);
#endif

    previous = millis();
    if( _modem == LORA )
    { // LoRa mode
        value = sx1272_readRegister(REG_IRQ_FLAGS);
        // Wait to ValidHeader interrupt
        while( (bitRead(value, 4) == 0) && (millis() - previous < (unsigned long)wait) )
        {
            value = sx1272_readRegister(REG_IRQ_FLAGS);
            // Condition to avoid an overflow (DO NOT REMOVE)
            if( millis() < previous )
            {
                previous = millis();
            }
        } // end while (millis)

        if( bitRead(value, 4) == 1 )
        { // header received
#ifdef SX1272_DEBUG
            SerialPrint("## Valid Header received in LoRa mode ##"NL);
#endif
            _hreceived = TRUE;

#ifdef W_NET_KEY
            // actually, need to wait until 3 bytes have been received
            while( (header < 3) && (millis() - previous < (unsigned long)wait) )
#else
            while( (header == 0) && (millis() - previous < (unsigned long)wait) )
#endif
            { // Waiting to read first payload bytes from packet
                header = sx1272_readRegister(REG_FIFO_RX_BYTE_ADDR);
                // Condition to avoid an overflow (DO NOT REMOVE)
                if( millis() < previous )
                {
                    previous = millis();
                }
            }

            if( header != 0 )
            { // Reading first byte of the received packet
#ifdef W_NET_KEY
                // added by C. Pham
                // if we actually wait for an ACK, there is no net key before ACK data
                if (_requestACK==0) {
                    _the_net_key_0 = sx1272_readRegister(REG_FIFO);
                    _the_net_key_1 = sx1272_readRegister(REG_FIFO);
                }
#endif
                _destination = sx1272_readRegister(REG_FIFO);
            }
        }
        else
        {
            forme = FALSE;
            _hreceived = FALSE;
#ifdef SX1272_DEBUG
            SerialPrint("** The timeout has expired **"NL);            
#endif
        }
    }
    else
    { // FSK mode
        value = sx1272_readRegister(REG_IRQ_FLAGS2);
        // Wait to Payload Ready interrupt
        while( (bitRead(value, 2) == 0) && (millis() - previous < wait) )
        {
            value = sx1272_readRegister(REG_IRQ_FLAGS2);
            // Condition to avoid an overflow (DO NOT REMOVE)
            if( millis() < previous )
            {
                previous = millis();
            }
        }// end while (millis)
        if( bitRead(value, 2) == 1 )	// something received
        {
            _hreceived = TRUE;
#ifdef SX1272_DEBUG
            SerialPrint("## Valid Preamble detected in FSK mode ##"NL);
#endif
            // Reading first byte of the received packet
            _destination = sx1272_readRegister(REG_FIFO);
        }
        else
        {
            forme = FALSE;
            _hreceived = FALSE;
#ifdef SX1272_DEBUG
            SerialPrint("** The timeout has expired **"NL);
            
#endif
        }
    }
    // We use _hreceived because we need to ensure that _destination value is correctly
    // updated and is not the _destination value from the previously packet
    if( _hreceived == TRUE )
    { // Checking destination
#ifdef SX1272_DEBUG
        SerialPrint("## Checking destination ##"NL);
#endif

        // added by C. Pham
#ifdef W_NET_KEY
        forme=TRUE;

        // if we wait for an ACK, then we do not check for net key
        if (_requestACK==0)
            if (_the_net_key_0!=_my_netkey[0] || _the_net_key_1!=_my_netkey[1]) {
                //#ifdef SX1272_DEBUG
                SerialPrint("## Wrong net key ##"NL);
                //#endif
                forme=FALSE;
            }
            else
            {
                //#ifdef SX1272_DEBUG
                SerialPrint("## Good net key ##"NL);
                //#endif
            }


        if( forme && ((_destination == _nodeAddress) || (_destination == BROADCAST_0)) )
#else
        // modified by C. Pham
        // if _rawFormat, accept all
        if( (_destination == _nodeAddress) || (_destination == BROADCAST_0) || _rawFormat)
#endif
        { // LoRa or FSK mode
            forme = TRUE;
#ifdef SX1272_DEBUG
            SerialPrint("## Packet received is for me ##"NL);
#endif
        }
        else
        {
            forme = FALSE;
#ifdef SX1272_DEBUG
            SerialPrint("## Packet received is not for me ##"NL);            
#endif
            if( _modem == LORA )	// STANDBY PARA MINIMIZAR EL CONSUMO
            { // LoRa mode
                //sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Setting standby LoRa mode
            }
            else
            { //  FSK mode
                sx1272_writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Setting standby FSK mode
            }
        }
    }
    //----else
    //	{
    //	}
    return forme;
}

/*
 Function: It gets and stores a packet if it is received before MAX_TIMEOUT expires.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_getPacketMAXTimeout(void)
{
    return sx1272_getPacket(MAX_TIMEOUT);
}

/*
 Function: It gets and stores a packet if it is received before ending 'wait' time.
 Returns:  Integer that determines if there has been any error
   // added by C. Pham
   state = 5  --> The command has been executed with no errors and an ACK is requested
   state = 3  --> The command has been executed but packet has been incorrectly received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden parameter value for this function
 Parameters:
   wait: time to wait while there is no a valid header received.
*/
int8_t sx1272_getPacket(uint16_t wait)
{
    uint8_t state = 2;
    byte value = 0x00;
    unsigned long previous;
    boolean p_received = FALSE;
    unsigned int i;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'getPacket'"NL);
#endif

    previous = millis();
    if( _modem == LORA )
    { // LoRa mode
        value = sx1272_readRegister(REG_IRQ_FLAGS);
        // Wait until the packet is received (RxDone flag) or the timeout expires
        while( (bitRead(value, 6) == 0) && (millis() - previous < (unsigned long)wait) )
        {
            value = sx1272_readRegister(REG_IRQ_FLAGS);
            // Condition to avoid an overflow (DO NOT REMOVE)
            if( millis() < previous )
            {
                previous = millis();
            }
        } // end while (millis)

        if( (bitRead(value, 6) == 1) && (bitRead(value, 5) == 0) )
        { // packet received & CRC correct
            p_received = TRUE;	// packet correctly received
            _reception = CORRECT_PACKET;
#ifdef SX1272_DEBUG
            SerialPrint("## Packet correctly received in LoRa mode ##"NL);
#endif
        }
        else
        {
            if( bitRead(value, 5) != 0 )
            { // CRC incorrect
                _reception = INCORRECT_PACKET;
                state = 3;
#ifdef SX1272_DEBUG
                SerialPrint("** The CRC is incorrect **"NL);                
#endif
            }
        }
        //sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Setting standby LoRa mode
    }
    else
    { // FSK mode
        value = sx1272_readRegister(REG_IRQ_FLAGS2);
        while( (bitRead(value, 2) == 0) && (millis() - previous < wait) )
        {
            value = sx1272_readRegister(REG_IRQ_FLAGS2);
            // Condition to avoid an overflow (DO NOT REMOVE)
            if( millis() < previous )
            {
                previous = millis();
            }
        } // end while (millis)
        if( bitRead(value, 2) == 1 )
        { // packet received
            if( bitRead(value, 1) == 1 )
            { // CRC correct
                _reception = CORRECT_PACKET;
                p_received = TRUE;
#ifdef SX1272_DEBUG
                SerialPrint("## Packet correctly received in FSK mode ##"NL);
#endif
            }
            else
            { // CRC incorrect
                _reception = INCORRECT_PACKET;
                state = 3;
                p_received = FALSE;
#ifdef SX1272_DEBUG
                SerialPrint("## Packet incorrectly received in FSK mode ##"NL);
#endif
            }
        }
        else
        {
#ifdef SX1272_DEBUG
            SerialPrint("** The timeout has expired **"NL);
            
#endif
        }
        sx1272_writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Setting standby FSK mode
    }
    if( p_received == TRUE )
    {
        // Store the packet
        if( _modem == LORA )
        {
            // comment by C. Pham
            // set the FIFO addr to 0 to read again all the bytes
            sx1272_writeRegister(REG_FIFO_ADDR_PTR, 0x00);  	// Setting address pointer in FIFO data buffer

#ifdef W_NET_KEY
            // added by C. Pham
            packet_received.netkey[0]=sx1272_readRegister(REG_FIFO);
            packet_received.netkey[1]=sx1272_readRegister(REG_FIFO);
#endif
            //modified by C. Pham
            if (!_rawFormat)
                packet_received.dst = sx1272_readRegister(REG_FIFO);	// Storing first byte of the received packet
            else
                packet_received.dst = 0;
        }
        else
        {
            value = sx1272_readRegister(REG_PACKET_CONFIG1);
            if( (bitRead(value, 2) == 0) && (bitRead(value, 1) == 0) )
            {
                packet_received.dst = sx1272_readRegister(REG_FIFO); // Storing first byte of the received packet
            }
            else
            {
                packet_received.dst = _destination;			// Storing first byte of the received packet
            }
        }

        // modified by C. Pham
        if (!_rawFormat) {
            packet_received.type = sx1272_readRegister(REG_FIFO);		// Reading second byte of the received packet
            packet_received.src = sx1272_readRegister(REG_FIFO);		// Reading second byte of the received packet
            packet_received.packnum = sx1272_readRegister(REG_FIFO);	// Reading third byte of the received packet
            //packet_received.length = sx1272_readRegister(REG_FIFO);	// Reading fourth byte of the received packet
        }
        else {
            packet_received.type = 0;
            packet_received.src = 0;
            packet_received.packnum = 0;
        }

        packet_received.length = sx1272_readRegister(REG_RX_NB_BYTES);

        if( _modem == LORA )
        {
            if (_rawFormat) {
                _payloadlength=packet_received.length;
            }
            else
                _payloadlength = packet_received.length - OFFSET_PAYLOADLENGTH;
        }
        if( packet_received.length > (MAX_LENGTH + 1) )
        {
#ifdef SX1272_DEBUG
            SerialPrint("Corrupted packet, length must be less than 256"NL);
#endif
        }
        else
        {
            for(i = 0; i < _payloadlength; i++)
            {
                packet_received.data[i] = sx1272_readRegister(REG_FIFO); // Storing payload
            }

            // commented by C. Pham
            //packet_received.retry = sx1272_readRegister(REG_FIFO);

            // Print the packet if debug_mode
#ifdef SX1272_DEBUG
            SerialPrint("## Packet received:"NL);
            SerialPrint("Destination: ");
            SerialPrintNr(packet_received.dst, HEX);			 	// Printing destination
            SerialPrint(NL"Type: ");
            SerialPrintNr(packet_received.type, DEC);			 	// Printing source
            SerialPrint(NL"Source: ");
            SerialPrintNr(packet_received.src, HEX);			 	// Printing source
            SerialPrint(NL"Packet number: ");
            SerialPrintNr(packet_received.packnum, DEC);			// Printing packet number
            //SerialPrint("Packet length: "NL);
            //SerialPrintNr(packet_received.length);                // Printing packet length
            SerialPrint(NL"Data: ");
            for(i = 0; i < _payloadlength; i++)
            {
                SerialPrintNr((char)packet_received.data[i], HEX);	// Printing payload
            }
            
            //SerialPrint("Retry number: "NL);
            //SerialPrintNr(packet_received.retry);                 // Printing number retry
            SerialPrint("##"NL);
            
#endif
            state = 0;

#ifdef W_REQUESTED_ACK
            // added by C. Pham
            // need to send an ACK
            if (packet_received.type & PKT_FLAG_ACK_REQ) {
                state = 5;
                _requestACK_indicator=1;
            }
            else
                _requestACK_indicator=0;
#endif
        }
    }
    else
    {
        state = 1;
        if( (_reception == INCORRECT_PACKET) && (_retries < _maxRetries) )
        {
            // comment by C. Pham
            // what is the purpose of incrementing retries here?
            // bug? not needed?
            _retries++;
#ifdef SX1272_DEBUG
            SerialPrint("## Retrying to send the last packet ##"NL);            
#endif
        }
    }
    if( _modem == LORA )
    {
        sx1272_writeRegister(REG_FIFO_ADDR_PTR, 0x00);  // Setting address pointer in FIFO data buffer
    }
    sx1272_clearFlags();	// Initializing flags
    if( wait > MAX_WAIT )
    {
        state = -1;
#ifdef SX1272_DEBUG
        SerialPrint("** The timeout must be smaller than 12.5 seconds **"NL);        
#endif
    }

    return state;
}

/*
 Function: It sets the packet destination.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   dest: destination value of the packet sent.
*/
int8_t sx1272_setDestination(uint8_t dest)
{
    int8_t state = 2;

#ifdef SX1272_DEBUG
    
    SerialPrint("Starting 'setDestination'"NL);
#endif

    state = 1;
    _destination = dest; // Storing destination in a global variable
    packet_sent.dst = dest;	 // Setting destination in packet structure
    packet_sent.src = _nodeAddress; // Setting source in packet structure
    packet_sent.packnum = _packetNumber;	// Setting packet number in packet structure
    _packetNumber++;
    state = 0;

#ifdef SX1272_DEBUG
    SerialPrint("## Destination ");
    SerialPrintNr(_destination, HEX);
    SerialPrint(" successfully set ##"NL);
    SerialPrint("## Source "NL);
    SerialPrintNr(packet_sent.src, DEC);
    SerialPrint(" successfully set ##"NL);
    SerialPrint("## Packet number ");
    SerialPrintNr(packet_sent.packnum, DEC);
    SerialPrint(" successfully set ##"NL);
    
#endif
    return state;
}

/*
 Function: It sets the timeout according to the configured mode.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_setTimeout(void)
{
    uint8_t state = 2;
    uint16_t delay_ms;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setTimeout'"NL);
#endif

    state = 1;
    if( _modem == LORA )
    {
        switch(_spreadingFactor)
        {	// Choosing Spreading Factor
        case SF_6:	switch(_bandwidth)
            {	// Choosing bandwidth
            case BW_125:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 335;
                    break;
                case CR_6: _sendTime = 352;
                    break;
                case CR_7: _sendTime = 368;
                    break;
                case CR_8: _sendTime = 386;
                    break;
                }
                break;
            case BW_250:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 287;
                    break;
                case CR_6: _sendTime = 296;
                    break;
                case CR_7: _sendTime = 305;
                    break;
                case CR_8: _sendTime = 312;
                    break;
                }
                break;
            case BW_500:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 242;
                    break;
                case CR_6: _sendTime = 267;
                    break;
                case CR_7: _sendTime = 272;
                    break;
                case CR_8: _sendTime = 276;
                    break;
                }
                break;
            }
            break;

        case SF_7:	switch(_bandwidth)
            {	// Choosing bandwidth
            case BW_125:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 408;
                    break;
                case CR_6: _sendTime = 438;
                    break;
                case CR_7: _sendTime = 468;
                    break;
                case CR_8: _sendTime = 497;
                    break;
                }
                break;
            case BW_250:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 325;
                    break;
                case CR_6: _sendTime = 339;
                    break;
                case CR_7: _sendTime = 355;
                    break;
                case CR_8: _sendTime = 368;
                    break;
                }
                break;
            case BW_500:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 282;
                    break;
                case CR_6: _sendTime = 290;
                    break;
                case CR_7: _sendTime = 296;
                    break;
                case CR_8: _sendTime = 305;
                    break;
                }
                break;
            }
            break;

        case SF_8:	switch(_bandwidth)
            {	// Choosing bandwidth
            case BW_125:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 537;
                    break;
                case CR_6: _sendTime = 588;
                    break;
                case CR_7: _sendTime = 640;
                    break;
                case CR_8: _sendTime = 691;
                    break;
                }
                break;
            case BW_250:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 388;
                    break;
                case CR_6: _sendTime = 415;
                    break;
                case CR_7: _sendTime = 440;
                    break;
                case CR_8: _sendTime = 466;
                    break;
                }
                break;
            case BW_500:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 315;
                    break;
                case CR_6: _sendTime = 326;
                    break;
                case CR_7: _sendTime = 340;
                    break;
                case CR_8: _sendTime = 352;
                    break;
                }
                break;
            }
            break;

        case SF_9:	switch(_bandwidth)
            {	// Choosing bandwidth
            case BW_125:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 774;
                    break;
                case CR_6: _sendTime = 864;
                    break;
                case CR_7: _sendTime = 954;
                    break;
                case CR_8: _sendTime = 1044;
                    break;
                }
                break;
            case BW_250:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 506;
                    break;
                case CR_6: _sendTime = 552;
                    break;
                case CR_7: _sendTime = 596;
                    break;
                case CR_8: _sendTime = 642;
                    break;
                }
                break;
            case BW_500:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 374;
                    break;
                case CR_6: _sendTime = 396;
                    break;
                case CR_7: _sendTime = 418;
                    break;
                case CR_8: _sendTime = 441;
                    break;
                }
                break;
            }
            break;

        case SF_10:	switch(_bandwidth)
            {	// Choosing bandwidth
            case BW_125:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 1226;
                    break;
                case CR_6: _sendTime = 1388;
                    break;
                case CR_7: _sendTime = 1552;
                    break;
                case CR_8: _sendTime = 1716;
                    break;
                }
                break;
            case BW_250:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 732;
                    break;
                case CR_6: _sendTime = 815;
                    break;
                case CR_7: _sendTime = 896;
                    break;
                case CR_8: _sendTime = 977;
                    break;
                }
                break;
            case BW_500:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 486;
                    break;
                case CR_6: _sendTime = 527;
                    break;
                case CR_7: _sendTime = 567;
                    break;
                case CR_8: _sendTime = 608;
                    break;
                }
                break;
            }
            break;

        case SF_11:	switch(_bandwidth)
            {	// Choosing bandwidth
            case BW_125:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 2375;
                    break;
                case CR_6: _sendTime = 2735;
                    break;
                case CR_7: _sendTime = 3095;
                    break;
                case CR_8: _sendTime = 3456;
                    break;
                }
                break;
            case BW_250:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 1144;
                    break;
                case CR_6: _sendTime = 1291;
                    break;
                case CR_7: _sendTime = 1437;
                    break;
                case CR_8: _sendTime = 1586;
                    break;
                }
                break;
            case BW_500:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 691;
                    break;
                case CR_6: _sendTime = 766;
                    break;
                case CR_7: _sendTime = 838;
                    break;
                case CR_8: _sendTime = 912;
                    break;
                }
                break;
            }
            break;

        case SF_12: switch(_bandwidth)
            {	// Choosing bandwidth
            case BW_125:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 4180;
                    break;
                case CR_6: _sendTime = 4836;
                    break;
                case CR_7: _sendTime = 5491;
                    break;
                case CR_8: _sendTime = 6146;
                    break;
                }
                break;
            case BW_250:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 1965;
                    break;
                case CR_6: _sendTime = 2244;
                    break;
                case CR_7: _sendTime = 2521;
                    break;
                case CR_8: _sendTime = 2800;
                    break;
                }
                break;
            case BW_500:
                switch(_codingRate)
                {	// Choosing coding rate
                case CR_5: _sendTime = 1102;
                    break;
                case CR_6: _sendTime = 1241;
                    break;
                case CR_7: _sendTime = 1381;
                    break;
                case CR_8: _sendTime = 1520;
                    break;
                }
                break;
            }
            break;
        default: _sendTime = MAX_TIMEOUT;
        }
    }
    else
    {
        _sendTime = MAX_TIMEOUT;
    }
    delay_ms = ((0.1*_sendTime) + 1);
    _sendTime = (uint16_t) ((_sendTime * 1.2) + (rand()%delay_ms));
#ifdef SX1272_DEBUG
    SerialPrint("Timeout to send/receive is: "NL);
    SerialPrintNr(_sendTime, DEC);
    SerialPrint(NL);
#endif
    state = 0;
    return state;
}

/*
 Function: It sets a char array payload packet in a packet struct.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_setPayload(char *payload)
{
    uint8_t state = 2;
    uint8_t state_f = 2;
    uint16_t length16;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setPayload'"NL);
#endif

    state = 1;
    length16 = (uint16_t)strlen(payload);
    state = sx1272_truncPayload(length16);
    if( state == 0 )
    {
        // fill data field until the end of the string
        unsigned int i;
        for(i = 0; i < _payloadlength; i++)
        {
            packet_sent.data[i] = payload[i];
        }
    }
    else
    {
        state_f = state;
    }
    if( ( _modem == FSK ) && ( _payloadlength > MAX_PAYLOAD_FSK ) )
    {
        _payloadlength = MAX_PAYLOAD_FSK;
        state = 1;
#ifdef SX1272_DEBUG
        SerialPrint("In FSK, payload length must be less than 60 bytes."NL);        
#endif
    }
    // set length with the actual counter value
    state_f = sx1272_setPacketLength(_rawFormat ? _payloadlength : _payloadlength+OFFSET_PAYLOADLENGTH);	// Setting packet length in packet structure
    return state_f;
}

/*
 Function: It sets a packet struct in FIFO in order to send it.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_setPacket2(uint8_t dest, char *payload)
{
    int8_t state = 2;
    unsigned int i;


#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setPacket'"NL);
#endif

    sx1272_clearFlags();	// Initializing flags

    if( _modem == LORA )
    { // LoRa mode
        sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Stdby LoRa mode to write in FIFO
    }
    else
    { // FSK mode
        sx1272_writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Stdby FSK mode to write in FIFO
    }

    _reception = CORRECT_PACKET;	// Updating incorrect value
    if( _retries == 0 )
    { // Updating this values only if is not going to re-send the last packet
        state = sx1272_setDestination(dest);	// Setting destination in packet structure
        packet_sent.retry = _retries;
        if( state == 0 )
        {
            state = sx1272_setPayload(payload);
        }
    }
    else
    {
        // comment by C. Pham
        // why to increase the length here?
        // bug?
        if( _retries == 1 )
        {
            packet_sent.length++;
        }
        state = sx1272_setPacketLength(_rawFormat ? _payloadlength : _payloadlength+OFFSET_PAYLOADLENGTH);
        packet_sent.retry = _retries;
#ifdef SX1272_DEBUG
        SerialPrint("** Retrying to send last packet ");
        SerialPrintNr(_retries, DEC);
        SerialPrint(" time **"NL);
#endif
    }

    // added by C. Pham
    // set the type to be a data packet
    packet_sent.type |= PKT_TYPE_DATA;

#ifdef W_REQUESTED_ACK
    // added by C. Pham
    // indicate that an ACK should be sent by the receiver
    if (_requestACK)
        packet_sent.type |= PKT_FLAG_ACK_REQ;
#endif

    sx1272_writeRegister(REG_FIFO_ADDR_PTR, 0x80);  // Setting address pointer in FIFO data buffer
    if( state == 0 )
    {
        state = 1;
        // Writing packet to send in FIFO
#ifdef W_NET_KEY
        // added by C. Pham
        packet_sent.netkey[0]=_my_netkey[0];
        packet_sent.netkey[1]=_my_netkey[1];
        //#ifdef SX1272_DEBUG
        SerialPrint("## Setting net key ##"NL);
        //#endif
        sx1272_writeRegister(REG_FIFO, packet_sent.netkey[0]);
        sx1272_writeRegister(REG_FIFO, packet_sent.netkey[1]);
#endif
        // added by C. Pham
        // we can skip the header for instance when we want to generate
        // at a higher layer a LoRaWAN packet
        if (!_rawFormat) {
            sx1272_writeRegister(REG_FIFO, packet_sent.dst); 		// Writing the destination in FIFO
            // added by C. Pham
            sx1272_writeRegister(REG_FIFO, packet_sent.type); 		// Writing the packet type in FIFO
            sx1272_writeRegister(REG_FIFO, packet_sent.src);		// Writing the source in FIFO
            sx1272_writeRegister(REG_FIFO, packet_sent.packnum);	// Writing the packet number in FIFO
        }
        // commented by C. Pham
        //sx1272_writeRegister(REG_FIFO, packet_sent.length); 	// Writing the packet length in FIFO
        for(i = 0; i < _payloadlength; i++)
        {
            sx1272_writeRegister(REG_FIFO, packet_sent.data[i]);  // Writing the payload in FIFO
        }
        // commented by C. Pham
        //sx1272_writeRegister(REG_FIFO, packet_sent.retry);		// Writing the number retry in FIFO
        state = 0;
#ifdef SX1272_DEBUG
        SerialPrint("## Packet set and written in FIFO ##"NL);
        // Print the complete packet if debug_mode
        SerialPrint("## Packet to send: "NL);
        SerialPrint("Destination: ");
        SerialPrintNr(packet_sent.dst, HEX);			 	// Printing destination
        SerialPrint(NL"Packet type: ");
        SerialPrintNr(packet_sent.type, DEC);			// Printing packet type
        SerialPrint(NL"Source: ");
        SerialPrintNr(packet_sent.src, HEX);			 	// Printing source
        SerialPrint(NL"Packet number: ");
        SerialPrintNr(packet_sent.packnum, DEC);			// Printing packet number
        SerialPrint(NL"Packet length: ");
        SerialPrintNr(packet_sent.length, DEC);			// Printing packet length
        SerialPrint(NL"Data: ");
        for(i = 0; i < _payloadlength; i++)
        {
            SerialPrintNr((char)packet_sent.data[i], HEX);		// Printing payload
        }
        
        //SerialPrint("Retry number: "NL);
        //SerialPrintNr(packet_sent.retry);			// Printing retry number
        SerialPrint("##"NL);
#endif
    }

    return state;
}

/*
 Function: It sets a packet struct in FIFO in order to sent it.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_setPacket(uint8_t dest, uint8_t *payload)
{
    int8_t state = 2;
    byte st0;
    unsigned int i;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setPacket'"NL);
#endif

    st0 = sx1272_readRegister(REG_OP_MODE);	// Save the previous status
    sx1272_clearFlags();	// Initializing flags

    if( _modem == LORA )
    { // LoRa mode
        sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Stdby LoRa mode to write in FIFO
    }
    else
    { // FSK mode
        sx1272_writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Stdby FSK mode to write in FIFO
    }

    _reception = CORRECT_PACKET;	// Updating incorrect value to send a packet (old or new)
    if( _retries == 0 )
    { // Sending new packet
        state = sx1272_setDestination(dest);	// Setting destination in packet structure
        packet_sent.retry = _retries;
        if( state == 0 )
        {
            state = sx1272_setPayload(payload);
        }
    }
    else
    {
        // comment by C. Pham
        // why to increase the length here?
        // bug?
        if( _retries == 1 )
        {
            packet_sent.length++;
        }
        state = sx1272_setPacketLength(_rawFormat ? _payloadlength : _payloadlength+OFFSET_PAYLOADLENGTH);
        packet_sent.retry = _retries;
#ifdef SX1272_DEBUG
        SerialPrint("** Retrying to send last packet ");
        SerialPrintNr(_retries, DEC);
        SerialPrint(" time **"NL);
#endif
    }

    // added by C. Pham
    // set the type to be a data packet
    packet_sent.type |= PKT_TYPE_DATA;

#ifdef W_REQUESTED_ACK
    // added by C. Pham
    // indicate that an ACK should be sent by the receiver
    if (_requestACK)
        packet_sent.type |= PKT_FLAG_ACK_REQ;
#endif

    sx1272_writeRegister(REG_FIFO_ADDR_PTR, 0x80);  // Setting address pointer in FIFO data buffer
    if( state == 0 )
    {
        state = 1;
        // Writing packet to send in FIFO
#ifdef W_NET_KEY
        // added by C. Pham
        packet_sent.netkey[0]=_my_netkey[0];
        packet_sent.netkey[1]=_my_netkey[1];
        //#ifdef SX1272_DEBUG
        SerialPrint("## Setting net key ##"NL);
        //#endif
        sx1272_writeRegister(REG_FIFO, packet_sent.netkey[0]);
        sx1272_writeRegister(REG_FIFO, packet_sent.netkey[1]);
#endif
        // added by C. Pham
        // we can skip the header for instance when we want to generate
        // at a higher layer a LoRaWAN packet
        if (!_rawFormat) {
            sx1272_writeRegister(REG_FIFO, packet_sent.dst); 		// Writing the destination in FIFO
            // added by C. Pham
            sx1272_writeRegister(REG_FIFO, packet_sent.type); 		// Writing the packet type in FIFO
            sx1272_writeRegister(REG_FIFO, packet_sent.src);		// Writing the source in FIFO
            sx1272_writeRegister(REG_FIFO, packet_sent.packnum);	// Writing the packet number in FIFO
        }
        // commented by C. Pham
        //sx1272_writeRegister(REG_FIFO, packet_sent.length); 	// Writing the packet length in FIFO
        for(i = 0; i < _payloadlength; i++)
        {
            sx1272_writeRegister(REG_FIFO, packet_sent.data[i]);  // Writing the payload in FIFO
        }
        // commented by C. Pham
        //sx1272_writeRegister(REG_FIFO, packet_sent.retry);		// Writing the number retry in FIFO
        state = 0;
#ifdef SX1272_DEBUG
        SerialPrint("## Packet set and written in FIFO ##"NL);
        // Print the complete packet if debug_mode
        SerialPrint("## Packet to send: "NL);
        SerialPrint("Destination: ");
        SerialPrintNr(packet_sent.dst, HEX);			// Printing destination
        SerialPrint(NL"Packet type: ");
        SerialPrintNr(packet_sent.type, DEC);			// Printing packet type
        SerialPrint(NL"Source: "NL);
        SerialPrintNr(packet_sent.src, HEX);			// Printing source
        SerialPrint(NL"Packet number: ");
        SerialPrintNr(packet_sent.packnum, DEC);        // Printing packet number
        SerialPrint(NL"Packet length: ");
        SerialPrintNr(packet_sent.length, DEC);			// Printing packet length
        SerialPrint(NL"Data: ");
        for(i = 0; i < _payloadlength; i++)
        {
            SerialPrintNr((char)packet_sent.data[i], HEX);		// Printing payload
        }
        
        //SerialPrint("Retry number: "NL);
        //SerialPrintNr(packet_sent.retry);			// Printing retry number
        SerialPrint("##"NL);
#endif
    }
    sx1272_writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    return state;
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendWithMAXTimeout(void)
{
    return sx1272_sendWithTimeout(MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendWithTimeout2(void)
{
    sx1272_setTimeout();
    return sx1272_sendWithTimeout(_sendTime);
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendWithTimeout(uint16_t wait)
{
    uint8_t state = 2;
    byte value = 0x00;
    unsigned long previous;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'sendWithTimeout'"NL);
#endif

    // sx1272_clearFlags();	// Initializing flags

    // wait to TxDone flag
    previous = millis();
    if( _modem == LORA )
    { // LoRa mode
        sx1272_clearFlags();	// Initializing flags

        sx1272_writeRegister(REG_OP_MODE, LORA_TX_MODE);  // LORA mode - Tx

#ifdef SX1272_DEBUG
        value = sx1272_readRegister(REG_OP_MODE);

        if (value & LORA_TX_MODE == LORA_TX_MODE)
            SerialPrint("OK"NL);
        else
            SerialPrint("ERROR"NL);
#endif
        value = sx1272_readRegister(REG_IRQ_FLAGS);
        // Wait until the packet is sent (TX Done flag) or the timeout expires
        while ((bitRead(value, 3) == 0) && (millis() - previous < wait))
        {
            value = sx1272_readRegister(REG_IRQ_FLAGS);
            // Condition to avoid an overflow (DO NOT REMOVE)
            if( millis() < previous )
            {
                previous = millis();
            }
        }
        state = 1;
    }
    else
    { // FSK mode
        sx1272_writeRegister(REG_OP_MODE, FSK_TX_MODE);  // FSK mode - Tx

        value = sx1272_readRegister(REG_IRQ_FLAGS2);
        // Wait until the packet is sent (Packet Sent flag) or the timeout expires
        while ((bitRead(value, 3) == 0) && (millis() - previous < wait))
        {
            value = sx1272_readRegister(REG_IRQ_FLAGS2);
            // Condition to avoid an overflow (DO NOT REMOVE)
            if( millis() < previous )
            {
                previous = millis();
            }
        }
        state = 1;
    }
    if( bitRead(value, 3) == 1 )
    {
        state = 0;	// Packet successfully sent
#ifdef SX1272_DEBUG
        SerialPrint("## Packet successfully sent ##"NL);        
#endif
    }
    else
    {
        if( state == 1 )
        {
#ifdef SX1272_DEBUG
            SerialPrint("** Timeout has expired **"NL);            
#endif
        }
        else
        {
#ifdef SX1272_DEBUG
            SerialPrint("** There has been an error and packet has not been sent **"NL);            
#endif
        }
    }

    sx1272_clearFlags();		// Initializing flags
    return state;
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendPacketMAXTimeout2(uint8_t dest, char *payload)
{
    return sx1272_sendPacketTimeout3(dest, payload, MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendPacketMAXTimeout(uint8_t dest,  uint8_t *payload, uint16_t length16)
{
    return sx1272_sendPacketTimeout4(dest, payload, length16, MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendPacketTimeout2(uint8_t dest, char *payload)
{
    uint8_t state = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'sendPacketTimeout'"NL);
#endif

    state = sx1272_setPacket2(dest, payload);	// Setting a packet with 'dest' destination
    if (state == 0)								// and writing it in FIFO.
    {
        state = sx1272_sendWithTimeout2();	// Sending the packet
    }
    return state;
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendPacketTimeout(uint8_t dest, uint8_t *payload, uint16_t length16)
{
    uint8_t state = 2;
    uint8_t state_f = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'sendPacketTimeout'"NL);
#endif

    state = sx1272_truncPayload(length16);
    if( state == 0 )
    {
        state_f = sx1272_setPacket(dest, payload);	// Setting a packet with 'dest' destination
    }												// and writing it in FIFO.
    else
    {
        state_f = state;
    }
    
    if( state_f == 0 )
    {
        state_f = sx1272_sendWithTimeout2();	// Sending the packet
    }
    
    return state_f;
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendPacketTimeout3(uint8_t dest, char *payload, uint16_t wait)
{
    uint8_t state = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'sendPacketTimeout'"NL);
#endif

    state = sx1272_setPacket2(dest, payload);	// Setting a packet with 'dest' destination
    if (state == 0)								// and writing it in FIFO.
    {
        state = sx1272_sendWithTimeout(wait);	// Sending the packet
    }
    return state;
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendPacketTimeout4(uint8_t dest, uint8_t *payload, uint16_t length16, uint16_t wait)
{
    uint8_t state = 2;
    uint8_t state_f = 2;

#ifdef SX1272_DEBUG
    SerialPrint("Starting 'sendPacketTimeout'"NL);
#endif

    state = sx1272_truncPayload(length16);
    if( state == 0 )
    {
        state_f = sx1272_setPacket(dest, payload);	// Setting a packet with 'dest' destination
    }
    else
    {
        state_f = state;
    }
    if( state_f == 0 )								// and writing it in FIFO.
    {
        state_f = sx1272_sendWithTimeout(wait);	// Sending the packet
    }
    return state_f;
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendPacketMAXTimeoutACK2(uint8_t dest, char *payload)
{
    return sx1272_sendPacketTimeoutACK3(dest, payload, MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information and receive an ACK.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendPacketMAXTimeoutACK(uint8_t dest, uint8_t *payload, uint16_t length16)
{
    return sx1272_sendPacketTimeoutACK4(dest, payload, length16, MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information and receive an ACK.
 Returns: Integer that determines if there has been any error
   state = 3  --> Packet has been sent but ACK has not been received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendPacketTimeoutACK2(uint8_t dest, char *payload)
{
    uint8_t state = 2;
    uint8_t state_f = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'sendPacketTimeoutACK'"NL);
#endif

#ifdef W_REQUESTED_ACK
    _requestACK = 1;
#endif
    state = sx1272_sendPacketTimeout2(dest, payload);	// Sending packet to 'dest' destination

    if( state == 0 )
    {
        state = sx1272_receive();	// Setting Rx mode to wait an ACK
    }
    else
    {
        state_f = state;
    }
    if( state == 0 )
    {
        // added by C. Pham
        SerialPrint("wait for ACK"NL);

        if( sx1272_availableData(MAX_TIMEOUT) )
        {
            state_f = sx1272_getACK(MAX_TIMEOUT);	// Getting ACK
        }
        else
        {
            state_f = 3;
            // added by C. Pham
            SerialPrint("no ACK"NL);
        }
    }
    else
    {
        state_f = state;
    }

#ifdef W_REQUESTED_ACK
    _requestACK = 0;
#endif
    return state_f;
}

/*
 Function: Configures the module to transmit information and receive an ACK.
 Returns: Integer that determines if there has been any error
   state = 3  --> Packet has been sent but ACK has not been received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendPacketTimeoutACK(uint8_t dest, uint8_t *payload, uint16_t length16)
{
    uint8_t state = 2;
    uint8_t state_f = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'sendPacketTimeoutACK'"NL);
#endif

#ifdef W_REQUESTED_ACK
    _requestACK = 1;
#endif
    // Sending packet to 'dest' destination
    state = sx1272_sendPacketTimeout(dest, payload, length16);

    // Trying to receive the ACK
    if( state == 0 )
    {
        state = sx1272_receive();	// Setting Rx mode to wait an ACK
    }
    else
    {
        state_f = state;
    }
    if( state == 0 )
    {
        if( sx1272_availableData(MAX_TIMEOUT) )
        {
            state_f = sx1272_getACK(MAX_TIMEOUT);	// Getting ACK
        }
        else
        {
            state_f = 3;
        }
    }
    else
    {
        state_f = state;
    }

#ifdef W_REQUESTED_ACK
    _requestACK = 0;
#endif
    return state_f;
}

/*
 Function: Configures the module to transmit information and receive an ACK.
 Returns: Integer that determines if there has been any error
   state = 3  --> Packet has been sent but ACK has not been received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendPacketTimeoutACK3(uint8_t dest, char *payload, uint16_t wait)
{
    uint8_t state = 2;
    uint8_t state_f = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'sendPacketTimeoutACK'"NL);
#endif

#ifdef W_REQUESTED_ACK
    _requestACK = 1;
#endif
    state = sx1272_sendPacketTimeout3(dest, payload, wait);	// Sending packet to 'dest' destination

    if( state == 0 )
    {
        state = sx1272_receive();	// Setting Rx mode to wait an ACK
    }
    else
    {
        state_f = 1;
    }
    if( state == 0 )
    {
        // added by C. Pham
        SerialPrint("wait for ACK"NL);

        if( sx1272_availableData(MAX_TIMEOUT) )
        {
            state_f = sx1272_getACK(MAX_TIMEOUT);	// Getting ACK
        }
        else
        {
            state_f = 3;
            // added by C. Pham
            SerialPrint("no ACK"NL);
        }
    }
    else
    {
        state_f = 1;
    }

#ifdef W_REQUESTED_ACK
    _requestACK = 0;
#endif
    return state_f;
}

/*
 Function: Configures the module to transmit information and receive an ACK.
 Returns: Integer that determines if there has been any error
   state = 3  --> Packet has been sent but ACK has not been received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendPacketTimeoutACK4(uint8_t dest, uint8_t *payload, uint16_t length16, uint16_t wait)
{
    uint8_t state = 2;
    uint8_t state_f = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'sendPacketTimeoutACK'"NL);
#endif

#ifdef W_REQUESTED_ACK
    _requestACK = 1;
#endif
    state = sx1272_sendPacketTimeout4(dest, payload, length16, wait);	// Sending packet to 'dest' destination

    if( state == 0 )
    {
        state = sx1272_receive();	// Setting Rx mode to wait an ACK
    }
    else
    {
        state_f = 1;
    }
    if( state == 0 )
    {
        // added by C. Pham
        SerialPrint("wait for ACK"NL);

        if( sx1272_availableData(MAX_TIMEOUT) )
        {
            state_f = sx1272_getACK(MAX_TIMEOUT);	// Getting ACK
        }
        else
        {
            state_f = 3;
            // added by C. Pham
            SerialPrint("no ACK"NL);
        }
    }
    else
    {
        state_f = 1;
    }

#ifdef W_REQUESTED_ACK
    _requestACK = 0;
#endif
    return state_f;
}

/*
 Function: It gets and stores an ACK if it is received, before ending 'wait' time.
 Returns: Integer that determines if there has been any error
   state = 2  --> The ACK has not been received
   state = 1  --> The N-ACK has been received with no errors
   state = 0  --> The ACK has been received with no errors
 Parameters:
   wait: time to wait while there is no a valid header received.
*/
uint8_t sx1272_getACK(uint16_t wait)
{
    uint8_t state = 2;
    byte value = 0x00;
    unsigned long previous;
    boolean a_received = FALSE;

    //#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'getACK'"NL);
    //#endif

    previous = millis();

    if( _modem == LORA )
    { // LoRa mode
        value = sx1272_readRegister(REG_IRQ_FLAGS);
        // Wait until the ACK is received (RxDone flag) or the timeout expires
        while ((bitRead(value, 6) == 0) && (millis() - previous < wait))
        {
            value = sx1272_readRegister(REG_IRQ_FLAGS);
            if( millis() < previous )
            {
                previous = millis();
            }
        }
        if( bitRead(value, 6) == 1 )
        { // ACK received
            // comment by C. Pham
            // not really safe because the received packet may not be an ACK
            // probability is low if using unicast to gateway, but if broadcast
            // can get a packet from another node!!
            a_received = TRUE;
        }
        // Standby para minimizar el consumo
        sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Setting standby LoRa mode
    }
    else
    { // FSK mode
        value = sx1272_readRegister(REG_IRQ_FLAGS2);
        // Wait until the packet is received (RxDone flag) or the timeout expires
        while ((bitRead(value, 2) == 0) && (millis() - previous < wait))
        {
            value = sx1272_readRegister(REG_IRQ_FLAGS2);
            if( millis() < previous )
            {
                previous = millis();
            }
        }
        if( bitRead(value, 2) == 1 )
        { // ACK received
            a_received = TRUE;
        }
        // Standby para minimizar el consumo
        sx1272_writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Setting standby FSK mode
    }

    // comment by C. Pham
    // not safe because the received packet may not be an ACK!
    if( a_received )
    {
        // Storing the received ACK
        ACK.dst = _destination;
        ACK.type = sx1272_readRegister(REG_FIFO);
        ACK.src = sx1272_readRegister(REG_FIFO);
        ACK.packnum = sx1272_readRegister(REG_FIFO);
        ACK.length = sx1272_readRegister(REG_FIFO);
        ACK.data[0] = sx1272_readRegister(REG_FIFO);
        ACK.data[1] = sx1272_readRegister(REG_FIFO);

        if (ACK.type == PKT_TYPE_ACK) {

            // Checking the received ACK
            if( ACK.dst == packet_sent.src )
            {
                if( ACK.src == packet_sent.dst )
                {
                    if( ACK.packnum == packet_sent.packnum )
                    {
                        if( ACK.length == 2 )
                        {
                            if( ACK.data[0] == CORRECT_PACKET )
                            {
                                state = 0;
                                //#ifdef SX1272_DEBUG
                                // Printing the received ACK
                                SerialPrint("## ACK received:"NL);
                                SerialPrint("Destination: ");
                                SerialPrintNr(ACK.dst, HEX);			 	// Printing destination
                                SerialPrint(NL"Source: ");
                                SerialPrintNr(ACK.src, HEX);			 	// Printing source
                                SerialPrint(NL"ACK number: ");
                                SerialPrintNr(ACK.packnum, DEC);			// Printing ACK number
                                SerialPrint(NL"ACK length: ");
                                SerialPrintNr(ACK.length, DEC);				// Printing ACK length
                                SerialPrint(NL"ACK payload: ");
                                SerialPrintNr(ACK.data[0], HEX);			// Printing ACK payload
                                SerialPrint(NL"ACK SNR of rcv pkt at gw: ");

                                value = ACK.data[1];

                                if( value & 0x80 ) // The SNR sign bit is 1
                                {
                                    // Invert and divide by 4
                                    value = ( ( ~value + 1 ) & 0xFF ) >> 2;
                                    _rcv_snr_in_ack = -value;
                                }
                                else
                                {
                                    // Divide by 4
                                    _rcv_snr_in_ack = ( value & 0xFF ) >> 2;
                                }

                                SerialPrintNr(_rcv_snr_in_ack, DEC);
                                SerialPrint("##"NL);
                                
                                //#endif
                            }
                            else
                            {
                                state = 1;
                                //#ifdef SX1272_DEBUG
                                SerialPrint("** N-ACK received **"NL);
                                
                                //#endif
                            }
                        }
                        else
                        {
                            state = 1;
                            //#ifdef SX1272_DEBUG
                            SerialPrint("** ACK length incorrectly received **"NL);                            
                            //#endif
                        }
                    }
                    else
                    {
                        state = 1;
                        //#ifdef SX1272_DEBUG
                        SerialPrint("** ACK number incorrectly received **"NL);                        
                        //#endif
                    }
                }
                else
                {
                    state = 1;
                    //#ifdef SX1272_DEBUG
                    SerialPrint("** ACK source incorrectly received **"NL);                    
                    //#endif
                }
            }
        }
        else
        {
            state = 1;
            //#ifdef SX1272_DEBUG
            SerialPrint("** ACK destination incorrectly received **"NL);            
            //#endif
        }
    }
    else
    {
        state = 1;
        //#ifdef SX1272_DEBUG
        SerialPrint("** ACK lost **"NL);        
        //#endif
    }
    sx1272_clearFlags();	// Initializing flags
    return state;
}

/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendPacketMAXTimeoutACKRetries2(uint8_t dest, char  *payload)
{
    return sx1272_sendPacketTimeoutACKRetries3(dest, payload, MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendPacketMAXTimeoutACKRetries(uint8_t dest, uint8_t *payload, uint16_t length16)
{
    return sx1272_sendPacketTimeoutACKRetries4(dest, payload, length16, MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendPacketTimeoutACKRetries2(uint8_t dest, char *payload)
{
    uint8_t state = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'sendPacketTimeoutACKRetries'"NL);
#endif

    // Sending packet to 'dest' destination and waiting an ACK response.
    state = 1;
    while( (state != 0) && (_retries <= _maxRetries) )
    {
        state = sx1272_sendPacketTimeoutACK2(dest, payload);
        _retries++;
    }
    _retries = 0;

    return state;
}

/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendPacketTimeoutACKRetries(uint8_t dest, uint8_t *payload, uint16_t length16)
{
    uint8_t state = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'sendPacketTimeoutACKRetries'"NL);
#endif

    // Sending packet to 'dest' destination and waiting an ACK response.
    state = 1;
    while((state != 0) && (_retries <= _maxRetries))
    {
        state = sx1272_sendPacketTimeoutACK(dest, payload, length16);
        _retries++;

    }
    _retries = 0;

    return state;
}

/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendPacketTimeoutACKRetries3(uint8_t dest, char *payload, uint16_t wait)
{
    uint8_t state = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'sendPacketTimeoutACKRetries'"NL);
#endif

    // Sending packet to 'dest' destination and waiting an ACK response.
    state = 1;
    while((state != 0) && (_retries <= _maxRetries))
    {
        state = sx1272_sendPacketTimeoutACK3(dest, payload, wait);
        _retries++;
    }
    _retries = 0;

    return state;
}

/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_sendPacketTimeoutACKRetries4(uint8_t dest, uint8_t *payload, uint16_t length16, uint16_t wait)
{
    uint8_t state = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'sendPacketTimeoutACKRetries'"NL);
#endif

    // Sending packet to 'dest' destination and waiting an ACK response.
    state = 1;
    while((state != 0) && (_retries <= _maxRetries))
    {
        state = sx1272_sendPacketTimeoutACK4(dest, payload, length16, wait);
        _retries++;
    }
    _retries = 0;

    return state;
}

/*
 Function: It gets the temperature from the measurement block module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_getTemp(void)
{
    byte st0;
    uint8_t state = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'getTemp'"NL);
#endif

    st0 = sx1272_readRegister(REG_OP_MODE);	// Save the previous status

    if( _modem == LORA )
    { // Allowing access to FSK registers while in LoRa standby mode
        sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_FSK_REGS_MODE);
    }
 
#ifdef SX1272_DEBUG 
    sx1272_readRegister(REG_OP_MODE);	
#endif
    
    state = 1;
    // Saving temperature value
    _temp = sx1272_readRegister(REG_TEMP);
    if((_temp & 0x80) == 0x80 ) // The SNR sign bit is 1
    {
        // Invert and divide by 4
        _temp = 0xff - _temp; //( ( ~_temp + 1 ) & 0xFF );
    }
    else
    {
        // Divide by 4
        _temp *= -1; // ( _temp & 0xFF );
    }
    _temp += _tempCalibration;

#ifdef SX1272_DEBUG
    SerialPrint("## Temperature is: ");
    SerialPrintNr(_temp, DEC);
    SerialPrint(" ##"NL);
    
#endif

    if( _modem == LORA )
    {
        sx1272_writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
    }

    state = 0;
    return state;
}

//**********************************************************************/
// Added by C. Pham
//**********************************************************************/

void sx1272_setPacketType(uint8_t type)
{
    packet_sent.type = type;

    #ifdef W_REQUESTED_ACK
            if (type & PKT_FLAG_ACK_REQ) _requestACK = 1;
    #endif
}

/*
 Function: Configures the module to perform CAD.
 Returns: Integer that determines if the number of requested CAD have been successfull
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sx1272_doCAD(uint8_t counter)
{
    uint8_t state = 2;
    byte value = 0x00;
    unsigned long startCAD, endCAD, startDoCad, endDoCad, previous;
    uint16_t wait = 100;
    boolean failedCAD=FALSE;
    uint8_t retryCAD = 3;
    uint8_t save_counter;
    byte st0;

    st0 = sx1272_readRegister(REG_OP_MODE);	// Save the previous status

#ifdef DEBUG_CAD
    SerialPrint("Starting 'doCAD'"NL);
#endif

    save_counter = counter;

    startDoCad=millis();

    if( _modem == LORA ) { // LoRa mode

        sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);

        do {

            // wait to CadDone flag
            startCAD = previous = millis();

            sx1272_clearFlags();	// Initializing flags

            sx1272_writeRegister(REG_OP_MODE, LORA_CAD_MODE);  // LORA mode - Cad

            value = sx1272_readRegister(REG_IRQ_FLAGS);
            // Wait until CAD ends (CAD Done flag) or the timeout expires
            while ((bitRead(value, 2) == 0) && (millis() - previous < wait))
            {
                value = sx1272_readRegister(REG_IRQ_FLAGS);
                // Condition to avoid an overflow (DO NOT REMOVE)
                if( millis() < previous )
                {
                    previous = millis();
                }
            }
            state = 1;

            endCAD = millis();

            if( bitRead(value, 2) == 1 )
            {
                state = 0;	// CAD successfully performed
#ifdef DEBUG_CAD				  
                SerialPrint("CAD duration "NL);
                SerialPrintNr(endCAD-startCAD);
                SerialPrint("CAD successfully performed"NL);
#endif				  

                value = sx1272_readRegister(REG_IRQ_FLAGS);

                // look for the CAD detected bit
                if( bitRead(value, 0) == 1 )
                {
                    // we detected activity
                    failedCAD=TRUE;
#ifdef DEBUG_CAD				  		
                    SerialPrint("CAD exits after "NL);
                    SerialPrintNr(save_counter-counter);
#endif				  		
                }

                counter--;
            }
            else
            {
#ifdef DEBUG_CAD			  	 	
                SerialPrint("CAD duration "NL);
                SerialPrintNr(endCAD-startCAD);
#endif				  
                if( state == 1 )
                {
#ifdef DEBUG_CAD
                    SerialPrint("Timeout has expired"NL);
#endif
                }
                else
                {
#ifdef DEBUG_CAD
                    SerialPrint("Error and CAD has not been performed"NL);
#endif
                }

                retryCAD--;

                // to many errors, so exit by indicating that channel is not free
                if (!retryCAD)
                    failedCAD=TRUE;
            }

        } while (counter && !failedCAD);
    }

    sx1272_writeRegister(REG_OP_MODE, st0);

    endDoCad=millis();

    sx1272_clearFlags();		// Initializing flags

#ifdef DEBUG_CAD	  
    SerialPrint("doCAD duration "NL);
    SerialPrintNr(endDoCad-startDoCad);
#endif

    if (failedCAD)
        return 2;

    return state;
}

//#define DEBUG_GETTOA

#ifdef DEBUG_GETTOA

void printDouble( double val, byte precision){
    // prints val with number of decimal places determine by precision
    // precision is a number from 0 to 6 indicating the desired decimial places
    // example: lcdPrintDouble( 3.1415, 2); // prints 3.14 (two decimal places)

    if(val < 0.0){
        SerialPrintNr('-');
        val = -val;
    }

    Serial.print (int(val));  //prints the int part
    if( precision > 0) {
        SerialPrint("."NL); // print the decimal point
        unsigned long frac;
        unsigned long mult = 1;
        byte padding = precision -1;
        while(precision--)
            mult *=10;

        if(val >= 0)
            frac = (val - int(val)) * mult;
        else
            frac = (int(val)- val ) * mult;
        unsigned long frac1 = frac;
        while( frac1 /= 10 )
            padding--;
        while(  padding--)
            SerialPrint("0"NL);
        SerialPrintNr(frac,DEC) ;
    }
}

#endif

uint16_t sx1272_getToA(uint8_t pl) {

    uint8_t DE = 0;
    uint32_t airTime = 0;

    double bw=0.0;

    bw=(_bandwidth==BW_125)?125e3:((_bandwidth==BW_250)?250e3:500e3);

#ifdef DEBUG_GETTOA
    SerialPrint("bw is "NL);
    SerialPrintNr(bw);

    SerialPrint("SF is "NL);
    SerialPrintNr(_spreadingFactor);
#endif

    //double ts=pow(2,_spreadingFactor)/bw;

    ////// from LoRaMAC SX1272GetTimeOnAir()

    // Symbol rate : time for one symbol (secs)
    double rs = bw / ( 1 << _spreadingFactor);
    double ts = 1 / rs;

    // must add 4 to the programmed preamble length to get the effective preamble length
    double tPreamble=((_preamblelength+4)+4.25)*ts;

#ifdef DEBUG_GETTOA	
    SerialPrint("ts is "NL);
    printDouble(ts,6);
    
    SerialPrint("tPreamble is "NL);
    printDouble(tPreamble,6);
    
#endif

    // for low data rate optimization
    if ((_bandwidth == BW_125) && _spreadingFactor == 12)
        DE = 1;

    // Symbol length of payload and time
    double tmp = (8*pl - 4*_spreadingFactor + 28 + 16 - 20*_header) /
            (double)(4*(_spreadingFactor-2*DE) );

#ifdef DEBUG_GETTOA                         
    SerialPrint("tmp is "NL);
    printDouble(tmp,6);
    
#endif

    tmp = ceil(tmp)*(_codingRate + 4);

    double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );

#ifdef DEBUG_GETTOA    
    SerialPrint("nPayload is "NL);
    SerialPrintNr(nPayload);
#endif

    double tPayload = nPayload * ts;
    // Time on air
    double tOnAir = tPreamble + tPayload;
    // in us secs
    airTime = floor( tOnAir * 1e6 + 0.999 );

    //////

#ifdef DEBUG_GETTOA    
    SerialPrint("airTime is "NL);
    SerialPrintNr(airTime);
#endif
    // return in ms
    return ceil(airTime/1000)+1;
}

// need to set _send_cad_number to a value > 0
// we advise using _send_cad_number=3 for a SIFS and _send_cad_number=9 for a DIFS
// prior to send any data
// TODO: have a maximum number of trials
void sx1272_CarrierSense(void) {

    int status;
    boolean carrierSenseRetry=FALSE;

    if (_send_cad_number && _enableCarrierSense) {
        do {
            do {

                // check for free channel (SIFS/DIFS)
                _startDoCad=millis();
                status = sx1272_doCAD(_send_cad_number);
                _endDoCad=millis();

                #ifdef SX1272_DEBUG
                SerialPrint("--> CAD duration "NL);
                SerialPrintNr(_endDoCad-_startDoCad, DEC);
                #endif

                if (!status) {
                    #ifdef SX1272_DEBUG
                    SerialPrint("OK1"NL);
                    #endif
                    
                    if (_extendedIFS)  {
                        // wait for random number of CAD
                        uint8_t w = rand()%8 + 1;

                        #ifdef SX1272_DEBUG
                        SerialPrint("--> waiting for "NL);
                        SerialPrintNr(w, DEC);
                        SerialPrint(" CAD = ");
                        SerialPrintNr(sx1272_CAD_value[_loraMode]*w, DEC);
                        #endif

                        delay_ms(sx1272_CAD_value[_loraMode]*w);

                        // check for free channel (SIFS/DIFS) once again
                        _startDoCad=millis();
                        status = sx1272_doCAD(_send_cad_number);
                        _endDoCad=millis();

                        #ifdef SX1272_DEBUG
                        SerialPrint("--> CAD duration "NL);
                        SerialPrintNr(_endDoCad-_startDoCad, DEC);
                        
                        if (!status)
                            SerialPrint("OK2"NL);
                        else
                            SerialPrint("###2"NL);
                        #endif
                    }
                }
                else {
                    #ifdef SX1272_DEBUG
                    SerialPrint("###1\n"NL);
                    #endif

                    // wait for random number of DIFS
                    uint8_t w = rand()%8 + 1;

                    #ifdef SX1272_DEBUG
                    SerialPrint("--> waiting for "NL);
                    SerialPrintNr(w, DEC);
                    SerialPrint(" DIFS (DIFS=3SIFS) = "NL);
                    SerialPrintNr(sx1272_SIFS_value[_loraMode]*3*w, DEC);
                    #endif

                    delay_ms(sx1272_SIFS_value[_loraMode]*3*w);

                    #ifdef SX1272_DEBUG
                    SerialPrint("--> retry\n"NL);
                    #endif
                }

            } while (status);

            // CAD is OK, but need to check RSSI
            if (_RSSIonSend) {

                status=sx1272_getRSSI();

                uint8_t rssi_retry_count=10;

                if (!status) {

                    #ifdef SX1272_DEBUG
                    SerialPrint("--> RSSI "NL);
                    SerialPrintNr(_RSSI, DEC);
                    #endif

                    while (_RSSI > -90 && rssi_retry_count) {

                        delay_ms(1);
                        sx1272_getRSSI();
                        
                        #ifdef SX1272_DEBUG
                        SerialPrint("--> RSSI "NL);
                        SerialPrintNr(_RSSI, DEC);
                        #endif

                        rssi_retry_count--;
                    }
                }
                else
                    #ifdef SX1272_DEBUG
                    SerialPrint("--> RSSI error\n"NL);
                    #endif

                if (!rssi_retry_count)
                    carrierSenseRetry=TRUE;
                else
                    carrierSenseRetry=FALSE;
            }

        } while (carrierSenseRetry);
    }
}

/*
 Function: Indicates the CR within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t	sx1272_getSyncWord(void)
{
    int8_t state = 2;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'getSyncWord'"NL);
#endif

    if( _modem == FSK )
    {
        state = -1;		// sync word is not available in FSK mode
#ifdef SX1272_DEBUG
        SerialPrint("** FSK mode hasn't sync word **"NL);        
#endif
    }
    else
    {
        _syncWord = sx1272_readRegister(REG_SYNC_WORD);

        state = 0;

#ifdef SX1272_DEBUG
        SerialPrint("## Sync word is ");
        SerialPrintNr(_syncWord, HEX);
        SerialPrint(" ##"NL);        
#endif
    }
    return state;
}

/*
 Function: Sets the sync word in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   cod: sw is sync word value to set in LoRa modem configuration.
*/
int8_t	sx1272_setSyncWord(uint8_t sw)
{
    byte st0;
    int8_t state = 2;
    byte config1;

#ifdef SX1272_DEBUG    
    SerialPrint("Starting 'setSyncWord'"NL);
#endif

    st0 = sx1272_readRegister(REG_OP_MODE);		// Save the previous status

    if( _modem == FSK )
    {
#ifdef SX1272_DEBUG
        SerialPrint("## Notice that FSK hasn't sync word parameter, so you are configuring it in LoRa mode ##"NL);
#endif
        state = sx1272_setLORA();
    }
    sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);		// Set Standby mode to write in registers

    sx1272_writeRegister(REG_SYNC_WORD, sw);

    delay_ms(100);

    config1 = sx1272_readRegister(REG_SYNC_WORD);

    if (config1==sw) {
        state=0;
        _syncWord = sw;
#ifdef SX1272_DEBUG
        SerialPrint("## Sync Word "NL);
        SerialPrintNr(sw, HEX);
        SerialPrint(" has been successfully set ##"NL);
        
#endif
    }
    else {
        state=1;
#ifdef SX1272_DEBUG
        SerialPrint("** There has been an error while configuring Sync Word parameter **"NL);
        
#endif
    }

    sx1272_writeRegister(REG_OP_MODE,st0);	// Getting back to previous status
    delay_ms(100);
    return state;
}


int8_t sx1272_setSleepMode(void) {

    int8_t state = 2;
    byte value;

    sx1272_writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
    sx1272_writeRegister(REG_OP_MODE, LORA_SLEEP_MODE);    // LoRa sleep mode
	
	//delay_ms(50);
	
    value = sx1272_readRegister(REG_OP_MODE);

	//SerialPrint("## REG_OP_MODE 0x"NL);
	//SerialPrintNr(value, HEX);
	
    if (value == LORA_SLEEP_MODE)
        state=0;
    else
        state=1;

    return state;
}
