/* Include files */
#include "app.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Include my files */
#include "PCB/Settings.h"
#include "PCB/IO.h"
#include "USART.h"
#include "Delay.h"

#include "LoRaWAN/timer.h"
#include "LoRaWAN/Comissioning.h"
#include "LoRaWAN/mac/LoRaMac.h"

//------------------------------------------------------------------------------

/*!
 * Timer to handle the state of LED1
 */
static TimerEvent_t Led1Timer;
static TimerEvent_t Led2Timer;
static TimerEvent_t Led3Timer;

/*!
 * \brief Function executed on Led 1 Timeout event
 */
static void OnLed1TimerEvent( void )
{
    TimerStop( &Led1Timer );
    // Switch LED 1 OFF
    SerialDisplayUpdateLedState( 1, 0 );
}

/*!
 * \brief Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent( void )
{
    TimerStop( &Led2Timer );
    // Switch LED 2 OFF
    SerialDisplayUpdateLedState( 2, 0 );
}

/*!
 * \brief Function executed on Led 2 Timeout event
 */
static void OnLed3TimerEvent( void )
{
    static uint8_t Led3State = 0;
    
    pin_Set(LED_1, PIN_TOGGLE);
    // Toggle LED 3
    Led3State ^= 1;
    SerialDisplayUpdateLedState( 3, Led3State );
}

//------------------------------------------------------------------------------
/*!
 * Defines the application data transmission duty cycle. 25s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                           25000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1

#if defined( USE_BAND_868 )

#include "LoRaWAN/mac/LoRaMacTest.h"
#include "Display/VT100/vt100.h"

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        false

#define USE_SEMTECH_DEFAULT_CHANNEL_LINEUP          false

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 ) 

#define LC4                { 867100000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC5                { 867300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC6                { 867500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC7                { 867700000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC8                { 867900000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }

#endif

#endif

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            2

/*!
 * User application data buffer size
 */
#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 )

#define LORAWAN_APP_DATA_SIZE                       16

#elif defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )

#define LORAWAN_APP_DATA_SIZE                       11

#endif

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif

/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = LORAWAN_APP_DATA_SIZE;

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           64

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;

/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = true;

/*!
 * Device states
 */
static enum eDevicState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;

/*!
 * LoRaWAN compliance tests support data
 */
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;
 
/*!
 * Strucure containing the Uplink status
 */
struct sLoRaMacUplinkStatus
{
    uint8_t Acked;
    int8_t Datarate;
    uint16_t UplinkCounter;
    uint8_t Port;
    uint8_t *Buffer;
    uint8_t BufferSize;
}LoRaMacUplinkStatus;
 
/*!
 * Strucure containing the Downlink status
 */
struct sLoRaMacDownlinkStatus
{
    int16_t Rssi;
    int8_t Snr;
    uint16_t DownlinkCounter;
    bool RxData;
    uint8_t Port;
    uint8_t *Buffer;
    uint8_t BufferSize;
}LoRaMacDownlinkStatus;
 
void SerialDisplayRefresh( void )
{
    MibRequestConfirm_t mibReq;
 
    SerialDisplayInit( );
    SerialDisplayUpdateActivationMode( OVER_THE_AIR_ACTIVATION );
 
#if( OVER_THE_AIR_ACTIVATION == 0 )
    SerialDisplayUpdateNwkId( LORAWAN_NETWORK_ID );
    SerialDisplayUpdateDevAddr( DevAddr );
    SerialDisplayUpdateKey( 12, NwkSKey );
    SerialDisplayUpdateKey( 13, AppSKey );
#endif
    SerialDisplayUpdateEui( 5, DevEui );
    SerialDisplayUpdateEui( 6, AppEui );
    SerialDisplayUpdateKey( 7, AppKey );
 
    mibReq.Type = MIB_NETWORK_JOINED;
    LoRaMacMibGetRequestConfirm( &mibReq );
    SerialDisplayUpdateNetworkIsJoined( mibReq.Param.IsNetworkJoined );
 
    SerialDisplayUpdateAdr( LORAWAN_ADR_ON );
#if defined( USE_BAND_868 )
    SerialDisplayUpdateDutyCycle( LORAWAN_DUTYCYCLE_ON );
#else
    SerialDisplayUpdateDutyCycle( false );
#endif
    SerialDisplayUpdatePublicNetwork( LORAWAN_PUBLIC_NETWORK );    
}
 
/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
    switch( port )
    {
    case 2:
        {
            uint16_t pressure;
            int16_t altitudeBar;
            int16_t temperature;
            int32_t latitude, longitude;
            int16_t altitudeGps;
            uint8_t batteryLevel;

            pressure     = 150;     // in hPa / 10
            temperature  = 2500;    // in °C * 100
            altitudeBar  = 3000;    // in m * 10
            batteryLevel = 254;     // 1 (very low) to 254 (fully charged)
            altitudeGps  = 2980;    // in m

            AppData[0] = 0; // Led state
            AppData[1] = ( pressure >> 8 ) & 0xFF;
            AppData[2] = pressure & 0xFF;
            AppData[3] = ( temperature >> 8 ) & 0xFF;
            AppData[4] = temperature & 0xFF;
            AppData[5] = ( altitudeBar >> 8 ) & 0xFF;
            AppData[6] = altitudeBar & 0xFF;
            AppData[7] = batteryLevel;
            AppData[8] = ( latitude >> 16 ) & 0xFF;
            AppData[9] = ( latitude >> 8 ) & 0xFF;
            AppData[10] = latitude & 0xFF;
            AppData[11] = ( longitude >> 16 ) & 0xFF;
            AppData[12] = ( longitude >> 8 ) & 0xFF;
            AppData[13] = longitude & 0xFF;
            AppData[14] = ( altitudeGps >> 8 ) & 0xFF;
            AppData[15] = altitudeGps & 0xFF;
        }
        break;
    case 224:
        if( ComplianceTest.LinkCheck == true )
        {
            ComplianceTest.LinkCheck = false;
            AppDataSize = 3;
            AppData[0] = 5;
            AppData[1] = ComplianceTest.DemodMargin;
            AppData[2] = ComplianceTest.NbGateways;
            ComplianceTest.State = 1;
        }
        else
        {
            switch( ComplianceTest.State )
            {
            case 4:
                ComplianceTest.State = 1;
                break;
            case 1:
                AppDataSize = 2;
                AppData[0] = ComplianceTest.DownLinkCounter >> 8;
                AppData[1] = ComplianceTest.DownLinkCounter;
                break;
            }
        }
        break;
    default:
        break;
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;
    
    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        
        //Status
        LoRaMacUplinkStatus.Acked = false;
        LoRaMacUplinkStatus.Port = 0;
        LoRaMacUplinkStatus.Buffer = NULL;
        LoRaMacUplinkStatus.BufferSize = 0;
        SerialDisplayUpdateFrameType( false );
    }
    else
    {
        //Status 
        LoRaMacUplinkStatus.Acked = false;
        LoRaMacUplinkStatus.Port = AppPort;
        LoRaMacUplinkStatus.Buffer = AppData;
        LoRaMacUplinkStatus.BufferSize = AppDataSize;
        SerialDisplayUpdateFrameType( IsTxConfirmed );
 
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &TxNextPacketTimer );

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            //DeviceState = DEVICE_STATE_JOIN;
        }
    }
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                
                //Status
                LoRaMacUplinkStatus.Acked = mcpsConfirm->AckReceived;
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }
        
        //Status
        LoRaMacUplinkStatus.Datarate = mcpsConfirm->Datarate;
        LoRaMacUplinkStatus.UplinkCounter = mcpsConfirm->UpLinkCounter;

        // Switch LED 1 ON
        SerialDisplayUpdateLedState( 1, 1 );
        SerialDisplayUpdateUplink( LoRaMacUplinkStatus.Acked, LoRaMacUplinkStatus.Datarate, LoRaMacUplinkStatus.UplinkCounter, LoRaMacUplinkStatus.Port, LoRaMacUplinkStatus.Buffer, LoRaMacUplinkStatus.BufferSize );

        TimerStart( &Led1Timer );
    }
    NextTx = true;
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot
    
    //Status
    LoRaMacDownlinkStatus.Rssi = mcpsIndication->Rssi;
    if( mcpsIndication->Snr & 0x80 ) { // The SNR sign bit is 1
        // Invert and divide by 4
        LoRaMacDownlinkStatus.Snr = ( ( ~mcpsIndication->Snr + 1 ) & 0xFF ) >> 2;
        LoRaMacDownlinkStatus.Snr = -LoRaMacDownlinkStatus.Snr;
    }
    else {
        // Divide by 4
        LoRaMacDownlinkStatus.Snr = ( mcpsIndication->Snr & 0xFF ) >> 2;
    }
    LoRaMacDownlinkStatus.DownlinkCounter++;
    LoRaMacDownlinkStatus.RxData = mcpsIndication->RxData;
    LoRaMacDownlinkStatus.Port = mcpsIndication->Port;
    LoRaMacDownlinkStatus.Buffer = mcpsIndication->Buffer;
    LoRaMacDownlinkStatus.BufferSize = mcpsIndication->BufferSize;

    if( ComplianceTest.Running == true )
    {
        ComplianceTest.DownLinkCounter++;
    }

    if( mcpsIndication->RxData == true )
    {
        switch( mcpsIndication->Port )
        {
        case 224:
            if( ComplianceTest.Running == false )
            {
                // Check compliance test enable command (i)
                if( ( mcpsIndication->BufferSize == 4 ) &&
                    ( mcpsIndication->Buffer[0] == 0x01 ) &&
                    ( mcpsIndication->Buffer[1] == 0x01 ) &&
                    ( mcpsIndication->Buffer[2] == 0x01 ) &&
                    ( mcpsIndication->Buffer[3] == 0x01 ) )
                {
                    IsTxConfirmed = false;
                    AppPort = 224;
                    AppDataSize = 2;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.LinkCheck = false;
                    ComplianceTest.DemodMargin = 0;
                    ComplianceTest.NbGateways = 0;
                    ComplianceTest.Running = true;
                    ComplianceTest.State = 1;
                    
                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( USE_BAND_868 )
                    LoRaMacTestSetDutyCycleOn( false );
#endif
                }
            }
            else
            {
                ComplianceTest.State = mcpsIndication->Buffer[0];
                switch( ComplianceTest.State )
                {
                case 0: // Check compliance test disable command (ii)
                    IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                    AppPort = LORAWAN_APP_PORT;
                    AppDataSize = LORAWAN_APP_DATA_SIZE;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.Running = false;
                    
                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                    LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( USE_BAND_868 )
                    LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                    break;
                case 1: // (iii, iv)
                    AppDataSize = 2;
                    break;
                case 2: // Enable confirmed messages (v)
                    IsTxConfirmed = true;
                    ComplianceTest.State = 1;
                    break;
                case 3:  // Disable confirmed messages (vi)
                    IsTxConfirmed = false;
                    ComplianceTest.State = 1;
                    break;
                case 4: // (vii)
                    AppDataSize = mcpsIndication->BufferSize;

                    AppData[0] = 4;
                    
                    uint8_t i;
                    for( i = 1; i < AppDataSize; i++ )
                    {
                        AppData[i] = mcpsIndication->Buffer[i] + 1;
                    }
                    break;
                case 5: // (viii)
                    {
                        MlmeReq_t mlmeReq;
                        mlmeReq.Type = MLME_LINK_CHECK;
                        LoRaMacMlmeRequest( &mlmeReq );
                    }
                    break;
                case 6: // (ix)
                    {
                        MlmeReq_t mlmeReq;

                        mlmeReq.Type = MLME_JOIN;

                        mlmeReq.Req.Join.DevEui = DevEui;
                        mlmeReq.Req.Join.AppEui = AppEui;
                        mlmeReq.Req.Join.AppKey = AppKey;

                        LoRaMacMlmeRequest( &mlmeReq );
                        DeviceState = DEVICE_STATE_SLEEP;
                    }
                    break;
                default:
                    break;
                }
            }
            break;
        default:
            break;
        }
    }

    // Switch LED 2 ON for each received downlink    
    SerialDisplayUpdateLedState( 2, 1 );
    SerialDisplayUpdateDownlink( LoRaMacDownlinkStatus.RxData, LoRaMacDownlinkStatus.Rssi, LoRaMacDownlinkStatus.Snr, LoRaMacDownlinkStatus.DownlinkCounter, LoRaMacDownlinkStatus.Port, LoRaMacDownlinkStatus.Buffer, LoRaMacDownlinkStatus.BufferSize );

    TimerStart( &Led2Timer );
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    MibRequestConfirm_t mibReq;
    
    if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mlmeConfirm->MlmeRequest )
        {
            case MLME_JOIN:
            {
                //Status
                mibReq.Type = MIB_NETWORK_JOINED;
                LoRaMacMibGetRequestConfirm( &mibReq );
                SerialDisplayUpdateNetworkIsJoined( mibReq.Param.IsNetworkJoined );
           
                // Status is OK, node has joined the network
                DeviceState = DEVICE_STATE_SEND;
                NextTx = true;
                break;
            }
            case MLME_LINK_CHECK:
            {
                // Check DemodMargin
                // Check NbGateways
                if( ComplianceTest.Running == true )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
                    ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
                }
                break;
            }
            default:
                break;
        }
    }
    
    NextTx = true;    
}

uint8_t GetBatteryLevel( void ) 
{
    return 0xFE;
}

/* Global variables */
volatile APP_DATA appData;

static int8_t buff[USART_BUFFER_SIZE], cmd[USART_BUFFER_SIZE];

UsartQueue_t usart_tx_pc, usart_rx_pc;  

/* Funtions */
void APP_Initialize ( void )
{
    appData.state = APP_STATE_INIT;
}

void APP_Tasks ( void )
{        
    static LoRaMacPrimitives_t LoRaMacPrimitives;
    static LoRaMacCallback_t LoRaMacCallbacks;
    static MibRequestConfirm_t mibReq;

    static uint8_t puls_lock = 0;
            
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            static char initzialized = FALSE;
            
            if (initzialized == FALSE) {
                                           
                usart_rx_pc.rd_pt = 0;
                usart_rx_pc.wr_pt = 0;
                              
                usart_tx_pc.rd_pt = 0;
                usart_tx_pc.wr_pt = 0;
                
                DRV_USART0_Open(sysObj.drvUsart0, DRV_IO_INTENT_READWRITE|DRV_IO_INTENT_NONBLOCKING);   //PC <-> MCU   
                                
                DRV_SPI0_Open(sysObj.spiObjectIdx0, DRV_IO_INTENT_READWRITE);
                DRV_SPI1_Open(sysObj.spiObjectIdx1, DRV_IO_INTENT_READWRITE); 
                
                Delay(1);
                
                SerialDisplayRefresh();
                                
                DeviceState = DEVICE_STATE_SLEEP;
                
                TimerTimeCounterInit();
                TimerInit(&Led3Timer, OnLed3TimerEvent);
                TimerSetValue(&Led3Timer, 1000);
                TimerStart(&Led3Timer);
                
                initzialized = TRUE;
            }
            
            appData.state = APP_STATE_IDLE;
            break;
        }
        
        case APP_STATE_IDLE:
        {      
            TimerCheck();
            
            switch( DeviceState )
            {
                case DEVICE_STATE_INIT:
                {
                    LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
                    LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
                    LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
                    LoRaMacCallbacks.GetBatteryLevel = GetBatteryLevel;
                    LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks );

                    TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );

                    TimerInit( &Led1Timer, OnLed1TimerEvent );
                    TimerSetValue( &Led1Timer, 2500 );

                    TimerInit( &Led2Timer, OnLed2TimerEvent );
                    TimerSetValue( &Led2Timer, 2500 );
                    
                    TimerInit( &Led2Timer, OnLed2TimerEvent );
                    TimerSetValue( &Led2Timer, 1000 );
                    TimerStart(&Led2Timer);
                    
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    mibReq.Type = MIB_PUBLIC_NETWORK;
                    mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
                    LoRaMacMibSetRequestConfirm( &mibReq );

    #if defined( USE_BAND_868 )
                    LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );

    #if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 ) 
                    LoRaMacChannelAdd( 3, ( ChannelParams_t )LC4 );
                    LoRaMacChannelAdd( 4, ( ChannelParams_t )LC5 );
                    LoRaMacChannelAdd( 5, ( ChannelParams_t )LC6 );
                    LoRaMacChannelAdd( 6, ( ChannelParams_t )LC7 );
                    LoRaMacChannelAdd( 7, ( ChannelParams_t )LC8 );
                    
                    mibReq.Type = MIB_RX2_CHANNEL;
                    mibReq.Param.Rx2Channel = ( Rx2ChannelParams_t ){ 869525000, DR_0 };
                    LoRaMacMibSetRequestConfirm( &mibReq );
    #endif

    #endif

                    //Status
                    SerialDisplayUpdateActivationMode( OVER_THE_AIR_ACTIVATION );
                    SerialDisplayUpdateAdr( LORAWAN_ADR_ON );
                    SerialDisplayUpdatePublicNetwork( LORAWAN_PUBLIC_NETWORK );
                    
                    LoRaMacUplinkStatus.UplinkCounter = 0;
                    LoRaMacDownlinkStatus.DownlinkCounter = 0;
                                                            
                    DeviceState = DEVICE_STATE_JOIN;
                    break;
                }
                case DEVICE_STATE_JOIN:
                {
    #if( OVER_THE_AIR_ACTIVATION != 0 )
                    
                    MlmeReq_t mlmeReq;

                    mlmeReq.Type = MLME_JOIN;

                    mlmeReq.Req.Join.DevEui = DevEui;
                    mlmeReq.Req.Join.AppEui = AppEui;
                    mlmeReq.Req.Join.AppKey = AppKey;

                    if( NextTx == true )
                    {
                        LoRaMacMlmeRequest( &mlmeReq );
                    }
                    DeviceState = DEVICE_STATE_SLEEP;                    
    #else
                    // Choose a random device address if not already defined in Comissioning.h
                    if( DevAddr == 0 )
                    {
                        // Random seed initialization
                        srand1( BoardGetRandomSeed( ) );

                        // Choose a random device address
                        DevAddr = randr( 0, 0x01FFFFFF );
                    }

                    mibReq.Type = MIB_NET_ID;
                    mibReq.Param.NetID = LORAWAN_NETWORK_ID;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    mibReq.Type = MIB_DEV_ADDR;
                    mibReq.Param.DevAddr = DevAddr;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    mibReq.Type = MIB_NWK_SKEY;
                    mibReq.Param.NwkSKey = NwkSKey;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    mibReq.Type = MIB_APP_SKEY;
                    mibReq.Param.AppSKey = AppSKey;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    mibReq.Type = MIB_NETWORK_JOINED;
                    mibReq.Param.IsNetworkJoined = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    DeviceState = DEVICE_STATE_SEND;
    #endif
                    break;
                }
                case DEVICE_STATE_SEND:
                {
                    if( NextTx == true )
                    {
                        //Status
                        SerialDisplayUpdateUplinkAcked( false );
                        SerialDisplayUpdateDonwlinkRxData( false );
                    
                        PrepareTxFrame( AppPort );

                        NextTx = SendFrame( );
                    }
                    if( ComplianceTest.Running == true )
                    {
                        // Schedule next packet transmission
                        TxDutyCycleTime = APP_TX_DUTYCYCLE; 
                    }
                    else
                    {
                        // Schedule next packet transmission
                        TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
                    }
                    DeviceState = DEVICE_STATE_CYCLE;
                    break;
                }
                case DEVICE_STATE_CYCLE:
                {
                    DeviceState = DEVICE_STATE_SLEEP;

                    // Schedule next packet transmission
                    TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
                    TimerStart( &TxNextPacketTimer );
                    break;
                }
                case DEVICE_STATE_SLEEP:
                {
                    // Wake up through events
                    break;
                }
                default:
                {
                    DeviceState = DEVICE_STATE_INIT;
                    break;
                }
            }
            
            //Check button P1
            if (pin_Get(PULS_1)) {
                
                if (puls_lock&0x02) break;
                else                puls_lock |= 0x02;
                
                DeviceState = DEVICE_STATE_INIT;
            }
            else puls_lock &= ~0x02;
            
            //Check button P2
            if (pin_Get(PULS_2)) {
                               
                if (puls_lock&0x01) break;
                else                puls_lock |= 0x01;
                
            }
            else puls_lock &= ~0x01;
                        
            break;
        }
        
        /* The default state should never be executed. */
        default:
        {
            
            appData.state = APP_STATE_IDLE;
            break;
        }
    }
}