//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - Out of Box
// Application Overview - This Application demonstrates Out of Box Experience 
//                        with CC32xx Launch Pad. It highlights the following 
//                        features:
//                     1. Easy Connection to CC3200 Launchpad
//                        - Direct Connection to LP by using CC3200 device in
//                          Access Point Mode(Default)
//                        - Connection using TI SmartConfigTechnology
//                     2. Easy access to CC3200 Using Internal HTTP server and 
//                        on-board webcontent
//                     3. Attractive Demos
//                        - Home Automation
//                        - Appliance Control
//                        - Security System
//                        - Thermostat
//
//*****************************************************************************

//****************************************************************************
//
//! \addtogroup oob
//! @{
//
//****************************************************************************

// Standard includes
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// Simplelink includes
#include "simplelink.h"
#include "netcfg.h"

// Driverlib includes
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "utils.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "pin.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_nvic.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"
#include "systick.h"
#include "rom_map.h"
#include <string.h>

// OS includes
#include "osi.h"

// Common interface includes
#include "gpio_if.h"
#include "uart_if.h"
#include "i2c_if.h"
#include "common.h"

// App Includes
#include "device_status.h"
#include "smartconfig.h"
#include "tmp006drv.h"
#include "bma222drv.h"
#include "pinmux.h"

#define APPLICATION_VERSION              "1.4.0"
#define APP_NAME                         "Out of Box"
#define OOB_TASK_PRIORITY                1
#define SPAWN_TASK_PRIORITY              9
#define OSI_STACK_SIZE                   2048
#define AP_SSID_LEN_MAX                 32
#define SH_GPIO_3                       3       /* P58 - Device Mode */
#define AUTO_CONNECTION_TIMEOUT_COUNT   50      /* 5 Sec */
#define SL_STOP_TIMEOUT                 200

typedef enum
{
  LED_OFF = 0,
  LED_ON,
  LED_BLINK
}eLEDStatus;

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
static const char pcDigits[] = "0123456789";
static unsigned char POST_token[] = "__SL_P_ULD";
static unsigned char GET_token_TEMP[]  = "__SL_G_UTP";
static unsigned char GET_token_ACC[]  = "__SL_G_UAC";
static unsigned char GET_token_UIC[]  = "__SL_G_UIC";
static int g_iInternetAccess = -1;
static unsigned char g_ucDryerRunning = 0;
static unsigned int g_uiDeviceModeConfig = ROLE_STA; //default is STA mode
static unsigned char g_ucLEDStatus = LED_OFF;
static unsigned long  g_ulStatus = 0;//SimpleLink Status
static unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
static unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
static int g_tmp006Status = 0;
static int g_bma222Status = 0;
#define SYSTICK_INTERRUPT_FREQUENCY_MS 10
// Desired temperature check interval in milliseconds
#define TEMPERATURE_CHECK_INTERVAL_MS 1000
// Calculate how many SysTick interrupts occur in the desired interval
#define TEMPERATURE_CHECK_INTERVAL (TEMPERATURE_CHECK_INTERVAL_MS / SYSTICK_INTERRUPT_FREQUENCY_MS)

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
#define SYSCLKFREQ 80000000ULL
#define TICKS_TO_US(ticks) \
    ((((ticks) / SYSCLKFREQ) * 1000000ULL) + \
    ((((ticks) % SYSCLKFREQ) * 1000000ULL) / SYSCLKFREQ))\

#define US_TO_TICKS(us) ((SYSCLKFREQ / 1000000ULL) * (us))

// systick reload value set to 40ms period
// (PERIOD_SEC) * (SYSCLKFREQ) = PERIOD_TICKS
#define SYSTICK_RELOAD_VAL 3200000UL

// track systick counter periods elapsed
// if it is not 0, we know the transmission ended
volatile int systick_wrapped = 0;


extern void (* const g_pfnVectors[])(void);

// struct for NEC packet format
typedef struct NECCode_t {
    unsigned char addr;
    unsigned char naddr;
    unsigned char data;
    unsigned char ndata;
} NECCode_t;

// union for writing packet into the struct
union NECCode_u {
    unsigned int raw;
    NECCode_t ir_code;
} NECCode_u;

// indicates presence of a valid ir code
volatile int pending_ir_code = 0;
// a global struct to track most recent received NEC code
volatile NECCode_t ir_code = { .addr = 0, .naddr = 0xFF, .data = 0, .ndata = 0xFF };

#define NEC_START_PULSE_MIN 4000 // in us
#define NEC_1_PULSE_MIN     1500 // in us
#define NEC_0_PULSE_MIN     500  // in us

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

// an example of how you can use structs to organize your pin settings for easier maintenance
typedef struct pin_t {
    unsigned long port;
    unsigned int pin;
    unsigned char state;
} pin_t;

// pin 50 to read IR signal
static pin_t ir_out = { .port = GPIOA0_BASE, .pin = 0x01, .state = 1 };

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
static void BoardInit(void);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//*****************************************************************************

/**
 * Reset SysTick Counter
 */
static inline void SysTickReset(void) {
    HWREG(NVIC_ST_CURRENT) = 1;
    systick_wrapped = 0;
}

/**
 * Interrupt handler for GPIOA0 port
 *
 * Only used here for decoding IR transmissions
 */
static void GPIOA0IntHandler(void) {
    // static variables for decoding IR transmissions
    // state needs to persist across invocations
    static int ir_transmission_active = 0;
    static uint64_t ulprev_systick_cnt = SYSTICK_RELOAD_VAL;
    static union NECCode_u data = { 0 };
    static int bit_cnt = 0;

    // temporary variables
    uint64_t ulcurr_systick_cnt;
    uint64_t ulsystick_delta;
    uint64_t ulsystick_delta_us;

    unsigned long ulStatus;

    // ulStatus is the masked status register
    // each 1 bit indicates the corresponding pin has an active interrupt
    ulStatus = MAP_GPIOIntStatus(ir_out.port, true);
    // see section 2.2.4 in technical reference manual
    // for why we clear interrupts at beginning of handler
    MAP_GPIOIntClear(ir_out.port, ulStatus);        // clear interrupts on GPIOA0

    // since the same handler is called for every interrupt-enabled pin
    // on the same port, it's good practice to specify actions
    // based on which pin has an active interrupt

    if (ulStatus & ir_out.pin) {
        if (ir_out.state) {
            // preceding state == 1 (falling edge)

            // read the current systick counter value
            ulcurr_systick_cnt = MAP_SysTickValueGet();

            if (!systick_wrapped) {
                // this is a pulse in the ir transmission

                // calculate the pulse width
                ulsystick_delta = ulprev_systick_cnt - ulcurr_systick_cnt;
                ulsystick_delta_us = TICKS_TO_US(ulsystick_delta);

                // decode meaning based on pulse width
                if (ulsystick_delta_us > NEC_START_PULSE_MIN) {
                    // beginning of a transmission, prepare to receive data
                    ir_transmission_active = 1;

                    // clear previous data
                    data.raw = 0;
                    bit_cnt = 0;

                } else if ((ulsystick_delta_us > NEC_1_PULSE_MIN) && ir_transmission_active) {
                    // a 1 bit detected
                    bit_cnt += 1;

                    // add the bit to raw
                    data.raw |= (1 << (bit_cnt - 1));

                } else if ((ulsystick_delta_us > NEC_0_PULSE_MIN) && ir_transmission_active) {
                    // a 0 bit detected
                    bit_cnt += 1;

                    // don't need to add the bit, since it's already 0
                }

                if (bit_cnt >= 32) {
                    // finished transmission, write to the global
                    ir_code = data.ir_code;
                    pending_ir_code = 1;
                }



            } else {
                // this is the beginning of a new ir transmission
                systick_wrapped = 0;
            }

        } else {
            // preceding state == 0 (rising edge)

            // always restart the systick counter on the rising edge
            SysTickReset();
        }

        // toggle state bit
        ir_out.state ^= 1;
    }
}

/**
 * SysTick Interrupt Handler
 *
 * Keep track of whether the systick counter wrapped
 */
static void SysTickHandler(void) {
    systick_wrapped = 1;
}


#ifdef USE_FREERTOS
//*****************************************************************************
//
//! Application defined hook (or callback) function - the tick hook.
//! The tick interrupt can optionally call this
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void
vApplicationTickHook( void )
{
}

//*****************************************************************************
//
//! Application defined hook (or callback) function - assert
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void
vAssertCalled( const char *pcFile, unsigned long ulLine )
{
    while(1)
    {

    }
}

//*****************************************************************************
//
//! Application defined idle task hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void
vApplicationIdleHook( void )
{

}

//*****************************************************************************
//
//! Application provided stack overflow hook function.
//!
//! \param  handle of the offending task
//! \param  name  of the offending task
//!
//! \return none
//!
//*****************************************************************************
void
vApplicationStackOverflowHook( OsiTaskHandle *pxTask, signed char *pcTaskName)
{
    ( void ) pxTask;
    ( void ) pcTaskName;

    for( ;; );
}

void vApplicationMallocFailedHook()
{
    while(1)
  {
    // Infinite loop;
  }
}
#endif

//*****************************************************************************
//
//! itoa
//!
//!    @brief  Convert integer to ASCII in decimal base
//!
//!     @param  cNum is input integer number to convert
//!     @param  cString is output string
//!
//!     @return number of ASCII parameters
//!
//!
//
//*****************************************************************************
static unsigned short itoa(char cNum, char *cString)
{
    char* ptr;
    char uTemp = cNum;
    unsigned short length;

    // value 0 is a special case
    if (cNum == 0)
    {
        length = 1;
        *cString = '0';

        return length;
    }

    // Find out the length of the number, in decimal base
    length = 0;
    while (uTemp > 0)
    {
        uTemp /= 10;
        length++;
    }

    // Do the actual formatting, right to left
    uTemp = cNum;
    ptr = cString + length;
    while (uTemp > 0)
    {
        --ptr;
        *ptr = pcDigits[uTemp % 10];
        uTemp /= 10;
    }

    return length;
}


//*****************************************************************************
//
//! ReadAccSensor
//!
//!    @brief  Read Accelerometer Data from Sensor
//!
//!
//!     @return none
//!
//!
//
//*****************************************************************************
void ReadAccSensor()
{
    //Define Accelerometer Threshold to Detect Movement
    const short csAccThreshold    = 5;

    signed char cAccXT1,cAccYT1,cAccZT1;
    signed char cAccXT2,cAccYT2,cAccZT2;
    signed short sDelAccX, sDelAccY, sDelAccZ;
    int iRet = -1;
    int iCount = 0;
      
    iRet = BMA222ReadNew(&cAccXT1, &cAccYT1, &cAccZT1);
    if(iRet)
    {
        //In case of error/ No New Data return
        return;
    }
    for(iCount=0;iCount<2;iCount++)
    {
        MAP_UtilsDelay((90*80*1000)); //30msec
        iRet = BMA222ReadNew(&cAccXT2, &cAccYT2, &cAccZT2);
        if(iRet)
        {
            //In case of error/ No New Data continue
            iRet = 0;
            continue;
        }

        else
        {                       
            sDelAccX = abs((signed short)cAccXT2 - (signed short)cAccXT1);
            sDelAccY = abs((signed short)cAccYT2 - (signed short)cAccYT1);
            sDelAccZ = abs((signed short)cAccZT2 - (signed short)cAccZT1);

            //Compare with Pre defined Threshold
            if(sDelAccX > csAccThreshold || sDelAccY > csAccThreshold ||
               sDelAccZ > csAccThreshold)
            {
                //Device Movement Detected, Break and Return
                g_ucDryerRunning = 1;
                break;
            }
            else
            {
                //Device Movement Static
                g_ucDryerRunning = 0;
            }
        }
    }
       
}

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(pWlanEvent == NULL)
    {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }
    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'
            // Applications can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] Device Connected to the AP: %s , "
                       "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                      g_ucConnectionSSID,g_ucConnectionBSSID[0],
                      g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                      g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                      g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION
            if(SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                UART_PRINT("[WLAN EVENT] Device disconnected from the AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on application's "
                           "request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else
            {
                UART_PRINT("[WLAN ERROR] Device disconnected from the AP AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        case SL_WLAN_STA_CONNECTED_EVENT:
        {
            // when device is in AP mode and any client connects to device cc3xxx
            //SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            //CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION_FAILED);

            //
            // Information about the connected client (like SSID, MAC etc) will
            // be available in 'slPeerInfoAsyncResponse_t' - Applications
            // can use it if required
            //
            // slPeerInfoAsyncResponse_t *pEventData = NULL;
            // pEventData = &pSlWlanEvent->EventData.APModeStaConnected;
            //

            UART_PRINT("[WLAN EVENT] Station connected to device\n\r");
        }
        break;

        case SL_WLAN_STA_DISCONNECTED_EVENT:
        {
            // when client disconnects from device (AP)
            //CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            //CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

            //
            // Information about the connected client (like SSID, MAC etc) will
            // be available in 'slPeerInfoAsyncResponse_t' - Applications
            // can use it if required
            //
            // slPeerInfoAsyncResponse_t *pEventData = NULL;
            // pEventData = &pSlWlanEvent->EventData.APModestaDisconnected;
            //
            UART_PRINT("[WLAN EVENT] Station disconnected from device\n\r");
        }
        break;

        case SL_WLAN_SMART_CONFIG_COMPLETE_EVENT:
        {
            //SET_STATUS_BIT(g_ulStatus, STATUS_BIT_SMARTCONFIG_START);

            //
            // Information about the SmartConfig details (like Status, SSID,
            // Token etc) will be available in 'slSmartConfigStartAsyncResponse_t'
            // - Applications can use it if required
            //
            //  slSmartConfigStartAsyncResponse_t *pEventData = NULL;
            //  pEventData = &pSlWlanEvent->EventData.smartConfigStartResponse;
            //

        }
        break;

        case SL_WLAN_SMART_CONFIG_STOP_EVENT:
        {
            // SmartConfig operation finished
            //CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_SMARTCONFIG_START);

            //
            // Information about the SmartConfig details (like Status, padding
            // etc) will be available in 'slSmartConfigStopAsyncResponse_t' -
            // Applications can use it if required
            //
            // slSmartConfigStopAsyncResponse_t *pEventData = NULL;
            // pEventData = &pSlWlanEvent->EventData.smartConfigStopResponse;
            //
        }
        break;

        default:
        {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(pNetAppEvent == NULL)
    {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }

    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                       "Gateway=%d.%d.%d.%d\n\r",
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));

            UNUSED(pEventData);
        }
        break;

        case SL_NETAPP_IP_LEASED_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

            //
            // Information about the IP-Leased details(like IP-Leased,lease-time,
            // mac etc) will be available in 'SlIpLeasedAsync_t' - Applications
            // can use it if required
            //
            // SlIpLeasedAsync_t *pEventData = NULL;
            // pEventData = &pNetAppEvent->EventData.ipLeased;
            //

        }
        break;

        case SL_NETAPP_IP_RELEASED_EVENT:
        {
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

            //
            // Information about the IP-Released details (like IP-address, mac
            // etc) will be available in 'SlIpReleasedAsync_t' - Applications
            // can use it if required
            //
            // SlIpReleasedAsync_t *pEventData = NULL;
            // pEventData = &pNetAppEvent->EventData.ipReleased;
            //
        }
        break;

        default:
        {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pSlHttpServerEvent, 
                                SlHttpServerResponse_t *pSlHttpServerResponse)
{
    switch (pSlHttpServerEvent->Event)
    {
        case SL_NETAPP_HTTPGETTOKENVALUE_EVENT:
        {
            unsigned char *ptr;

            ptr = pSlHttpServerResponse->ResponseData.token_value.data;
            pSlHttpServerResponse->ResponseData.token_value.len = 0;
            if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, 
                    GET_token_TEMP, strlen((const char *)GET_token_TEMP)) == 0)
            {
                if (g_tmp006Status)
                {
                    float fCurrentTemp;
                    TMP006DrvGetTemp(&fCurrentTemp);
                    char cTemp = (char)fCurrentTemp;
                    short sTempLen = itoa(cTemp,(char*)ptr);
                    ptr[sTempLen++] = ' ';
                    ptr[sTempLen] = 'F';
                    pSlHttpServerResponse->ResponseData.token_value.len += sTempLen;
                }
                else
                {
                    char cTemp = (char)76;
                    short sTempLen = itoa(cTemp,(char*)ptr);
                    ptr[sTempLen++] = ' ';
                    ptr[sTempLen] = 'F';
                    pSlHttpServerResponse->ResponseData.token_value.len += sTempLen;
                }
            }

            if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, 
                      GET_token_UIC, strlen((const char *)GET_token_UIC)) == 0)
            {
                if(g_iInternetAccess==0)
                    strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,"1");
                else
                    strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,"0");
                pSlHttpServerResponse->ResponseData.token_value.len =  1;
            }

            if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, 
                       GET_token_ACC, strlen((const char *)GET_token_ACC)) == 0)
            {
                if (g_bma222Status)
                {
                    ReadAccSensor();
                }
                else
                {
                    g_ucDryerRunning = 0;
                }

                if(g_ucDryerRunning)
                {
                    strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,"Running");
                    pSlHttpServerResponse->ResponseData.token_value.len += strlen("Running");
                }
                else
                {
                    strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,"Stopped");
                    pSlHttpServerResponse->ResponseData.token_value.len += strlen("Stopped");
                }
            }
        }
            break;

        case SL_NETAPP_HTTPPOSTTOKENVALUE_EVENT:
        {
            unsigned char led;
            unsigned char *ptr = pSlHttpServerEvent->EventData.httpPostData.token_name.data;

            //g_ucLEDStatus = 0;
            if(memcmp(ptr, POST_token, strlen((const char *)POST_token)) == 0)
            {
                ptr = pSlHttpServerEvent->EventData.httpPostData.token_value.data;
                if(memcmp(ptr, "LED", 3) != 0)
                    break;
                ptr += 3;
                led = *ptr;
                ptr += 2;
                if(led == '1')
                {
                    if(memcmp(ptr, "ON", 2) == 0)
                    {
                        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
                                                g_ucLEDStatus = LED_ON;

                    }
                    else if(memcmp(ptr, "Blink", 5) == 0)
                    {
                        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
                        g_ucLEDStatus = LED_BLINK;
                    }
                    else
                    {
                        GPIO_IF_LedOff(MCU_RED_LED_GPIO);
                                                g_ucLEDStatus = LED_OFF;
                    }
                }
                else if(led == '2')
                {
                    if(memcmp(ptr, "ON", 2) == 0)
                    {
                        GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
                    }
                    else if(memcmp(ptr, "Blink", 5) == 0)
                    {
                        GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
                        g_ucLEDStatus = 1;
                    }
                    else
                    {
                        GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
                    }
                }

            }
        }
            break;
        default:
            break;
    }
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    if(pDevEvent == NULL)
    {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if(pSock == NULL)
    {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }
    //
    // This application doesn't work w/ socket - Events are not expected
    //
    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status)
            {
                case SL_ECLOSE: 
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n", 
                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default: 
                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

        default:
            UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }
}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************

//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    None
//!
//! \return None
//!
//*****************************************************************************
static void InitializeAppVariables()
{
    g_ulStatus = 0;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    g_iInternetAccess = -1;
    g_ucDryerRunning = 0;
    g_uiDeviceModeConfig = ROLE_STA; //default is STA mode
    g_ucLEDStatus = LED_OFF;    
}


//****************************************************************************
//
//! Confgiures the mode in which the device will work
//!
//! \param iMode is the current mode of the device
//!
//!
//! \return   SlWlanMode_t
//!                        
//
//****************************************************************************
static int ConfigureMode(int iMode)
{
    long   lRetVal = -1;

    lRetVal = sl_WlanSetMode(iMode);
    ASSERT_ON_ERROR(lRetVal);

    /* Restart Network processor */
    lRetVal = sl_Stop(SL_STOP_TIMEOUT);

    // reset status bits
    CLR_STATUS_BIT_ALL(g_ulStatus);

    return sl_Start(NULL,NULL,NULL);
}


//****************************************************************************
//
//!    \brief Connects to the Network in AP or STA Mode - If ForceAP Jumper is
//!                                             Placed, Force it to AP mode
//!
//! \return  0 - Success
//!            -1 - Failure
//
//****************************************************************************
long ConnectToNetwork()
{
    long lRetVal = -1;
    unsigned int uiConnectTimeoutCnt =0;

    // staring simplelink
    lRetVal =  sl_Start(NULL,NULL,NULL);
    ASSERT_ON_ERROR( lRetVal);

    // Device is in AP Mode and Force AP Jumper is not Connected
    if(ROLE_STA != lRetVal && g_uiDeviceModeConfig == ROLE_STA )
    {
        if (ROLE_AP == lRetVal)
        {
            // If the device is in AP mode, we need to wait for this event 
            // before doing anything 
            while(!IS_IP_ACQUIRED(g_ulStatus))
            {
            #ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
            #endif
            }
        }
        //Switch to STA Mode
        lRetVal = ConfigureMode(ROLE_STA);
        ASSERT_ON_ERROR( lRetVal);
    }

    //Device is in STA Mode and Force AP Jumper is Connected
    if(ROLE_AP != lRetVal && g_uiDeviceModeConfig == ROLE_AP )
    {
         //Switch to AP Mode
         lRetVal = ConfigureMode(ROLE_AP);
         ASSERT_ON_ERROR( lRetVal);

    }

    //No Mode Change Required
    if(lRetVal == ROLE_AP)
    {
        //waiting for the AP to acquire IP address from Internal DHCP Server
        // If the device is in AP mode, we need to wait for this event 
        // before doing anything 
        while(!IS_IP_ACQUIRED(g_ulStatus))
        {
        #ifndef SL_PLATFORM_MULTI_THREADED
            _SlNonOsMainLoopTask(); 
        #endif
        }
        //Stop Internal HTTP Server
        lRetVal = sl_NetAppStop(SL_NET_APP_HTTP_SERVER_ID);
        ASSERT_ON_ERROR( lRetVal);

        //Start Internal HTTP Server
        lRetVal = sl_NetAppStart(SL_NET_APP_HTTP_SERVER_ID);
        ASSERT_ON_ERROR( lRetVal);

       char cCount=0;
       
       //Blink LED 3 times to Indicate AP Mode
       for(cCount=0;cCount<3;cCount++)
       {
           //Turn RED LED On
           GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           osi_Sleep(400);
           
           //Turn RED LED Off
           GPIO_IF_LedOff(MCU_RED_LED_GPIO);
           osi_Sleep(400);
       }

       char ssid[32];
       unsigned short len = 32;
       unsigned short config_opt = WLAN_AP_OPT_SSID;
       sl_WlanGet(SL_WLAN_CFG_AP_ID, &config_opt , &len, (unsigned char* )ssid);
       UART_PRINT("\n\r Connect to : \'%s\'\n\r\n\r",ssid);
    }
    else
    {
        //Stop Internal HTTP Server
        lRetVal = sl_NetAppStop(SL_NET_APP_HTTP_SERVER_ID);
        ASSERT_ON_ERROR( lRetVal);

        //Start Internal HTTP Server
        lRetVal = sl_NetAppStart(SL_NET_APP_HTTP_SERVER_ID);
        ASSERT_ON_ERROR( lRetVal);

        //waiting for the device to Auto Connect
        while(uiConnectTimeoutCnt<AUTO_CONNECTION_TIMEOUT_COUNT &&
            ((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))) 
        {
            //Turn RED LED On
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            osi_Sleep(50);
            
            //Turn RED LED Off
            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
            osi_Sleep(50);
            
            uiConnectTimeoutCnt++;
        }
        //Couldn't connect Using Auto Profile
        if(uiConnectTimeoutCnt == AUTO_CONNECTION_TIMEOUT_COUNT)
        {
            //Blink Red LED to Indicate Connection Error
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            
            CLR_STATUS_BIT_ALL(g_ulStatus);

            //Connect Using Smart Config
            UART_PRINT("\n\r Connect with SmartConfig \n\r\n\r");
            lRetVal = SmartConfigConnect();
            ASSERT_ON_ERROR(lRetVal);

            //Waiting for the device to Auto Connect
            while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
            {
                MAP_UtilsDelay(500);              
            }
    }
    //Turn RED LED Off
    GPIO_IF_LedOff(MCU_RED_LED_GPIO);

    g_iInternetAccess = ConnectionTest();

    }
    return SUCCESS;
}


//****************************************************************************
//
//!    \brief Read Force AP GPIO and Configure Mode - 1(Access Point Mode)
//!                                                  - 0 (Station Mode)
//!
//! \return                        None
//
//****************************************************************************
static void ReadDeviceConfiguration()
{
    unsigned int uiGPIOPort;
    unsigned char pucGPIOPin;
    unsigned char ucPinValue;
        
    //Read GPIO
    GPIO_IF_GetPortNPin(SH_GPIO_3,&uiGPIOPort,&pucGPIOPin);
    ucPinValue = GPIO_IF_Get(SH_GPIO_3,uiGPIOPort,pucGPIOPin);
        
    //If Connected to VCC, Mode is AP
    if(ucPinValue == 1)
    {
        //AP Mode
        g_uiDeviceModeConfig = ROLE_AP;
    }
    else
    {
        //STA Mode
        g_uiDeviceModeConfig = ROLE_STA;
    }

}

//****************************************************************************
//
//!    \brief OOB Application Main Task - Initializes SimpleLink Driver and
//!                                              Handles HTTP Requests
//! \param[in]                  pvParameters is the data passed to the Task
//!
//! \return                        None
//
//****************************************************************************
static void OOBTask(void *pvParameters)
{

    long lRetVal = -1;
       float fCurrentTemp = 0.0;

    //Handle Async Events
    while(1)
    {
        //LED Actions
        lRetVal = TMP006DrvGetTemp(&fCurrentTemp);
                if(lRetVal == SUCCESS)
                {
                    // Print the current temperature to the UART console
                    UART_PRINT("Current Temperature: %f F\n\r", fCurrentTemp);
                }
                else
                {
                    UART_PRINT("Failed to read temperature from TMP006 sensor\n\r");
                }

                // Delay for a bit before reading the temperature again
                osi_Sleep(1000); // Delay for 5000 milliseconds (5 seconds)
}}

//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void
DisplayBanner(char * AppName)
{
    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\t\t     CC3200 %s Application       \n\r", AppName);
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\n\n\n\r");
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);

        // Enable Processor
        //
        MAP_IntMasterEnable();
        MAP_IntEnable(FAULT_SYSTICK);

        PRCMCC3200MCUInit();

}
static void SysTickInit(void) {
    systick_wrapped = 1;

    MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);

    MAP_SysTickIntRegister(SysTickHandler);

    MAP_SysTickIntEnable();
    MAP_SysTickEnable();
}
//*

//****************************************************************************
//                            MAIN FUNCTION
//****************************************************************************
//void CheckTemperatureTask(void *pvParameters) {
//    float currentTemperature = 0.0;
//    static uint32_t lastSysTick = 0;
//    UART_PRINT("CheckTemperatureTask started\n\r");
//
//    while (1) {
//        if (pending_ir_code) {
//                   Report("Received NEC Data: addr = %d\tdata = %d\r\n", ir_code.addr, ir_code.data);
//
//                   if (ir_code.data == 1) {
//                       Report("2 got!");
//                       GPIOPinWrite(GPIOA0_BASE, 0x8,  0x8);
//                   }
//                   if (ir_code.data == 3) {
//
//                   }
//                   if (ir_code.data == 5) {
//
//                   }
//                   if (ir_code.data == 7) {
//
//                   }
//                   pending_ir_code = 0;
//               }
//        if (systickCounter - lastSysTick >= TEMPERATURE_CHECK_INTERVAL) {
//                    lastSysTick = systickCounter;  // Update the last check time
//
//                    // Perform the temperature check
//                    if (TMP006DrvGetTemp(&currentTemperature) == SUCCESS) {
//                       // UART_PRINT("Current Temperature: %f F\n\r", currentTemperature);
//                    } else {
//                        UART_PRINT("Failed to read temperature from TMP006 sensor\n\r");
//                    }
//                }
//
////        osi_Sleep(10); // Delay for a bit before reading the temperature again
//    }
//}



void main()
{
    int waterflag = 0;
    long   lRetVal = -1;
    unsigned long ulStatus;
    //
    // Board Initialization
    //
    BoardInit();
    
    //
    // Configure the pinmux settings for the peripherals exercised
    //
    PinMuxConfig();
    SysTickInit();

    PinConfigSet(PIN_58,PIN_STRENGTH_2MA|PIN_STRENGTH_4MA,PIN_TYPE_STD_PD);

    // Initialize Global Variables
    InitializeAppVariables();
    


    // UART Init
    //
    InitTerm();
    ClearTerm();
    DisplayBanner(APP_NAME);

    //
    // LED Init
    //
//    GPIO_IF_LedConfigure(LED1);
//
//    //Turn Off the LEDs
//    GPIO_IF_LedOff(MCU_RED_LED_GPIO);


    //
    // I2C Init
    //
    lRetVal = I2C_IF_Open(I2C_MASTER_MODE_FST);
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        DBG_PRINT("I2C open failed\n\r");
    }
    else
    {
        //Init Accelerometer Sensor
        lRetVal = BMA222Open();
        if(lRetVal < 0)
        {
            ERR_PRINT(lRetVal);
            DBG_PRINT("BMA222 sensor has error. Check jumpers on LaunchPad.\n\r\n\r");
        }
        else
        {
            g_bma222Status = 1;
        }

        //Init Temprature Sensor
        lRetVal = TMP006DrvOpen();
        if(lRetVal < 0)
        {
            ERR_PRINT(lRetVal);
            DBG_PRINT("TMP006 sensor has error. Check jumpers on LaunchPad or TMP006 may not be populated. \n\r    Default temperature will be 76 degrees F\n\r\n\r");
        }
        else
        {
            g_tmp006Status = 1;
        }
    }

    //
    // Simplelinkspawntask
    //
    MAP_GPIOIntRegister(ir_out.port, GPIOA0IntHandler);

        //
        // Configure interrupts on both rising and falling edges for pin 50
        //
        MAP_GPIOIntTypeSet(ir_out.port, ir_out.pin, GPIO_BOTH_EDGES);   // read ir_output

        ulStatus = MAP_GPIOIntStatus (ir_out.port, false);
        MAP_GPIOIntClear(ir_out.port, ulStatus);            // clear interrupts on GPIOA2

        // Enable interrupts on ir_output
        MAP_GPIOIntEnable(ir_out.port, ir_out.pin);
//    lRetVal = osi_TaskCreate(CheckTemperatureTask, "TempCheckTask", OSI_STACK_SIZE, NULL, OOB_TASK_PRIORITY, NULL);
//        if (lRetVal < 0) {
//            ERR_PRINT(lRetVal);
//            LOOP_FOREVER();
//        }
//
//        // Start OS Scheduler
//        osi_start();
        float currentTemperature = 0.0;
    while (1)
    {


        if (TMP006DrvGetTemp(&currentTemperature) == SUCCESS) {
                               //UART_PRINT("Current Temperature: %f F\n\r", currentTemperature);
                            } else {
                                UART_PRINT("Failed to read temperature from TMP006 sensor\n\r");
                            }
        if (currentTemperature>= 90){
            if (waterflag == 0) {
                Report("spray enabled!");
                GPIOPinWrite(GPIOA0_BASE, 0x20,  0x20);
                waterflag = 1;
            }
        } else {
            if (waterflag == 1) {
                Report("spray disabled!");
                GPIOPinWrite(GPIOA0_BASE, 0x20,  0);
                waterflag = 0;
            }

        }
        if (pending_ir_code) {
                          //Report("Received NEC Data: addr = %d\tdata = %d\r\n", ir_code.addr, ir_code.data);

                          if (ir_code.data == 1) {
                              Report("Move F");
                              GPIOPinWrite(GPIOA0_BASE, 0x8,  0x8);//left rear wheel
                              GPIOPinWrite(GPIOA0_BASE, 0x40,  0x40);//right rear wheel
                              GPIOPinWrite(GPIOA0_BASE, 0x8,  0);//left wheel
                              GPIOPinWrite(GPIOA0_BASE, 0x40,  0);//right wheel
                          }
                          if (ir_code.data == 3) {
                              Report("Move L");
                              GPIOPinWrite(GPIOA0_BASE, 0x40,  0x40);//right rear wheel

                              GPIOPinWrite(GPIOA0_BASE, 0x40,  0);//right wheel
                          }
                          if (ir_code.data == 5) {
                              Report("Move R");
                              GPIOPinWrite(GPIOA0_BASE, 0x8,  0x8);//left rear wheel

                              GPIOPinWrite(GPIOA0_BASE, 0x8,  0);//left wheel
                          }
                          if (ir_code.data == 7) {
                              Report("Move B");
                              GPIOPinWrite(GPIOA0_BASE, 0x80,  0x8);//left front wheel
                              GPIOPinWrite(GPIOA1_BASE, 0x1,  0x40);//right front wheel

                              GPIOPinWrite(GPIOA0_BASE, 0x80,  0);//left wheel
                              GPIOPinWrite(GPIOA1_BASE, 0x1,  0);//right wheel
                          }
                          pending_ir_code = 0;
                      }
    }

}

