/*
 * main.c - MQTT Twitter Controlled RGB LED sample application
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * Application Name     -   MQTT Twitter Controlled RGB LED
 * Application Overview -   This is a sample application demonstrating how
 *                          to connect to an MQTT broker and publish/subscribe
 *                          to topics. A web server was created to search
 *                          for all public tweets containing the hashtag
 *                          #MSP432LaunchPad and a RGB(#, #, #) control command.
 *                          The web server publishes the RGB() command to all
 *                          LaunchPads running the demo. The application also
 *                          publishes the device's MAC Address when push
 *                          button S1 is pressed on the LaunchPad. The web
 *                          server then tweets the received MAC Address on
 *                          the MSP LaunchPad Twitter account.
 *
 *                          Refer to README.txt for more information
 */

// Standard includes
#include <stdlib.h>
#include <string.h>

#include "driverlib.h"
#include "simplelink.h"
#include "sl_common.h"
#include "MQTTClient.h"

//#include "msp.h"
//#include "..\inc\bump.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTick.h"
#include "..\inc\LaunchPad.h"
//#include "..\inc\MotorSimple.h"
#include "..\inc\Motor.h"
//#include "..\inc\Bump.h"
#include "..\inc\odometry.h"

#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/I2CB1.h"
#include "../inc/CortexM.h"
#include "../inc/opt3101.h"
#include "../inc/LaunchPad.h"
#include "../inc/UART0.h"
#include "../inc/BumpInt.h"

//#include "..\inc\Bump.h"
//#include "..\inc\SysTickInts.h"
#include "..\inc\Motor.h"
#include "..\inc\PWM.h"
#include <string.h>
#include <stdio.h>

/*
 * Values for below macros shall be modified per the access-point's (AP) properties
 * SimpleLink device will connect to following AP when the application is executed
 */
//#define SSID_NAME       "ECE DESIGN LAB 2.4"       /* Access point name to connect to. */
#define SSID_NAME       "Wifi"       /* Access point name to connect to. */
#define SEC_TYPE        SL_SEC_TYPE_WPA_WPA2     /* Security type of the Access piont */
//#define PASSKEY         "ecedesignlab12345"   /* Password in case of secure AP */
#define PASSKEY         "wifipassword"   /* Password in case of secure AP */
#define PASSKEY_LEN     pal_Strlen(PASSKEY)  /* Password length in case of secure AP */

/*
 * MQTT server and topic properties that shall be modified per application
 */
#define MQTT_BROKER_SERVER  "broker.hivemq.com"
#define SUBSCRIBE_TOPIC "arf"
#define PUBLISH_TOPIC "arf"

// MQTT message buffer size
#define BUFF_SIZE 32


#define APPLICATION_VERSION "1.0.0"

#define MCLK_FREQUENCY 48000000
#define PWM_PERIOD 255

#define SL_STOP_TIMEOUT        0xFF

#define SMALL_BUF           32
#define MAX_SEND_BUF_SIZE   512
#define MAX_SEND_RCV_SIZE   1024

//BT & MOVE
#define Init UART0_Init
#define Clear UartClear
#define SetCursor UartSetCur
#define OutString UART0_OutString
#define OutChar UART0_OutChar
#define OutUDec UART0_OutUDec


uint32_t Distances[3];
uint32_t FilteredDistances[3];
uint32_t Amplitudes[3];
uint32_t TxChannel;
uint32_t StartTime;
uint32_t TimeToConvert; // in msec
uint32_t i;
uint32_t Error;
int32_t UR, UL;  // PWM duty 0 to 14,998
int32_t SetPoint = 250; // mm //was 250
int32_t LeftDistance,CenterDistance,RightDistance; // mm
uint32_t ON = 0;
char hope;


//Controller
#define PWMNOMINAL 14999
#define SWING 2000
#define DESIRED 250
#define TOOFAR 400 // was 400

// assumes track is 500mm
int32_t Mode=0; // 0 stop, 1 run
int32_t Ki=1;  // integral controller gain
int32_t Kp=4;  // proportional controller gain //was 4
int32_t status = 0;


//-----------------Bluetooth stuff--------------------------------------------------------------
//void mapBits(char* x){
//    int N;
//    if(x[0] != 0x0A){
//        char ch = x[0];
//        printf("ch: %c\n", ch);
//        N = (int)(ch)+0;
//    }
//
//
//    switch (N) {
//    case 0x64: // forward
//        ON = 1;
//       break;
//
//    case 0x73: //stop:
//       Motor_Stop();
//       ON = 0;
//       break;
//    default:
//       break;
//    }
//}

//----------------UART--------------------------------------------------------------------------

void UartSetCur(uint8_t newX, uint8_t newY)
{
  if(newX == 6){
    UART0_OutString("\n\rTxChannel= ");
    UART0_OutUDec(newY-1);
    UART0_OutString(" Distance= ");
  }else{
    UART0_OutString("\n\r");
  }
}
void UartClear(void){UART0_OutString("\n\r");};

bool pollDistanceSensor(void)
{
  if(OPT3101_CheckDistanceSensor())
  {
    TxChannel = OPT3101_GetMeasurement(Distances,Amplitudes);
    return true;
  }
  return false;
}

int32_t Left(int32_t left){
return (1247*left)/2048 + 22;
}

int32_t Right(int32_t right){
  return  (right*(59*right + 7305) + 2348974)/32768;
}

void Controller_Right(void){ // runs at 100 Hz
  if(Mode){
    if((RightDistance>DESIRED)){
      SetPoint = (RightDistance)/2;
    }else{
      SetPoint = DESIRED;
    }
    /*if(LeftDistance < RightDistance ){
      Error = LeftDistance-SetPoint;
    }else {
      Error = SetPoint-RightDistance;
    }*/

    Error = SetPoint-RightDistance;
    //UR = UR + Ki*Error;      // adjust right motor
    UR = PWMNOMINAL+Kp*Error; // proportional control
    UR = UR + Ki*Error;      // adjust right motor
    UL = PWMNOMINAL-Kp*Error; // proportional control
    if(UR < (PWMNOMINAL-SWING)){UR = PWMNOMINAL-SWING;} // 3,000 to 7,000
    if(UR > (PWMNOMINAL+SWING)){UR = PWMNOMINAL+SWING;}
    if(UL < (PWMNOMINAL-SWING)){UL = PWMNOMINAL-SWING;} // 3,000 to 7,000
    if(UL > (PWMNOMINAL+SWING)){UL = PWMNOMINAL+SWING;}

    //turns left if the center measurement and right measurement is small enough that we will hit the wall if we don't turn
    if((RightDistance<250) && (CenterDistance <250)){
        UL = 0;
        UR = PWMNOMINAL;
    }

    Motor_Forward(UL,UR);

  }
}

volatile uint32_t Time;
uint8_t Bumper;
int Semaphore = 0;
uint16_t max = 9;
char command[5] = "00000";

void bump(){
    Bumper = BumpInt_Read();
    if (Bumper != 0xED){
        Motor_Stop();
        Clock_Delay1ms(1000);
    }

}
/////////////////////////////

/* Application specific status/error codes */
typedef enum{
    DEVICE_NOT_IN_STATION_MODE = -0x7D0,        /* Choosing this number to avoid overlap with host-driver's error codes */
    HTTP_SEND_ERROR = DEVICE_NOT_IN_STATION_MODE - 1,
    HTTP_RECV_ERROR = HTTP_SEND_ERROR - 1,
    HTTP_INVALID_RESPONSE = HTTP_RECV_ERROR -1,
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

#define min(X,Y) ((X) < (Y) ? (X) : (Y))


/*
 * GLOBAL VARIABLES -- Start
 */
/* Button debounce state variables */
volatile unsigned int S1buttonDebounce = 0;
volatile unsigned int S2buttonDebounce = 0;
volatile int publishID = 0;

unsigned char macAddressVal[SL_MAC_ADDR_LEN];
unsigned char macAddressLen = SL_MAC_ADDR_LEN;

char macStr[18];        // Formatted MAC Address String
char uniqueID[9];       // Unique ID generated from TLV RAND NUM and MAC Address

Network n;
Client hMQTTClient;     // MQTT Client

_u32  g_Status = 0;
struct{
    _u8 Recvbuff[MAX_SEND_RCV_SIZE];
    _u8 SendBuff[MAX_SEND_BUF_SIZE];

    _u8 HostName[SMALL_BUF];
    _u8 CityName[SMALL_BUF];

    _u32 DestinationIP;
    _i16 SockID;
}g_AppData;

/* Port mapper configuration register */
const uint8_t port_mapping[] =
{
    //Port P2:
    PM_TA0CCR1A, PM_TA0CCR2A, PM_TA0CCR3A, PM_NONE, PM_TA1CCR1A, PM_NONE, PM_NONE, PM_NONE
};

/* TimerA UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_8,          // SMCLK/8 = 6MHz
        90000,                                  // 15ms debounce period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

/*
 * GLOBAL VARIABLES -- End
 */


/*
 * STATIC FUNCTION DEFINITIONS -- Start
 */
static _i32 establishConnectionWithAP();
static _i32 configureSimpleLinkToDefaultState();
static _i32 initializeAppVariables();
static void displayBanner();
static void messageArrived(MessageData*);
static void generateUniqueID();


/*
 * STATIC FUNCTION DEFINITIONS -- End
 */


/*
 * ASYNCHRONOUS EVENT HANDLERS -- Start
 */

/*!
    \brief This function handles WLAN events

    \param[in]      pWlanEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(pWlanEvent == NULL)
        CLI_Write(" [WLAN EVENT] NULL Pointer Error \n\r");
    
    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);

            /*
             * Information about the connected AP (like name, MAC etc) will be
             * available in 'slWlanConnectAsyncResponse_t' - Applications
             * can use it if required
             *
             * slWlanConnectAsyncResponse_t *pEventData = NULL;
             * pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
             *
             */
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            /* If the user has initiated 'Disconnect' request, 'reason_code' is SL_USER_INITIATED_DISCONNECTION */
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                CLI_Write(" Device disconnected from the AP on application's request \n\r");
            }
            else
            {
                CLI_Write(" Device disconnected from the AP on an ERROR..!! \n\r");
            }
        }
        break;

        default:
        {
            CLI_Write(" [WLAN EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*!
    \brief This function handles events for IP address acquisition via DHCP
           indication

    \param[in]      pNetAppEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(pNetAppEvent == NULL)
        CLI_Write(" [NETAPP EVENT] NULL Pointer Error \n\r");
 
    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            /*
             * Information about the connected AP's IP, gateway, DNS etc
             * will be available in 'SlIpV4AcquiredAsync_t' - Applications
             * can use it if required
             *
             * SlIpV4AcquiredAsync_t *pEventData = NULL;
             * pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
             * <gateway_ip> = pEventData->gateway;
             *
             */
        }
        break;

        default:
        {
            CLI_Write(" [NETAPP EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*!
    \brief This function handles callback for the HTTP server events

    \param[in]      pHttpEvent - Contains the relevant event information
    \param[in]      pHttpResponse - Should be filled by the user with the
                    relevant response information

    \return         None

    \note

    \warning
*/
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    /*
     * This application doesn't work with HTTP server - Hence these
     * events are not handled here
     */
    CLI_Write(" [HTTP EVENT] Unexpected event \n\r");
}

/*!
    \brief This function handles general error events indication

    \param[in]      pDevEvent is the event passed to the handler

    \return         None
*/
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    /*
     * Most of the general errors are not FATAL are are to be handled
     * appropriately by the application
     */
    CLI_Write(" [GENERAL EVENT] \n\r");
}

/*!
    \brief This function handles socket events indication

    \param[in]      pSock is the event passed to the handler

    \return         None
*/
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if(pSock == NULL)
        CLI_Write(" [SOCK EVENT] NULL Pointer Error \n\r");

    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
        {
            /*
            * TX Failed
            *
            * Information about the socket descriptor and status will be
            * available in 'SlSockEventData_t' - Applications can use it if
            * required
            *
            * SlSockEventData_t *pEventData = NULL;
            * pEventData = & pSock->EventData;
            */
            switch( pSock->EventData.status )
            {
                case SL_ECLOSE:
                    CLI_Write(" [SOCK EVENT] Close socket operation failed to transmit all queued packets\n\r");
                break;


                default:
                    CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
                break;
            }
        }
        break;

        default:
            CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
        break;
    }
}
/*
 * ASYNCHRONOUS EVENT HANDLERS -- End
 */


/*
 * Application's entry point
 */
int main(int argc, char** argv)
{

    _i32 retVal = -1;

    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);

    /* Stop WDT and initialize the system-clock of the MCU */
    stopWDT();
    initClk();

//    /* GPIO Setup for Pins 2.0-2.2 */
//    MAP_PMAP_configurePorts((const uint8_t *) port_mapping, PMAP_P2MAP, 1,
//        PMAP_DISABLE_RECONFIGURATION);
//
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
        GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Confinguring P1.1 & P1.4 as an input and enabling interrupts */
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);

    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

//    /* Configure TimerA0 for RGB LED*/
//    TA0CCR0 = PWM_PERIOD;                   // PWM Period
//    TA0CCTL1 = OUTMOD_7;                    // CCR1 reset/set
//    TA0CCR1 = PWM_PERIOD * (0/255);                 // CCR1 PWM duty cycle
//    TA0CCTL2 = OUTMOD_7;                    // CCR2 reset/set
//    TA0CCR2 = PWM_PERIOD * (0/255);                 // CCR2 PWM duty cycle
//    TA0CCTL3 = OUTMOD_7;                    // CCR3 reset/set
//    TA0CCR3 = PWM_PERIOD * (0/255);                 // CCR3 PWM duty cycle
//    TA0CTL = TASSEL__SMCLK | MC__UP | TACLR;  // SMCLK, up mode, clear TAR

    /* Configuring TimerA1 for Up Mode */
    Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig);

    Interrupt_enableInterrupt(INT_TA1_0);
    Interrupt_enableInterrupt(INT_PORT1);
    Interrupt_enableMaster();

    /* Configure command line interface */
    CLI_Configure();

    displayBanner();

    /*
     * Following function configures the device to default state by cleaning
     * the persistent settings stored in NVMEM (viz. connection profiles &
     * policies, power policy etc)
     *
     * Applications may choose to skip this step if the developer is sure
     * that the device is in its default state at start of application
     *
     * Note that all profiles and persistent settings that were done on the
     * device will be lost
     */
    retVal = configureSimpleLinkToDefaultState();
    if(retVal < 0)
    {
        if (DEVICE_NOT_IN_STATION_MODE == retVal)
            CLI_Write(" Failed to configure the device in its default state \n\r");

        LOOP_FOREVER();
    }

    CLI_Write(" Device is configured in default state \n\r");

    /*
     * Assumption is that the device is configured in station mode already
     * and it is in its default state
     */
    retVal = sl_Start(0, 0, 0);
    if ((retVal < 0) ||
        (ROLE_STA != retVal) )
    {
        CLI_Write(" Failed to start the device \n\r");
        LOOP_FOREVER();
    }

    CLI_Write(" Device started as STATION \n\r");

    /* Connecting to WLAN AP */
    retVal = establishConnectionWithAP();
    if(retVal < 0)
    {
        CLI_Write(" Failed to establish connection w/ an AP \n\r");
        LOOP_FOREVER();
    }

    CLI_Write(" Connection established w/ AP and IP is acquired \n\r");

    // Obtain MAC Address
    sl_NetCfgGet(SL_MAC_ADDRESS_GET,NULL,&macAddressLen,(unsigned char *)macAddressVal);

    // Print MAC Addres to be formatted string
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            macAddressVal[0], macAddressVal[1], macAddressVal[2], macAddressVal[3], macAddressVal[4], macAddressVal[5]);

    // Generate 32bit unique ID from TLV Random Number and MAC Address
    generateUniqueID();

    int rc = 0;
    unsigned char buf[100];
    unsigned char readbuf[100];

    NewNetwork(&n);
    rc = ConnectNetwork(&n, MQTT_BROKER_SERVER, 1883);

    if (rc != 0) {
        CLI_Write(" Failed to connect to MQTT broker \n\r");
        LOOP_FOREVER();
    }
    CLI_Write(" Connected to MQTT broker \n\r");

    MQTTClient(&hMQTTClient, &n, 1000, buf, 100, readbuf, 100);
    MQTTPacket_connectData cdata = MQTTPacket_connectData_initializer;
    cdata.MQTTVersion = 3;
    cdata.clientID.cstring = uniqueID;
    rc = MQTTConnect(&hMQTTClient, &cdata);

    if (rc != 0) {
        CLI_Write(" Failed to start MQTT client \n\r");
        LOOP_FOREVER();
    }
    CLI_Write(" Started MQTT client successfully \n\r");

    rc = MQTTSubscribe(&hMQTTClient, SUBSCRIBE_TOPIC, QOS0, messageArrived);

    if (rc != 0) {
        CLI_Write(" Failed to subscribe to /msp/cc3100/demo topic \n\r");
        LOOP_FOREVER();
    }
    CLI_Write(" Subscribed to /msp/cc3100/demo topic \n\r");

    rc = MQTTSubscribe(&hMQTTClient, uniqueID, QOS0, messageArrived);

    if (rc != 0) {
        CLI_Write(" Failed to subscribe to uniqueID topic \n\r");
        LOOP_FOREVER();
    }
    CLI_Write(" Subscribed to uniqueID topic \n\r");

    //Calling the Motor Lib init here
    uint16_t duty;
    Clock_Init48MHz();
//    LaunchPad_Init();   // built-in switches and LEDs
//    Bump_Init();        // bump switches
//    Motor_InitSimple(); // initialization
    Motor_Init(); // initialization
//    Motor_Forward(3000,3000);
//    Bump_Init();
//    Odometry_SetPower(7500,5000);
//    Odometry_Init(0,0,0);


    // busy-wait implementation
      uint32_t channel = 1;
      Clock_Init48MHz();
      Motor_Init();

      //Odometry_Init(0, 0, 0);
      I2CB1_Init(30); // baud rate = 12MHz/30=400kHz

      SysTick->LOAD = 0x00FFFFFF;           // maximum reload value
      SysTick->CTRL = 0x00000005;

      Clear();
      OPT3101_Init();
      OPT3101_Setup();
      OPT3101_CalibrateInternalCrosstalk();
      OPT3101_StartMeasurementChannel(channel);
      OPT3101_ArmInterrupts(&TxChannel, Distances, Amplitudes);

      BumpInt_Init();

      LPF_Init(100,8);
      LPF_Init2(100,8);
      LPF_Init3(100,8);

      StartTime = SysTick->VAL;
      UART0_Init();
      EnableInterrupts();
      Mode = 1;


    while(1){
        rc = MQTTYield(&hMQTTClient, 10);
        if (rc != 0) {
            CLI_Write(" MQTT failed to yield \n\r");
            LOOP_FOREVER();
        }

//        UpdatePosition();
//        ForwardUntilXStart(1);

        while(ON == 0){
              hope = UART0_InChar();
              if(hope == 'd'){ON = 1;}
          }

          while(ON == 1){
          TimeToConvert = ((StartTime-SysTick->VAL)&0x00FFFFFF)/48000;
          if(TxChannel <= 2)
          {
              if(TxChannel==0){
                      if(Amplitudes[0] > 1000){
                        LeftDistance = FilteredDistances[0] = Left(LPF_Calc(Distances[0]));
                      }else{
                        LeftDistance = FilteredDistances[0] = 500;
                      }

              }else if(TxChannel==1){
                      if(Amplitudes[1] > 1000){
                        CenterDistance = FilteredDistances[1] = LPF_Calc2(Distances[1]);
                      }else{
                        CenterDistance = FilteredDistances[1] = 500;
                      }
              }else {
                      if(Amplitudes[2] > 1000){
                        RightDistance = FilteredDistances[2] = Right(LPF_Calc3(Distances[2]));
                      }else{
                        RightDistance = FilteredDistances[2] = 500;
                      }
                    }

              TxChannel = 3;
              channel = (channel+1)%3;
              OPT3101_StartMeasurementChannel(channel);
              StartTime = SysTick->VAL;
              i=i+1;
              }
              Controller_Right();

                  if(i >= 100){
                    i = 0;
              //      SetCursor(3, 5);
              //      OutUDec(SetPoint);
              //      SetCursor(3, 6);
              //      OutSDec(Error);
              //      SetCursor(3, 7);
              //      OutUDec(UL); OutChar(','); OutUDec(UR);
                  }

              bump();

              hope = ((char)(EUSCI_A0->RXBUF));
              if(hope == 's'){ON = 0;}
              //WaitForInterrupt();
          }



//        //TODO: Copy this for sending MQTT message - This is the MQTT Send Handler
//        if (publishID) {
//            int rc = 0;
//            MQTTMessage msg;
//            msg.dup = 0;
//            msg.id = 0;
//            msg.payload = uniqueID;
//            msg.payloadlen = 8;
//            msg.qos = QOS0;
//            msg.retained = 0;
//            rc = MQTTPublish(&hMQTTClient, PUBLISH_TOPIC, &msg);
//
//            if (rc != 0) {
//                CLI_Write(" Failed to publish unique ID to MQTT broker \n\r");
//                LOOP_FOREVER();
//            }
//            CLI_Write(" Published unique ID successfully \n\r");
//
//            publishID = 0;
//        }
//        Delay(10);

        if(Bump_Read()){
            int rc = 0;
            MQTTMessage msg;
            msg.dup = 0;
            msg.id = 0;
            msg.payload = "Bumped";
            msg.payloadlen = 6;
            msg.qos = QOS0;
            msg.retained = 0;
            rc = MQTTPublish(&hMQTTClient, "arf/bump", &msg);
        }else{
            int rc = 0;
            MQTTMessage msg;
            msg.dup = 0;
            msg.id = 0;
            msg.payload = " ";
            msg.payloadlen = 1;
            msg.qos = QOS0;
            msg.retained = 0;
            rc = MQTTPublish(&hMQTTClient, "arf/bump", &msg);
        }
        Delay(10);

        //Send RPM
        int rpmL = 100, rpmR=100;
//        sendMQTTMessageInt(rpmL, "arf/rpmL", 5);
//        sendMQTTMessage(rpmR, "arf/rpmR", 5);
        char value[10];
        ltoa(rpmL, value, 10);
        sendMQTTMessage(value, "arf/rpmL", sizeof(rpmL)-1);
        ltoa(rpmR, value, 10);
        sendMQTTMessage(value, "arf/rpmR", sizeof(rpmR)-1);

        //send distance to object
        ltoa(Odometry_GetX(), value, 10);
        sendMQTTMessage(value, "arf/distL", sizeof(rpmL)-1);
        ltoa(rpmL, value, 10);
        sendMQTTMessage(rpmR, "arf/distC", sizeof(rpmL)-1);
        ltoa(rpmL, value, 10);
        sendMQTTMessage(rpmR, "arf/distR", sizeof(rpmL)-1);

    }
}

char message[10];
void sendMQTTMessage(char *message[], char *topic[], int length){
    int rc = 0;
    MQTTMessage msg;
    msg.dup = 0;
    msg.id = 0;
    msg.payload = message;
    msg.payloadlen = length;
    msg.qos = QOS0;
    msg.retained = 0;
    rc = MQTTPublish(&hMQTTClient, topic, &msg);
    CLI_Write(rc);
    Delay(10);
}

void sendMQTTMessageInt(int message, char *topic[], int length){
    int rc = 0;
    MQTTMessage msg;
    msg.dup = 0;
    msg.id = 0;
    msg.payload = message;
    msg.payloadlen = length;
    msg.qos = QOS0;
    msg.retained = 0;
    rc = MQTTPublish(&hMQTTClient, topic, &msg);
    Delay(10);
}

static void generateUniqueID() {
    CRC32_setSeed(TLV->RANDOM_NUM_1, CRC32_MODE);
    CRC32_set32BitData(TLV->RANDOM_NUM_2);
    CRC32_set32BitData(TLV->RANDOM_NUM_3);
    CRC32_set32BitData(TLV->RANDOM_NUM_4);
    int i;
    for (i = 0; i < 6; i++)
    CRC32_set8BitData(macAddressVal[i], CRC32_MODE);

    uint32_t crcResult = CRC32_getResult(CRC32_MODE);
    sprintf(uniqueID, "%06X", crcResult);
}

//****************************************************************************
//
//!    \brief MQTT message received callback - Called when a subscribed topic
//!                                            receives a message.
//! \param[in]                  data is the data passed to the callback
//!
//! \return                        None
//
//****************************************************************************
static void messageArrived(MessageData* data) {
    char buf[BUFF_SIZE];

    char *tok;
    long color;

    // Check for buffer overflow
    if (data->topicName->lenstring.len >= BUFF_SIZE) {
//      UART_PRINT("Topic name too long!\n\r");
        return;
    }
    if (data->message->payloadlen >= BUFF_SIZE) {
//      UART_PRINT("Payload too long!\n\r");
        return;
    }

    strncpy(buf, data->topicName->lenstring.data,
        min(BUFF_SIZE, data->topicName->lenstring.len));
    buf[data->topicName->lenstring.len] = 0;



    strncpy(buf, data->message->payload,
        min(BUFF_SIZE, data->message->payloadlen));
    buf[data->message->payloadlen] = 0;

//    tok = strtok(buf, " ");
//    color = strtol(tok, NULL, 10);
//    TA0CCR1 = PWM_PERIOD * (color/255.0);                 // CCR1 PWM duty cycle
//    tok = strtok(NULL, " ");
//    color = strtol(tok, NULL, 10);
//    TA0CCR2 = PWM_PERIOD * (color/255.0);                // CCR2 PWM duty cycle
//    tok = strtok(NULL, " ");
//    color = strtol(tok, NULL, 10);
//    TA0CCR3 = PWM_PERIOD * (color/255.0);                  // CCR3 PWM duty cycle
//
//TODO: Implement the handle algorithm - This is MQTT Handler

    CLI_Write(buf);
    if(!strcmp(buf, "go")){
        Motor_Forward(5000,5000);
        CLI_Write("We Go");

        int rc;
        MQTTMessage msg;
        msg.dup = 0;
        msg.id = 0;
        msg.payload = "5000";
        msg.payloadlen = 8;
        msg.qos = QOS0;
        msg.retained = 0;
        rc = MQTTPublish(&hMQTTClient, "arf/leftduty", &msg);
        rc = MQTTPublish(&hMQTTClient, "arf/rightduty", &msg);
    }else if(!strcmp(buf, "stop")){
        Motor_Stop();
        CLI_Write("We Stop");

        int rc;
        MQTTMessage msg;
        msg.dup = 0;
        msg.id = 0;
        msg.payload = "0";
        msg.payloadlen = 8;
        msg.qos = QOS0;
        msg.retained = 0;
        rc = MQTTPublish(&hMQTTClient, "arf/leftduty", &msg);
        rc = MQTTPublish(&hMQTTClient, "arf/rightduty", &msg);
    }else if(!strcmp(buf, "right")){
        Motor_Forward(5000,2500);
    }else if(!strcmp(buf, "left")){
        Motor_Forward(2500,5000);
    }


    return;
}

/*
 * Port 1 interrupt handler. This handler is called whenever the switch attached
 * to P1.1 is pressed.
 */
void PORT1_IRQHandler(void)
{
    uint32_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    if (status & GPIO_PIN1)
    {
        if (S1buttonDebounce == 0)
        {
            S1buttonDebounce = 1;

            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

            // Publish the unique ID
            publishID = 1;

            MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
        }
    }
    if (status & GPIO_PIN4)
    {
        if (S2buttonDebounce == 0)
        {
            S2buttonDebounce = 1;

            CLI_Write(" MAC Address: \n\r ");
            CLI_Write(macStr);
            CLI_Write("\n\r");

            MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
        }
    }
}

void TA1_0_IRQHandler(void)
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    if (P1IN & GPIO_PIN1)
    {
        S1buttonDebounce = 0;
    }
    if (P1IN & GPIO_PIN4)
    {
        S2buttonDebounce = 0;
    }

    if ((P1IN & GPIO_PIN1) && (P1IN & GPIO_PIN4))
    {
        Timer_A_stopTimer(TIMER_A1_BASE);
    }
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
                TIMER_A_CAPTURECOMPARE_REGISTER_0);
}


/*!
    \brief This function configure the SimpleLink device in its default state. It:
           - Sets the mode to STATION
           - Configures connection policy to Auto and AutoSmartConfig
           - Deletes all the stored profiles
           - Enables DHCP
           - Disables Scan policy
           - Sets Tx power to maximum
           - Sets power policy to normal
           - Unregisters mDNS services
           - Remove all filters

    \param[in]      none

    \return         On success, zero is returned. On error, negative is returned
*/
static _i32 configureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    _u8           val = 1;
    _u8           configOpt = 0;
    _u8           configLen = 0;
    _u8           power = 0;

    _i32          retVal = -1;
    _i32          mode = -1;

    mode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(mode);

    /* If the device is not in station-mode, try configuring it in station-mode */
    if (ROLE_STA != mode)
    {
        if (ROLE_AP == mode)
        {
            /* If the device is in AP mode, we need to wait for this event before doing anything */
            while(!IS_IP_ACQUIRED(g_Status)) { _SlNonOsMainLoopTask(); }
        }

        /* Switch to STA role and restart */
        retVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Stop(SL_STOP_TIMEOUT);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(retVal);

        /* Check if the device is in station again */
        if (ROLE_STA != retVal)
        {
            /* We don't want to proceed if the device is not coming up in station-mode */
            ASSERT_ON_ERROR(DEVICE_NOT_IN_STATION_MODE);
        }
    }

    /* Get the device's version-information */
    configOpt = SL_DEVICE_GENERAL_VERSION;
    configLen = sizeof(ver);
    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (_u8 *)(&ver));
    ASSERT_ON_ERROR(retVal);

    /* Set connection policy to Auto + SmartConfig (Device's default connection policy) */
    retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove all profiles */
    retVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(retVal);

    /*
     * Device in station-mode. Disconnect previous connection if any
     * The function returns 0 if 'Disconnected done', negative number if already disconnected
     * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
     */
    retVal = sl_WlanDisconnect();
    if(0 == retVal)
    {
        /* Wait */
        while(IS_CONNECTED(g_Status)) { _SlNonOsMainLoopTask(); }
    }

    /* Enable DHCP client*/
    retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&val);
    ASSERT_ON_ERROR(retVal);

    /* Disable scan */
    configOpt = SL_SCAN_POLICY(0);
    retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Set Tx power level for station mode
       Number between 0-15, as dB offset from max power - 0 will set maximum power */
    power = 0;
    retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (_u8 *)&power);
    ASSERT_ON_ERROR(retVal);

    /* Set PM policy to normal */
    retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Unregister mDNS services */
    retVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove  all 64 filters (8*8) */
    pal_Memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    retVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(retVal);

    retVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(retVal);

    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);

    return retVal; /* Success */
}

/*!
    \brief Connecting to a WLAN Access point

    This function connects to the required AP (SSID_NAME).
    The function will return once we are connected and have acquired IP address

    \param[in]  None

    \return     0 on success, negative error-code on error

    \note

    \warning    If the WLAN connection fails or we don't acquire an IP address,
                We will be stuck in this function forever.
*/
static _i32 establishConnectionWithAP()
{
    SlSecParams_t secParams = {0};
    _i32 retVal = 0;

    secParams.Key = PASSKEY;
    secParams.KeyLen = PASSKEY_LEN;
    secParams.Type = SEC_TYPE;

    retVal = sl_WlanConnect(SSID_NAME, pal_Strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(retVal);

    /* Wait */
    while((!IS_CONNECTED(g_Status)) || (!IS_IP_ACQUIRED(g_Status))) { _SlNonOsMainLoopTask(); }

    return SUCCESS;
}

/*!
    \brief This function initializes the application variables

    \param[in]  None

    \return     0 on success, negative error-code on error
*/
static _i32 initializeAppVariables()
{
    g_Status = 0;
    pal_Memset(&g_AppData, 0, sizeof(g_AppData));

    return SUCCESS;
}

/*!
    \brief This function displays the application's banner

    \param      None

    \return     None
*/
static void displayBanner()
{
    CLI_Write("\n\r\n\r");
    CLI_Write(" MQTT Twitter Controlled RGB LED - Version ");
    CLI_Write(APPLICATION_VERSION);
    CLI_Write("\n\r*******************************************************************************\n\r");
}
