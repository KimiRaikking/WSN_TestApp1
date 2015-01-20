/******************************************************************************
  Filename:       WSN_TestApp1.c
  Revised:        $Date: 2012-03-07 01:04:58 -0800 (Wed, 07 Mar 2012) $
  Revision:       $Revision: 29656 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
******************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends "Hello World" to another "Generic"
  application every 5 seconds.  The application will also
  receives "Hello World" packets.

  The "Hello World" messages are sent/received as MSG type message.

  This applications doesn't have a profile, so it handles everything
  directly - itself.

  Key control:
    SW1:
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "WSN_TestApp1.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
#include "MT_UART.h"

#include "NLMEDE.h"

/* RTOS */
#if defined( IAR_ARMCM3_LM )
#include "RTOS_App.h"
#endif  

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 zgprofid;
NodeAddr	MyNodeAddr;
uint8	NwkAddrFrame[16];		//2bytes:cmd type;2bytes:device type;2bytes:short addr;2bytes:parent addr;8bytes:ext addr
DeviceEndPoint* pHeadPoint;
uint16 NodeShortAddr ;
bool IsNodeInList ;
// This list should be filled with Application specific Cluster IDs.
const cId_t WSN_TestApp1_ClusterList[WSN_TestApp1_MAX_CLUSTERS] =
{
  WSN_TestApp1_CLUSTERID,
  WSN_TestApp1_NWKCMD_CLUSTERID
 
};

const SimpleDescriptionFormat_t WSN_TestApp1_SimpleDesc =
{
  WSN_TestApp1_ENDPOINT,              //  int Endpoint;
  WSN_TestApp1_PROFID,                //  uint16 AppProfId[2];
  WSN_TestApp1_DEVICEID,              //  uint16 AppDeviceId[2];
  WSN_TestApp1_DEVICE_VERSION,        //  int   AppDevVer:4;
  WSN_TestApp1_FLAGS,                 //  int   AppFlags:4;
  WSN_TestApp1_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)WSN_TestApp1_ClusterList,  //  byte *pAppInClusterList;
  WSN_TestApp1_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)WSN_TestApp1_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in WSN_TestApp1_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t WSN_TestApp1_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte WSN_TestApp1_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // WSN_TestApp1_Init() is called.
devStates_t WSN_TestApp1_NwkState;


byte WSN_TestApp1_TransID;  // This is the unique message ID (counter)

afAddrType_t WSN_TestApp1_DstAddr;
afAddrType_t WSN_TestApp1_Unicast_DstAddr;			//this is the unicast destination address of coord 
afAddrType_t WSN_TestApp1_Broadcast_DstAddr;		//this is the broadcast destination address for test

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void WSN_TestApp1_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void WSN_TestApp1_HandleKeys( byte shift, byte keys );
static void WSN_TestApp1_MessageMSGCB( afIncomingMSGPacket_t *pckt );
static void WSN_TestApp1_SendTheMessage( void );
static void WSN_TestApp1_ProcessNwkCmdMSG(byte* pNwkAddrFrame);		//process incoming network cmd msg

#if defined( IAR_ARMCM3_LM )
static void WSN_TestApp1_ProcessRtosMessage( void );
#endif

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      WSN_TestApp1_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void WSN_TestApp1_Init( uint8 task_id )
{
  WSN_TestApp1_TaskID = task_id;
  WSN_TestApp1_NwkState = DEV_INIT;
  WSN_TestApp1_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  //add uart function 
  MT_UartInit();
  MT_UartRegisterTaskID(task_id);
  //HalUARTWrite(0,"HELLO\n",6);

  WSN_TestApp1_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  WSN_TestApp1_DstAddr.endPoint = WSN_TestApp1_ENDPOINT;
  WSN_TestApp1_DstAddr.addr.shortAddr = 0x0000;

  //config unicast destination address  to coordinator short addr
  WSN_TestApp1_Unicast_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  WSN_TestApp1_Unicast_DstAddr.endPoint = WSN_TestApp1_ENDPOINT;
  WSN_TestApp1_Unicast_DstAddr.addr.shortAddr = 0x0000;

  //config broadcast destination address 
  WSN_TestApp1_Broadcast_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  WSN_TestApp1_Broadcast_DstAddr.endPoint = WSN_TestApp1_ENDPOINT;
  WSN_TestApp1_Broadcast_DstAddr.addr.shortAddr = 0xFFFF;
  
  // Fill out the endpoint description.
  WSN_TestApp1_epDesc.endPoint = WSN_TestApp1_ENDPOINT;
  WSN_TestApp1_epDesc.task_id = &WSN_TestApp1_TaskID;
  WSN_TestApp1_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&WSN_TestApp1_SimpleDesc;
  WSN_TestApp1_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &WSN_TestApp1_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( WSN_TestApp1_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "WSN_TestApp1", HAL_LCD_LINE_1 );
#endif

  ZDO_RegisterForZDOMsg( WSN_TestApp1_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( WSN_TestApp1_TaskID, Match_Desc_rsp );
  ZDO_RegisterForZDOMsg(WSN_TestApp1_TaskID,IEEE_addr_rsp);			//register IEEE_addr_rsp  ZDO msg by kimi
  ZDO_RegisterForZDOMsg(WSN_TestApp1_TaskID,Device_annce);			//register Device_annce ZDO msg by kimi

#if defined( IAR_ARMCM3_LM )
  // Register this task with RTOS task initiator
  RTOS_RegisterApp( task_id, WSN_TestApp1_RTOS_MSG_EVT );
#endif
}

/*********************************************************************
 * @fn      WSN_TestApp1_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 WSN_TestApp1_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( WSN_TestApp1_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          WSN_TestApp1_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          WSN_TestApp1_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
          }
          break;

        case AF_INCOMING_MSG_CMD:
          WSN_TestApp1_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
		/*	close owner defined network topology finding msg	
	  	zgprofid = zgStackProfile;
		if(zgprofid == 0x02)
			HalLedBlink(HAL_LED_1,3,50,1000);			//to judge if the stack profile is Zigbee Pro stack profile
          WSN_TestApp1_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if (  (WSN_TestApp1_NwkState == DEV_ZB_COORD) 
		  		||(WSN_TestApp1_NwkState == DEV_ROUTER)
              	|| (WSN_TestApp1_NwkState == DEV_END_DEVICE) )		
              
          {
				MyNodeAddr.MyShortAddr = NLME_GetShortAddr();
				byte* pMyExtAddr = NLME_GetExtAddr();
				for(uint8 i=0;i<8;i++)
					MyNodeAddr.MyExtAddr[i] = *(pMyExtAddr+i);
				switch(WSN_TestApp1_NwkState)
				{	case DEV_ZB_COORD:
						MyNodeAddr.MyDeviceType = Node_Coord_Type;
						break;
					case DEV_ROUTER:
						MyNodeAddr.MyDeviceType = Node_Router_Type;
						break;
					case DEV_END_DEVICE:
						MyNodeAddr.MyDeviceType = Node_EndDevice_Type;
						break;
					default:
						break;

				}
				MyNodeAddr.ParentShortAddr = NLME_GetCoordShortAddr();
				if(MyNodeAddr.MyDeviceType == Node_Coord_Type)
				{
					pHeadPoint = (DeviceEndPoint*)osal_mem_alloc(sizeof(DeviceEndPoint));
					pHeadPoint->DeviceNodeAddr = MyNodeAddr;
					pHeadPoint->pNextNode = (DeviceEndPoint*)NULL;				//store coord node addr info

					
					
				}
					
				if(WSN_TestApp1_NwkState != DEV_ZB_COORD)
				{	
					
				//build network addr frame for router and end device
				NwkAddrFrame[0] = 0x00;
				NwkAddrFrame[1] = 0x00;		//CMD TYPE
				NwkAddrFrame[2] = 0x00;
				NwkAddrFrame[3] = MyNodeAddr.MyDeviceType;		//device type
				NwkAddrFrame[4] = (MyNodeAddr.MyShortAddr & 0xff00)>>8;		//higher byte of short addr
				NwkAddrFrame[5] = MyNodeAddr.MyShortAddr & 0x00ff;		//my short address,lower byte of short addr
				NwkAddrFrame[6] = (MyNodeAddr.ParentShortAddr & 0xff00)>>8;
				NwkAddrFrame[7] = MyNodeAddr.ParentShortAddr & 0x00ff;		//parent short address
				
				for(uint8 i=0;i<8;i++)
					NwkAddrFrame[8+i] = MyNodeAddr.MyExtAddr[i];			//my ext address,it is dangerous of overflow for array
				if(AF_DataRequest(&WSN_TestApp1_Unicast_DstAddr,&WSN_TestApp1_epDesc,
								WSN_TestApp1_NWKCMD_CLUSTERID,
								16,
								(byte*)&NwkAddrFrame,
								&WSN_TestApp1_TransID,
								AF_DISCV_ROUTE,AF_DEFAULT_RADIUS) == afStatus_SUCCESS)
				{
					HalLedBlink(HAL_LED_2,3,50,500);
				}

				
				
				}
			// Start sending "the" message in a regular interval.
			
            osal_start_timerEx( WSN_TestApp1_TaskID,
                                WSN_TestApp1_SEND_MSG_EVT,
                                WSN_TestApp1_SEND_MSG_TIMEOUT );
                                
          }
		*/
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( WSN_TestApp1_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in WSN_TestApp1_Init()).
  if ( events & WSN_TestApp1_SEND_MSG_EVT )
  {
    // Send "the" message
    WSN_TestApp1_SendTheMessage();

    // Setup to send message again
    osal_start_timerEx( WSN_TestApp1_TaskID,
                        WSN_TestApp1_SEND_MSG_EVT,
                        WSN_TestApp1_SEND_MSG_TIMEOUT );

    // return unprocessed events
    return (events ^ WSN_TestApp1_SEND_MSG_EVT);
  }

  //owner defined event
  if(events & WSN_TestApp1_SEND_NWK_CMD_EVT)
  {
		if(WSN_TestApp1_NwkState != DEV_ZB_COORD)
		{
			
			WSN_TestApp1_SendTheMessage();
		}
		return (events^WSN_TestApp1_SEND_NWK_CMD_EVT);
  }

  
#if defined( IAR_ARMCM3_LM )
  // Receive a message from the RTOS queue
  if ( events & WSN_TestApp1_RTOS_MSG_EVT )
  {
    // Process message from RTOS queue
    WSN_TestApp1_ProcessRtosMessage();

    // return unprocessed events
    return (events ^ WSN_TestApp1_RTOS_MSG_EVT);
  }
#endif

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      WSN_TestApp1_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
static void WSN_TestApp1_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined( BLINK_LEDS )
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            WSN_TestApp1_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            WSN_TestApp1_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            WSN_TestApp1_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;

	  case IEEE_addr_rsp:
	  	{
			ZDO_NwkIEEEAddrResp_t* pRsp = ZDO_ParseAddrRsp( inMsg);
			if(pRsp)
			{	
				
				
				HalUARTWrite(0,(uint8*)pRsp,sizeof(*pRsp)+2*pRsp->numAssocDevs);	//note the length
				MicroWait(5000);			//in case uart frame superposition
				for(uint8 i =0;i<pRsp->numAssocDevs;i++)
					{
					//request to the associated devs if not null of the list
					
					uint16 DevShortAddr = pRsp->devList[i];
					if(ZDP_IEEEAddrReq(DevShortAddr,ZDP_ADDR_REQTYPE_EXTENDED,0,0)== afStatus_SUCCESS);
						//HalLedBlink(HAL_LED_2,1,50,1000);
					}
			}
			
			osal_mem_free(pRsp);		//must free ZDO_NwkIEEEAddrResp_t* data whichi allocated in ZDO_ParseAddrRsp
			
	  }
	  break;

	  case Device_annce:
	  	{
			ZDO_DeviceAnnce_t* pAnnce;
			ZDO_ParseDeviceAnnce(inMsg,pAnnce);
			//when new dev joining network,call IEEE_addr_rsp request
			if(ZDP_IEEEAddrReq(0x0000,ZDP_ADDR_REQTYPE_EXTENDED,0,0)== afStatus_SUCCESS)
				HalLedBlink(HAL_LED_2,2,50,500);
	  }
	  break;
  }
}

/*********************************************************************
 * @fn      WSN_TestApp1_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void WSN_TestApp1_HandleKeys( uint8 shift, uint8 keys )
{
  zAddrType_t dstAddr;

  // Shift is used to make each button/switch dual purpose.
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
      // Since SW1 isn't used for anything else in this application...
#if defined( SWITCH1_BIND )
      // we can use SW1 to simulate SW2 for devices that only have one switch,
      keys |= HAL_KEY_SW_2;
#elif defined( SWITCH1_MATCH )
      // or use SW1 to simulate SW4 for devices that only have one switch
      keys |= HAL_KEY_SW_4;
#endif
    }

    if ( keys & HAL_KEY_SW_2 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

      // Initiate an End Device Bind Request for the mandatory endpoint
      dstAddr.addrMode = Addr16Bit;
      dstAddr.addr.shortAddr = 0x0000; // Coordinator
      ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                            WSN_TestApp1_epDesc.endPoint,
                            WSN_TestApp1_PROFID,
                            WSN_TestApp1_MAX_CLUSTERS, (cId_t *)WSN_TestApp1_ClusterList,
                            WSN_TestApp1_MAX_CLUSTERS, (cId_t *)WSN_TestApp1_ClusterList,
                            FALSE );
    }

    if ( keys & HAL_KEY_SW_3 )
    {
    }

    if ( keys & HAL_KEY_SW_4 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
      // Initiate a Match Description Request (Service Discovery)
      dstAddr.addrMode = AddrBroadcast;
      dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
      ZDP_MatchDescReq( &dstAddr, NWK_BROADCAST_SHORTADDR,
                        WSN_TestApp1_PROFID,
                        WSN_TestApp1_MAX_CLUSTERS, (cId_t *)WSN_TestApp1_ClusterList,
                        WSN_TestApp1_MAX_CLUSTERS, (cId_t *)WSN_TestApp1_ClusterList,
                        FALSE );
    }
	if(keys & HAL_KEY_SW_1)		//button s1
	{
		if(ZDP_IEEEAddrReq(0x0000,ZDP_ADDR_REQTYPE_EXTENDED,0,0)== afStatus_SUCCESS)
			HalLedBlink(HAL_LED_2,2,50,1000);
		
	}
  }

  

  
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      WSN_TestApp1_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
static void WSN_TestApp1_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  
  switch ( pkt->clusterId )
  {
	
	case WSN_TestApp1_CLUSTERID:
      // "the" message
     // HalUARTWrite(0,pkt->cmd.Data,12);
	  
#if defined( LCD_SUPPORTED )
      HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
	case WSN_TestApp1_NWKCMD_CLUSTERID:
		//close this cluster id
		//HalUARTWrite(0,pkt->cmd.Data,16);
		//ZDP_IEEEAddrReq(0x0000,ZDP_ADDR_REQTYPE_EXTENDED,0,0);
		//WSN_TestApp1_ProcessNwkCmdMSG(pkt->cmd.Data);		//process the incoming af msg about network node address
		//HalLedBlink(HAL_LED_1,5,50,1000);
		break;
	
  }
}

/*********************************************************************
 * @fn      WSN_TestApp1_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
static void WSN_TestApp1_SendTheMessage( void )
{
  //char theMessageData[] = "Hello World";

  if ( AF_DataRequest( &WSN_TestApp1_Unicast_DstAddr, &WSN_TestApp1_epDesc,
                       WSN_TestApp1_NWKCMD_CLUSTERID,
                       16,
                       (byte *)&NwkAddrFrame,
                       &WSN_TestApp1_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
    HalLedBlink(HAL_LED_2,3,50,1000);
  }
  else
  {
    // Error occurred in request to send.
  }
}

#if defined( IAR_ARMCM3_LM )
/*********************************************************************
 * @fn      WSN_TestApp1_ProcessRtosMessage
 *
 * @brief   Receive message from RTOS queue, send response back.
 *
 * @param   none
 *
 * @return  none
 */
static void WSN_TestApp1_ProcessRtosMessage( void )
{
  osalQueue_t inMsg;

  if ( osal_queue_receive( OsalQueue, &inMsg, 0 ) == pdPASS )
  {
    uint8 cmndId = inMsg.cmnd;
    uint32 counter = osal_build_uint32( inMsg.cbuf, 4 );

    switch ( cmndId )
    {
      case CMD_INCR:
        counter += 1;  /* Increment the incoming counter */
                       /* Intentionally fall through next case */

      case CMD_ECHO:
      {
        userQueue_t outMsg;

        outMsg.resp = RSP_CODE | cmndId;  /* Response ID */
        osal_buffer_uint32( outMsg.rbuf, counter );    /* Increment counter */
        osal_queue_send( UserQueue1, &outMsg, 0 );  /* Send back to UserTask */
        break;
      }
      
      default:
        break;  /* Ignore unknown command */    
    }
  }
}
#endif

/*********************************************************************
* @fn 	static void WSN_TestApp1_ProcessNwkCmdMSG
*
* @brief	process incoming network addr cmd msg
*
* @param	pNwkAddrFrame: incoming af NwkAddrFrame
*
* return	none
 */
 static void WSN_TestApp1_ProcessNwkCmdMSG(byte* pNwkAddrFrame)
{
	
	//first judge if the head is null,if true,do nothing
	if(pHeadPoint != NULL)
	{
		//judge if the node is already in the list
		DeviceEndPoint* pTempPoint1;
		DeviceEndPoint* pTempPoint2;
		pTempPoint1 = (DeviceEndPoint*)osal_mem_alloc(sizeof(DeviceEndPoint));
		pTempPoint2 = (DeviceEndPoint*)osal_mem_alloc(sizeof(DeviceEndPoint));
		//osal_memcpy(pTempPoint2,pHeadPoint,sizeof(DeviceEndPoint));
		pTempPoint2 = pHeadPoint;
		IsNodeInList = false;
		while(pTempPoint2)		//search if the point is in the list
		{
			//pTempPoint1 = pTempPoint2;
			//osal_memcpy(pTempPoint1,pTempPoint2,sizeof(DeviceEndPoint));
			pTempPoint1 = pTempPoint2;
			if(osal_memcmp(pTempPoint1->DeviceNodeAddr.MyExtAddr,(pNwkAddrFrame+8),8))		//compare extend address not short address
			{
				IsNodeInList = true;
				break;
			}
			pTempPoint2 = pTempPoint2->pNextNode;
			
		}
		//process the incoming node
		if(IsNodeInList)
		{
			//if true ,update the existing node
			pTempPoint1->DeviceNodeAddr.MyDeviceType = *(pNwkAddrFrame+3);
			pTempPoint1->DeviceNodeAddr.MyShortAddr = *(pNwkAddrFrame+4);
			pTempPoint1->DeviceNodeAddr.MyShortAddr = pTempPoint1->DeviceNodeAddr.MyShortAddr<< 8;
			pTempPoint1->DeviceNodeAddr.MyShortAddr += *(pNwkAddrFrame+5);
			pTempPoint1->DeviceNodeAddr.ParentShortAddr = *(pNwkAddrFrame+6);
			pTempPoint1->DeviceNodeAddr.ParentShortAddr = pTempPoint1->DeviceNodeAddr.ParentShortAddr<< 8;
			pTempPoint1->DeviceNodeAddr.ParentShortAddr += *(pNwkAddrFrame+7);
			for(uint8 j=0;j<8;j++)
				pTempPoint1->DeviceNodeAddr.MyExtAddr[j] = *(pNwkAddrFrame+8+j);
			
		}
		else
		{
			//if false,add new node in the tail of the list,and pTempPoint1 point to the tail of the list
			DeviceEndPoint* pNewPoint1;
			pNewPoint1 = (DeviceEndPoint*)osal_mem_alloc(sizeof(DeviceEndPoint));
			osal_memset(pNewPoint1,0,sizeof(DeviceEndPoint));
			pNewPoint1->DeviceNodeAddr.MyDeviceType = *(pNwkAddrFrame+3);
			pNewPoint1->DeviceNodeAddr.MyShortAddr = *(pNwkAddrFrame+4);
			pNewPoint1->DeviceNodeAddr.MyShortAddr = pNewPoint1->DeviceNodeAddr.MyShortAddr<< 8;
			pNewPoint1->DeviceNodeAddr.MyShortAddr += *(pNwkAddrFrame+5);
			pNewPoint1->DeviceNodeAddr.ParentShortAddr = *(pNwkAddrFrame+6);
			pNewPoint1->DeviceNodeAddr.ParentShortAddr = pNewPoint1->DeviceNodeAddr.ParentShortAddr<< 8;
			pNewPoint1->DeviceNodeAddr.ParentShortAddr += *(pNwkAddrFrame+7);
			for(uint8 k=0;k<8;k++)
				pNewPoint1->DeviceNodeAddr.MyExtAddr[k] = (uint8)*(pNwkAddrFrame+8+k);
			pTempPoint1->pNextNode = pNewPoint1;
			pNewPoint1->pNextNode = NULL;
			
		}
		//free mem allocation
		//osal_mem_free(pTempPoint1);
		//osal_mem_free(pTempPoint2);
		
		//output the list
		DeviceEndPoint* pTempPoint;
		pTempPoint = (DeviceEndPoint*)osal_mem_alloc(sizeof(DeviceEndPoint));
		//osal_memcpy(pTempPoint,pHeadPoint,sizeof(DeviceEndPoint));
		pTempPoint = pHeadPoint;
		//pTempPoint = pHeadPoint;
		while(pTempPoint)
		{
			byte NodeInfo[14];
			NodeInfo[0] = 0x00;
			NodeInfo[1] = pTempPoint->DeviceNodeAddr.MyDeviceType;
			NodeInfo[2] = (pTempPoint->DeviceNodeAddr.MyShortAddr & 0xff00)>>8;
			NodeInfo[3] = pTempPoint->DeviceNodeAddr.MyShortAddr & 0x00ff;
			NodeInfo[4] = (pTempPoint->DeviceNodeAddr.ParentShortAddr & 0xff00)>>8;
			NodeInfo[5] = pTempPoint->DeviceNodeAddr.ParentShortAddr & 0x00ff;
			for(uint8 i=0;i<8;i++)
				NodeInfo[6+i] = pTempPoint->DeviceNodeAddr.MyExtAddr[i];
			HalUARTWrite(0,(uint8*)NodeInfo,14);
			pTempPoint = pTempPoint->pNextNode;
		}
		//osal_mem_free(pTempPoint);
	}
	
	
}