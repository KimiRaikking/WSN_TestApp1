/**************************************************************************************************
  Filename:       WSN_TestApp1.h
  Revised:        $Date: 2012-02-12 15:58:41 -0800 (Sun, 12 Feb 2012) $
  Revision:       $Revision: 29216 $

  Description:    This file contains the Generic Application definitions.


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
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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
**************************************************************************************************/

#ifndef WSN_TestApp1_H
#define WSN_TestApp1_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"

/*********************************************************************
 * CONSTANTS
 */

// These constants are only for example and should be changed to the
// device's needs
#define WSN_TestApp1_ENDPOINT           10

#define WSN_TestApp1_PROFID             0x0F04
#define WSN_TestApp1_DEVICEID           0x0001
#define WSN_TestApp1_DEVICE_VERSION     0
#define WSN_TestApp1_FLAGS              0

#define WSN_TestApp1_MAX_CLUSTERS       2
#define WSN_TestApp1_CLUSTERID          1
#define	WSN_TestApp1_NWKCMD_CLUSTERID	2		//CLUSTER ID FOR NETWORK CMD

#define WSN_NETWORK_CMD_FRAME_LENGTH	16		//16 BYTES FOR WSN NWK CMD FRAME LENGTH

// Send Message Timeout
#define WSN_TestApp1_SEND_MSG_TIMEOUT   5000     // Every 5 seconds
#define WSN_TestApp1_SEND_NWK_CMD_TIMEOUT	5000		//EVERY 1 SECOND
// Application Events (OSAL) - These are bit weighted definitions.
#define WSN_TestApp1_SEND_MSG_EVT       0x0001
#define WSN_TestApp1_SEND_NWK_CMD_EVT	0x0002

#if defined( IAR_ARMCM3_LM )
#define WSN_TestApp1_RTOS_MSG_EVT       0x0002
#endif  

#define Node_Coord_Type					0x00
#define Node_Router_Type				0x01
#define Node_EndDevice_Type				0x02

typedef struct NodeAddrInfo{
uint16 MyShortAddr;
uint8	MyExtAddr[8];
uint8	MyDeviceType;
uint16	ParentShortAddr;
}NodeAddr;

typedef struct DeviceNode{
	NodeAddr DeviceNodeAddr;
	struct DeviceNode* pNextNode;
}DeviceEndPoint;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Generic Application
 */
extern void WSN_TestApp1_Init( byte task_id );

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 WSN_TestApp1_ProcessEvent( byte task_id, UINT16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* WSN_TestApp1_H */
