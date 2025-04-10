/*
 * Copyright (C) 2025 CPSquare Lab
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "app.h"
#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "param.h"

#include "radiolink.h"
#include "configblock.h"

#define DEBUG_MODULE "APPFOLLOWER"
#include "debug.h"

#include "token_ring.h"
#include "DTR_p2p_interface.h"

typedef enum {
  init,
  standby,
  square_formation
} State;

typedef enum {
  nothing,
  start,
  square,
  land
} Command;

// define the ids of each node in the network
#define NETWORK_TOPOLOGY {.size = 4, .devices_ids = {0, 1, 2, 3} } // Maximum size of network is 20 by default

// store current id of drone in P2P DTR network
static uint8_t my_id;

// store defined network topology and ids in the network
static dtrTopology topology = NETWORK_TOPOLOGY;

// store leader drone current position
/*
static float leaderX;
static float leaderY;
static float leaderZ;
*/

// leader starts in init state
static State state = init;

// leader starts in nothing command
//static Command command = nothing;

// function to load/send transmit packet so it transmit current x, y, z of follower drone
void sendPackets(float x, float y, float z){
	dtrPacket transmitSignal;
	transmitSignal.messageType = DATA_FRAME;
	transmitSignal.sourceId = my_id;

	// create a array of 3 floats and copy it to transmit packet data
	float followerPos[3] = {x, y, z};
	memcpy(transmitSignal.data, followerPos, sizeof(followerPos));
	
	transmitSignal.dataSize = sizeof(followerPos);
	// transmit to leader drone, id of 0 (first to join network)
	transmitSignal.targetId = 0;
	transmitSignal.packetSize = DTR_PACKET_HEADER_SIZE + transmitSignal.dataSize;

	bool res;
	res = dtrSendPacket(&transmitSignal);
	if (res){
		DTR_DEBUG_PRINT("Packet sent to DTR protocol\n");
	}
	else{
		DEBUG_PRINT("Packet not sent to DTR protocol\n");
	}
}

void p2pcallbackHandler(P2PPacket *p){
	// If the packet is a DTR service packet, then the handler will handle it.
	// It returns true if the packet was handled.

    if (!dtrP2PIncomingHandler(p)){
		// If packet was not handled from DTR , then it is a normal packet 
		// that user has to handle. 
	}
}


// static void setHoverSetpoint(setpoint_t *setpoint, float x, float y, float z, float yaw){
// 	setpoint->mode.z = modeAbs;
// 	setpoint->position.z = z;
   
   
// 	setpoint->mode.yaw = modeAbs;
// 	setpoint->attitude.yaw = yaw;
   
   
// 	setpoint->mode.x = modeAbs;
// 	setpoint->mode.y = modeAbs;
// 	setpoint->position.x = x;
// 	setpoint->position.y = y;
  
//   }

void appMain(){

	logVarId_t idFlowX = logGetVarId("stateEstimate", "x");
  	logVarId_t idFlowY = logGetVarId("stateEstimate", "y");
	logVarId_t idFlowZ = logGetVarId("stateEstimate", "z");
	paramVarId_t idFlowDeck = paramGetVarId("deck", "bcFlow2");

	//static setpoint_t setpoint;
	
	// set self id for network
	my_id = dtrGetSelfId();

	// enable P2P DTR network
	dtrEnableProtocol(topology);

	vTaskDelay(2000);

	// register the callback function so that the CF can receive packets as well
	p2pRegisterCB(p2pcallbackHandler);

	dtrPacket receivedPacket;
	float currentX;
	float currentY;
	float currentZ;

	while(1){

		uint8_t flowDeckOn = paramGetUint(idFlowDeck);

		if(state==init){
			// wait for command to start up from leader drone
			// wait until a P2P DTR packet is received
			if(dtrGetPacket(&receivedPacket, portMAX_DELAY)){
				DEBUG_PRINT("Received data from %d : \n",receivedPacket.sourceId);
				
				

			} 
			// regardless of if command is sent or not send back current position to 
			if(flowDeckOn){

				currentX = logGetFloat(idFlowX);
				currentY = logGetFloat(idFlowY);
				currentZ = logGetFloat(idFlowZ);
	
				sendPackets(currentX, currentY, currentZ);
			}
		} else if(state==standby){
 
		} else if(state==square_formation){

		} else{

		}
		
		
	}
}
