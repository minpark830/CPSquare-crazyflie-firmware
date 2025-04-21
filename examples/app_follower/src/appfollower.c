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
#include "estimator_kalman.h"

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
#define NETWORK_TOPOLOGY {.size = 3, .devices_ids = {231, 230, 232} } // Maximum size of network is 20 by default
//#define NETWORK_TOPOLOGY {.size = 4, .devices_ids = {0, 1, 2, 3} } // Maximum size of network is 20 by default

#define LEADER_ID 231

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

// define hover setpoint function to be based off positional values (x,y,z, yaw)
static void setHoverSetpoint(setpoint_t *setpoint, float x, float y, float z, float yaw){
	setpoint->mode.z = modeAbs;
	setpoint->position.z = z;
   
   
	setpoint->mode.yaw = modeAbs;
	setpoint->attitude.yaw = yaw;
   
   
	setpoint->mode.x = modeAbs;
	setpoint->mode.y = modeAbs;
	setpoint->position.x = x;
	setpoint->position.y = y;
  
}

// function to load/send transmit packet so it transmit current x, y, z of leader drone to follower
void sendFollowerPosition(float x, float y, float z){
	dtrPacket transmitSignal;
	transmitSignal.messageType = DATA_FRAME;
	transmitSignal.sourceId = my_id;
	transmitSignal.dataSize = 3*sizeof(float);
	transmitSignal.data[0] = x;
	transmitSignal.data[1] = y;
	transmitSignal.data[2] = z;
	// transmit to follower drone
	transmitSignal.targetId = LEADER_ID;
	transmitSignal.packetSize = DTR_PACKET_HEADER_SIZE + transmitSignal.dataSize;

	bool res;
	res = dtrSendPacket(&transmitSignal);
	if (res){
		DTR_DEBUG_PRINT("Leader Packet sent to DTR protocol\n");
	}
	else{
		DEBUG_PRINT("Leader Packet not sent to DTR protocol\n");
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

void appMain(){

	static setpoint_t setpoint;

	logVarId_t idFlowX = logGetVarId("stateEstimate", "x");
  	logVarId_t idFlowY = logGetVarId("stateEstimate", "y");
	logVarId_t idFlowZ = logGetVarId("stateEstimate", "z");
	paramVarId_t idFlowDeck = paramGetVarId("deck", "bcFlow2");
	
	// set self id for network
	my_id = dtrGetSelfId();

	// DEBUG_PRINT("Current ID: %d\n", my_id);

	// DEBUG_PRINT("Network Topology: %d", topology.size);
	// for (int i = 0; i < topology.size; i++){
	// 	DEBUG_PRINT("%d ", topology.devices_ids[i]);
	// }
	// DEBUG_PRINT("\n");

	// enable P2P DTR network
	dtrEnableProtocol(topology);

	vTaskDelay(2000);

	// register the callback function so that the CF can receive packets as well
	p2pRegisterCB(p2pcallbackHandler);

	dtrPacket receivedPacket;
	// float currentX;
	// float currentY;
	// float currentZ;

	DEBUG_PRINT("First Kalman Filter Reset\n");

	// reset kalman filter
	estimatorKalmanInit();

	while(1){

		uint8_t flowDeckOn = paramGetUint(idFlowDeck);

		if(state==init){
			// wait for command to start up from leader drone
			// wait until a P2P DTR packet is received

			if(dtrGetPacket(&receivedPacket, portMAX_DELAY)){
				DEBUG_PRINT("Received data from %d : \n",receivedPacket.sourceId);
				estimatorKalmanInit();
				
				DEBUG_PRINT("Kalman Filter Reset\n");

				if(flowDeckOn){
					DEBUG_PRINT("x: %f, y: %f, z: %f\n", (double)logGetFloat(idFlowX), (double)logGetFloat(idFlowY), (double)logGetFloat(idFlowZ));
				}

				// received the start command from leader drone
				if(receivedPacket.dataSize == 1 && receivedPacket.data[0] == start){
					DEBUG_PRINT("Received start command from leader\n");
					state = standby;
					setHoverSetpoint(&setpoint, 0, 0, 0.4, 0);
					commanderSetSetpoint(&setpoint, 3);
				}
			} 
			
		} else if(state==standby){

			//DEBUG_PRINT("Current State: standby\n");
			// NOTE IMPORTANT TO KEEP DELAY NOT TOO FAST
			vTaskDelay(10);
			if(flowDeckOn){
				DEBUG_PRINT("x: %f, y: %f, z: %f\n", (double)logGetFloat(idFlowX), (double)logGetFloat(idFlowY), (double)logGetFloat(idFlowZ));
			}
			setHoverSetpoint(&setpoint, 0, 0, 0.4, 0);
			commanderSetSetpoint(&setpoint, 3);

			sendFollowerPosition((double)logGetFloat(idFlowX), (double)logGetFloat(idFlowY), (double)logGetFloat(idFlowZ));

			if(dtrGetPacket(&receivedPacket, 0)){
				DEBUG_PRINT("Received packet from Leader drone\n");
				DEBUG_PRINT("From %d x: %f, y: %f, z: %f\n", receivedPacket.sourceId, receivedPacket.data[0], receivedPacket.data[1], receivedPacket.data[2]);
			}

		} else if(state==square_formation){

		} else{

		}
		
		
	}
}
