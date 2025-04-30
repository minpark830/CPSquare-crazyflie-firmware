/*
 * Copyright (C) 2025 CPSquare Lab
 * Written by Minwoo Park 
*/

// importing standard C libraries
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// importing bitcraze C libraries
#include "app.h"
#include "commander.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "param.h"
#include "radiolink.h"
#include "configblock.h"
#include "debug.h"
#include "estimator_kalman.h"
#include "token_ring.h"
#include "DTR_p2p_interface.h"

// defining constants
#define DEBUG_MODULE "APPFOLLOWER"

#define START 1
#define SEND_DATA 2
#define LAND 3
#define SQUARE_FORM 4

typedef enum {
  init,
  standby,
  square_formation,
  landing
} State;

typedef enum {
  nothing,
  start,
  square,
  land
} Command;

// define the ids of each node in the network
#define NETWORK_TOPOLOGY {.size = 2, .devices_ids = {232, 233} } // Maximum size of network is 20 by default

#define FOLLOWER_2_ID 232
#define FOLLOWER_3_ID 233

// store current id of drone in P2P DTR network
static uint8_t my_id;

// store defined network topology and ids in the network
static dtrTopology topology = NETWORK_TOPOLOGY;

// store leader drone current position

static float leaderX;
static float leaderY;
static float leaderZ;

static int command;


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
	transmitSignal.targetId = FOLLOWER_2_ID;
	
	memcpy(&transmitSignal.data[0], &x, sizeof(float));         
	memcpy(&transmitSignal.data[4], &y, sizeof(float));         
	memcpy(&transmitSignal.data[8], &z, sizeof(float));       
  
	transmitSignal.dataSize = 3 * sizeof(float);  
	transmitSignal.packetSize = DTR_PACKET_HEADER_SIZE + transmitSignal.dataSize;
  
	bool res = dtrSendPacket(&transmitSignal);
  
	if (res) {
		DTR_DEBUG_PRINT("Send Follower Position\n");
	} else {
		DEBUG_PRINT("Didn't Send Follower Position\n");
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
	//paramVarId_t idFlowDeck = paramGetVarId("deck", "bcFlow2");
	
	// set self id for network
	my_id = dtrGetSelfId();

	// enable P2P DTR network
	dtrEnableProtocol(topology);

	DEBUG_PRINT("ID: %d\n", my_id);

	vTaskDelay(2000);

	// register the callback function so that the CF can receive packets as well
	p2pRegisterCB(p2pcallbackHandler);

	dtrPacket receivedPacket;


	//DEBUG_PRINT("First Kalman Filter Reset\n");

	// reset kalman filter
	estimatorKalmanInit();

	while(1){

		//uint8_t flowDeckOn = paramGetUint(idFlowDeck);

		if(state==init){
			// wait for command to start up from leader drone
			// wait until a P2P DTR packet is received

			if(dtrGetPacket(&receivedPacket, portMAX_DELAY)){
				//DEBUG_PRINT("Received data from %d : \n",receivedPacket.sourceId);
				estimatorKalmanInit();
				// received the start command from leader drone
				if(receivedPacket.dataSize == sizeof(int) && receivedPacket.data[0] == START && receivedPacket.sourceId == FOLLOWER_2_ID){
					
					DEBUG_PRINT("Received start command from leader\n");
					state = standby;
					setHoverSetpoint(&setpoint, 0, 0, 0.5, 0);
					commanderSetSetpoint(&setpoint, 3);
				}
			} 
			
		} else if(state==standby){

			DEBUG_PRINT("Current State: standby\n");
			// NOTE IMPORTANT TO KEEP DELAY NOT TOO FAST
			vTaskDelay(10);
			setHoverSetpoint(&setpoint, 0, 0, 0.5, 0);
			commanderSetSetpoint(&setpoint, 3);

			if(dtrGetPacket(&receivedPacket, 10)){
				if(receivedPacket.dataSize == sizeof(int) && receivedPacket.sourceId == FOLLOWER_2_ID && receivedPacket.targetId == my_id){
					memcpy(&command, &receivedPacket.data[0], sizeof(int));
					DEBUG_PRINT("Command is %d\n", command);
					switch(command){
						case SQUARE_FORM:
							state = square_formation;
							vTaskDelay(10);
							setHoverSetpoint(&setpoint, 0, -0.5, 0.5, 0);
							commanderSetSetpoint(&setpoint, 3);
							break;
						case LAND:
							state = landing;
							break;
					}
				} else if(receivedPacket.dataSize == 3*sizeof(float) && receivedPacket.sourceId == FOLLOWER_2_ID && receivedPacket.targetId == my_id){
					memcpy(&leaderX, &receivedPacket.data[0], sizeof(float));
					memcpy(&leaderY, &receivedPacket.data[4], sizeof(float));
					memcpy(&leaderZ, &receivedPacket.data[8], sizeof(float));
					DEBUG_PRINT("Received packet from Leader drone\n");
					DEBUG_PRINT("From %d x: %f, y: %f, z: %f\n", receivedPacket.sourceId, (double)leaderX, (double)leaderY, (double)leaderZ);
				}
			}

		} else if(state==square_formation){

			DEBUG_PRINT("Current State: square_formation\n");

			float newX = leaderX;
			float newY = leaderY - 0.5f;

			vTaskDelay(10);
			setHoverSetpoint(&setpoint, newX, newY, 0.5, 0);
			commanderSetSetpoint(&setpoint, 3);


			if(dtrGetPacket(&receivedPacket, 10)){
				if(receivedPacket.dataSize == sizeof(int) && receivedPacket.sourceId == FOLLOWER_2_ID && receivedPacket.targetId == my_id){
					memcpy(&command, &receivedPacket.data[0], sizeof(int));
					DEBUG_PRINT("Command is %d\n", command);
					switch(command){
						case SEND_DATA:
							DEBUG_PRINT("Send Data to Leader\n");
							DEBUG_PRINT("x: %f, y: %f, z: %f\n", (double)logGetFloat(idFlowX), (double)logGetFloat(idFlowY), (double)logGetFloat(idFlowZ));
							sendFollowerPosition(logGetFloat(idFlowX), logGetFloat(idFlowY), logGetFloat(idFlowZ));	
							break;
						case LAND:
							state = landing;
							break;
					}
				} else if(receivedPacket.dataSize == 3*sizeof(float) && receivedPacket.sourceId == FOLLOWER_2_ID && receivedPacket.targetId == my_id){
					memcpy(&leaderX, &receivedPacket.data[0], sizeof(float));
					memcpy(&leaderY, &receivedPacket.data[4], sizeof(float));
					memcpy(&leaderZ, &receivedPacket.data[8], sizeof(float));
					DEBUG_PRINT("Received packet from Leader drone\n");
					DEBUG_PRINT("From %d x: %f, y: %f, z: %f\n", receivedPacket.sourceId, (double)leaderX, (double)leaderY, (double)leaderZ);
				}
			}

		} else if(state==landing){
			DEBUG_PRINT("Current State: landing\n");

			vTaskDelay(10);
			setHoverSetpoint(&setpoint, logGetFloat(idFlowX), logGetFloat(idFlowY), 0.1, 0);
			commanderSetSetpoint(&setpoint, 3);
			state = init;
		} else{

		}
		
		
	}
}
