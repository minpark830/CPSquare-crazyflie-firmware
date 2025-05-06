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

#include "formation.h"
#include "formation.c"

#define MESSAGE_LENGHT 1

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
// #define NETWORK_TOPOLOGY {.size = 3, .devices_ids = {231, 232, 230} } // Maximum size of network is 20 by default
//#define NETWORK_TOPOLOGY {.size = 4, .devices_ids = {0, 1, 2, 3} } // Maximum size of network is 20 by default

#define LEADER_ID 231
#define FOLLOWER_1_ID 232
#define FOLLOWER_2_ID 230
#define FOLLOWER_3_ID 233

// store current id of drone in P2P DTR network
static uint8_t my_id;

// store defined network topology and ids in the network
// static dtrTopology topology = NETWORK_TOPOLOGY;

// store leader drone current position

static float leaderX = 0.0;
static float leaderY = 0.0;
static float leaderZ = 0.0;

// static int command;


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
// void sendFollowerPosition(float x, float y, float z){
// 	dtrPacket transmitSignal;
// 	transmitSignal.messageType = DATA_FRAME;
// 	transmitSignal.sourceId = my_id;
// 	transmitSignal.targetId = LEADER_ID;
	
// 	memcpy(&transmitSignal.data[0], &x, sizeof(float));         
// 	memcpy(&transmitSignal.data[4], &y, sizeof(float));         
// 	memcpy(&transmitSignal.data[8], &z, sizeof(float));       
  
// 	transmitSignal.dataSize = 3 * sizeof(float);  
// 	transmitSignal.packetSize = DTR_PACKET_HEADER_SIZE + transmitSignal.dataSize;
  
// 	bool res = dtrSendPacket(&transmitSignal);
  
// 	if (res) {
// 		DTR_DEBUG_PRINT("Send Follower Position\n");
// 	} else {
// 		DEBUG_PRINT("Didn't Send Follower Position\n");
// 	}
// }

void p2preply(void){
   
	logVarId_t idFlowX = logGetVarId("stateEstimate", "x");
	logVarId_t idFlowY = logGetVarId("stateEstimate", "y");
	logVarId_t idFlowZ = logGetVarId("stateEstimate", "z");
	// paramVarId_t idFlowDeck = paramGetVarId("deck", "bcFlow2");
	float pos_X = logGetFloat(idFlowX);
	float pos_Y = logGetFloat(idFlowY);
	float pos_Z = logGetFloat(idFlowZ);
	static P2PPacket p_reply;
	p_reply.port=0x00;
	p_reply.data[0]=my_id;
	//char *str="G";
	memcpy(&p_reply.data[1], &pos_X, sizeof(float));
	memcpy(&p_reply.data[1 + sizeof(float)], &pos_Y, sizeof(float));
	memcpy(&p_reply.data[1 + (2*sizeof(float))], &pos_Z, sizeof(float));
	p_reply.size = 1 + 3 * sizeof(float);  // ID + velFront + velSide
	radiolinkSendP2PPacketBroadcast(&p_reply);
	DEBUG_PRINT("p2preply sent\n");
   }

   void p2pcallbackHandler(P2PPacket *p){
	// Parse the data from the other crazyflie and print it
	uint8_t other_id = p->data[0];
	static char msg[MESSAGE_LENGHT + 1];
	memcpy(&msg, &p->data[1], sizeof(char)*MESSAGE_LENGHT);
	msg[MESSAGE_LENGHT] = 0;
	uint8_t rssi = p->rssi;
	DEBUG_PRINT("[RSSI: -%d dBm] Message from CF nr. %d, %s\n", rssi, other_id, msg);
	if (other_id == LEADER_ID){
	  DEBUG_PRINT("in msg\n");
	  if (strcmp(msg, "i") == 0) {
		DEBUG_PRINT("idle\n");
		state = init;
	  } 
	  else if (strcmp(msg, "w") == 0) {
		DEBUG_PRINT("standby\n");
		p2preply();
		state = standby;
		
	  }
	  else if (strcmp(msg, "s") == 0) {
		DEBUG_PRINT("formation\n");
		p2preply();
		state = square_formation;
		
	  }
	  else if (strcmp(msg, "l") == 0) {
		DEBUG_PRINT("stopping\n");
		state = landing;
	  }
	  else if (state == square_formation) {
		memcpy(&leaderX, &p->data[1], sizeof(float));
		memcpy(&leaderY, &p->data[1 + sizeof(float)], sizeof(float));
		memcpy(&leaderZ, &p->data[1 + (2*sizeof(float))], sizeof(float));
		p2preply();
		//DEBUG_PRINT("Received velFront = %f, velSide = %f\n", velFront, (double)velSide);
	  }
	  
	}
  }

void appMain(){

	static setpoint_t setpoint;

	logVarId_t idFlowX = logGetVarId("stateEstimate", "x");
  	logVarId_t idFlowY = logGetVarId("stateEstimate", "y");
	// logVarId_t idFlowZ = logGetVarId("stateEstimate", "z");
	//paramVarId_t idFlowDeck = paramGetVarId("deck", "bcFlow2");
	
	// set self id for network
	my_id = dtrGetSelfId();

	DEBUG_PRINT("ID: %d\n", my_id);

	vTaskDelay(2000);
	// float timeOuter;
	// register the callback function so that the CF can receive packets as well
	p2pRegisterCB(p2pcallbackHandler);

	static P2PPacket p_reply;
	p_reply.port=0x00;
	p_reply.data[0]=my_id;


	//DEBUG_PRINT("First Kalman Filter Reset\n");

	// reset kalman filter
	estimatorKalmanInit();

	while(1){

		//uint8_t flowDeckOn = paramGetUint(idFlowDeck);

		if(state==init){
			// wait for command to start up from leader drone
			// wait until a P2P DTR packet is received
			vTaskDelay(M2T(30));
			memset(&setpoint, 0, sizeof(setpoint_t));
        	commanderSetSetpoint(&setpoint, 3);
			// if(dtrGetPacket(&receivedPacket, portMAX_DELAY)){
			// 	//DEBUG_PRINT("Received data from %d : \n",receivedPacket.sourceId);
			// 	estimatorKalmanInit();
			// 	// received the start command from leader drone
			// 	if(receivedPacket.dataSize == sizeof(int) && receivedPacket.data[0] == START && receivedPacket.sourceId == LEADER_ID){
					
			// 		DEBUG_PRINT("Received start command from leader\n");
			// 		state = standby;
			// 		setHoverSetpoint(&setpoint, 0, 0, 0.5, 0);
			// 		commanderSetSetpoint(&setpoint, 3);
			// 	}
			// } 
			
		} else if(state==standby){

			DEBUG_PRINT("Current State: standby\n");
			// NOTE IMPORTANT TO KEEP DELAY NOT TOO FAST
			vTaskDelay(10);
			setHoverSetpoint(&setpoint, 0, 0, 0.5, 0);
			commanderSetSetpoint(&setpoint, 3);

			// if(dtrGetPacket(&receivedPacket, 10)){
			// 	if(receivedPacket.dataSize == sizeof(int) && receivedPacket.sourceId == LEADER_ID && receivedPacket.targetId == my_id){
			// 		memcpy(&command, &receivedPacket.data[0], sizeof(int));
			// 		DEBUG_PRINT("Command is %d\n", command);
			// 		switch(command){
			// 			case SQUARE_FORM:
			// 				state = square_formation;
 			// 				initDesiredPosition(1, my_id);
			// 				break;
			// 			case LAND:
			// 				state = landing;
			// 				break;
			// 		}
			// 	} 
			// }

		} else if(state==square_formation){

			DEBUG_PRINT("Current State: square_formation\n");
			static float newX = 0;
			static float newY = 0;

			switch(my_id){
				case FOLLOWER_1_ID:
					newX = leaderX + 0.2f;
					newY = leaderY;
					break;
				case FOLLOWER_2_ID:
					newX = leaderX;
					newY = leaderY - 0.2f;
					break;
				case FOLLOWER_3_ID:
					newX = leaderX + 0.2f;
					newY = leaderY - 0.2f;
					break;
			}
			
			// if(dtrGetPacket(&receivedPacket, 10)){
			// 	DEBUG_PRINT("Packet Recived\n");
			// 	if(receivedPacket.dataSize == sizeof(int) && receivedPacket.sourceId == LEADER_ID && receivedPacket.targetId == my_id){
			// 		memcpy(&command, &receivedPacket.data[0], sizeof(int));
			// 		DEBUG_PRINT("Command is %d\n", command);
			// 		switch(command){
			// 			case SEND_DATA:
			// 				DEBUG_PRINT("Send Data to Leader\n");
			// 				DEBUG_PRINT("x: %f, y: %f, z: %f\n", (double)logGetFloat(idFlowX), (double)logGetFloat(idFlowY), (double)logGetFloat(idFlowZ));
			// 				sendFollowerPosition(logGetFloat(idFlowX), logGetFloat(idFlowY), logGetFloat(idFlowZ));	
			// 				break;
			// 			case LAND:
			// 				state = landing;
			// 				break;
			// 		}
			// 	} 
			// }

			// timeOuter = usecTimestamp() / 1e6;
			// calculatePosition(leaderX, leaderY, &newX, &newY, timeOuter);
			vTaskDelay(20);
			setHoverSetpoint(&setpoint, newX, newY, 0.5, 0);
			commanderSetSetpoint(&setpoint, 3);

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
