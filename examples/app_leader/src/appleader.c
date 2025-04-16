/*
 * Copyright (C) 2025 CPSquare Lab
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "app.h"
#include "app_channel.h"
#include "commander.h"
#include "FreeRTOS.h"
#include "task.h"
#include "estimator_kalman.h"

#include "radiolink.h"
#include "configblock.h"
 
#include "debug.h"

#include "log.h"
#include "param.h"

#include "token_ring.h"
#include "DTR_p2p_interface.h"
 
#define DEBUG_MODULE "APPLEADER"

// define the ids of each node in the network
#define NETWORK_TOPOLOGY {.size = 2, .devices_ids = {231, 232} } // Maximum size of network is 20 by default
//#define NETWORK_TOPOLOGY {.size = 4, .devices_ids = {0, 1, 2, 3} } // Maximum size of network is 20 by default

#define LEADER_ID 231

// store current id of drone in P2P DTR network
static uint8_t my_id;

// store defined network topology and ids in the network
static dtrTopology topology = NETWORK_TOPOLOGY;

// define received packet for app channel
typedef struct testPacketRX_s {
  int command;
} __attribute__((packed)) testPacketRX;

// define transmit packet for app channel
typedef struct testPacketTX_s {
  int id;
  float x;
  float y;
  float z;
} __attribute__((packed)) testPacketTX;

// static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate){
//   setpoint->mode.z = modeAbs;
//   setpoint->position.z = z;
//   setpoint->mode.yaw = modeVelocity;
//   setpoint->attitudeRate.yaw = yawrate;
//   setpoint->mode.x = modeVelocity;
//   setpoint->mode.y = modeVelocity;
//   setpoint->velocity.x = vx;
//   setpoint->velocity.y = vy;
//   setpoint->velocity_body = true;
// }

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

// define velocity hover setpoint function based off velocity values
static void setVelocityHoverSetpoint(setpoint_t *setpoint, float xVeloc, float yVeloc, float zVeloc, float yawVeloc){
	setpoint->mode.z = modeVelocity;
	setpoint->velocity.z = zVeloc;
   
   
	setpoint->mode.yaw = modeAbs;
	setpoint->attitudeRate.yaw = yawVeloc;
   
   
	setpoint->mode.x = modeVelocity;
	setpoint->mode.y = modeVelocity;
	setpoint->velocity.x = xVeloc;
	setpoint->velocity.y = yVeloc;
  
}

// define transmit data packet function based off positional values (FlowX, FlowY, FlowZ)
static bool transmitData(uint8_t flowDeckOn, testPacketTX *txPacket, int id, logVarId_t idFlowX, logVarId_t idFlowY, logVarId_t idFlowZ){
  if(flowDeckOn){
    txPacket->id = id;
    txPacket->x = logGetFloat(idFlowX);
    txPacket->y = logGetFloat(idFlowY);
    txPacket->z = logGetFloat(idFlowZ);

    appchannelSendDataPacketBlock(txPacket, sizeof(*txPacket)); 
  } else{
    return false;
  }
  return true;
}

// function to load/send transmit packet so it transmit current x, y, z of leader drone to follower
void sendLeaderPosition(int targetID, float x, float y, float z){
	dtrPacket transmitSignal;
	transmitSignal.messageType = DATA_FRAME;
	transmitSignal.sourceId = my_id;

	// create a array of 3 floats and copy it to transmit packet data
	float leaderPos[3] = {x, y, z};
	memcpy(transmitSignal.data, leaderPos, sizeof(leaderPos));
	
	transmitSignal.dataSize = sizeof(leaderPos);
	// transmit to follower drone
	transmitSignal.targetId = targetID;
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

// function to load/send transmit packet so it transmit command to follower drone
void sendCommandToFollower(int targetID, int command){
  dtrPacket transmitSignal;
	transmitSignal.messageType = DATA_FRAME;
	transmitSignal.sourceId = my_id;
	transmitSignal.dataSize = 1;
  transmitSignal.data[0] = command;
	// transmit to follower drone
	transmitSignal.targetId = targetID;
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

typedef enum {
  init,
  standby,
  square_formation,
  landing,
  going_right,
  going_left,
  going_back,
  going_forward,
  stopping,
  rhombus_formation,
  triangle_formation
} State;

typedef enum {
  nothing,
  start,
  square,
  land,
  right,
  left,
  back,
  forward,
  stop,
  rhombus,
  triangle
} Command;

// store current id of drone in P2P DTR network
//static uint8_t my_id;

// store defined network topology and ids in the network
//static dtrTopology topology = NETWORK_TOPOLOGY;

// leader starts in init state
static State state = init;

// leader starts in nothing command
static Command command = nothing;

void appMain() {

  vTaskDelay(3000);
 
  static testPacketRX rxPacket;
  static testPacketTX txPacket;
  //static dtrPacket received_packet;
  static setpoint_t setpoint;

  logVarId_t idFlowX = logGetVarId("stateEstimate", "x");
  logVarId_t idFlowY = logGetVarId("stateEstimate", "y");
  logVarId_t idFlowZ = logGetVarId("stateEstimate", "z");
  paramVarId_t idFlowDeck = paramGetVarId("deck", "bcFlow2");

  //bool firstCommand = true;

  // set self id for network
	my_id = dtrGetSelfId();

	// enable P2P DTR network
	dtrEnableProtocol(topology);

  vTaskDelay(2000);
  // register the callback function so that the CF can receive packets as well
	p2pRegisterCB(p2pcallbackHandler);

  DEBUG_PRINT("Current ID: %d\n", my_id);

  // reset kalman filter
  estimatorKalmanInit();

  while(1) {

    uint8_t flowDeckOn = paramGetUint(idFlowDeck);

    if(state==init) {

      DEBUG_PRINT("Current State: init\n");

      // wait for start command from the computer 
      // note it is important to keep this one as APPCHANNEL_WAIT_FOREVER for some reason
      if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), APPCHANNEL_WAIT_FOREVER)) {

        command = (int)rxPacket.command;

        // reset kalman filter
        estimatorKalmanInit();

        switch (command) {
          case start:
            state = standby;
            setHoverSetpoint(&setpoint, 0, 0, 0.5, 0);
            commanderSetSetpoint(&setpoint, 3);
            break;
          default:
            DEBUG_PRINT("NO VALID COMMAND SENT FOR STATE: init\n");
        }
      
      }

    } else if(state == standby){

      DEBUG_PRINT("Current State: standby\n");

      // it seems delay is required from the crazyflie from crashing
      vTaskDelay(10);
      setHoverSetpoint(&setpoint, 0, 0, 0.5, 0);
      commanderSetSetpoint(&setpoint, 3);

      if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 0)) {
        command = (int)rxPacket.command;

        sendCommandToFollower(0xFF,1);

        switch (command) {
          case square:
            state = square_formation;
            break;
          case rhombus:
            state = rhombus_formation;
            break;
          case triangle:
            state = triangle_formation;
            break;
          case land:
            state = landing;
            break;
          default:
              DEBUG_PRINT("NO VALID COMMAND SENT FOR STATE: standby\n");
        }
        
      }

    } else if(state == square_formation){ 

      //DEBUG_PRINT("Current State: square_formation\n");

      vTaskDelay(10);
      setHoverSetpoint(&setpoint, logGetFloat(idFlowX), logGetFloat(idFlowY), 0.5, 0);
      commanderSetSetpoint(&setpoint, 3);

      if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 0)) {
        command = (int)rxPacket.command;

        if(command == land){
          state = landing;
          transmitData(flowDeckOn, &txPacket, my_id, idFlowX, idFlowY, idFlowZ);
        }
      }

      transmitData(flowDeckOn, &txPacket, my_id, idFlowX, idFlowY, idFlowZ);

    } else if(state == going_right){
      
      DEBUG_PRINT("Current State: going_right\n");

      vTaskDelay(10);
      setVelocityHoverSetpoint(&setpoint, 0, -0.1, 0, 0);
      commanderSetSetpoint(&setpoint, 3);

      if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 0)) {
        command = (int)rxPacket.command;

        if(command == land){
          state = landing;
          transmitData(flowDeckOn, &txPacket, my_id, idFlowX, idFlowY, idFlowZ);
        }
      }


    } else if(state == going_left){
      
      DEBUG_PRINT("Current State: going_left\n");

      vTaskDelay(10);
      setVelocityHoverSetpoint(&setpoint, 0, 0.1, 0, 0);
      commanderSetSetpoint(&setpoint, 3);

      if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 0)) {
        command = (int)rxPacket.command;

        if(command == land){
          state = landing;
          transmitData(flowDeckOn, &txPacket, my_id, idFlowX, idFlowY, idFlowZ);
        }
      }


    } else if(state == going_back){
      
      DEBUG_PRINT("Current State: going_back\n");

      vTaskDelay(10);
      setVelocityHoverSetpoint(&setpoint, -0.1, 0, 0, 0);
      commanderSetSetpoint(&setpoint, 3);

      if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 0)) {
        command = (int)rxPacket.command;

        if(command == land){
          state = landing;
          transmitData(flowDeckOn, &txPacket, my_id, idFlowX, idFlowY, idFlowZ);
        }
      }


    } else if(state == going_forward){
      
      DEBUG_PRINT("Current State: going_forward\n");

      vTaskDelay(10);
      setVelocityHoverSetpoint(&setpoint, 0.1, 0, 0, 0);
      commanderSetSetpoint(&setpoint, 3);

      if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 0)) {
        command = (int)rxPacket.command;

        if(command == land){
          state = landing;
          transmitData(flowDeckOn, &txPacket, my_id, idFlowX, idFlowY, idFlowZ);
        }
      }


    } else if (state == stopping){

      DEBUG_PRINT("Current State: stopping\n");

      vTaskDelay(10);
      setVelocityHoverSetpoint(&setpoint, 0, 0, 0, 0);
      commanderSetSetpoint(&setpoint, 3);

      if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 0)) {
        command = (int)rxPacket.command;

        if(command == land){
          state = landing;
          transmitData(flowDeckOn, &txPacket, my_id, idFlowX, idFlowY, idFlowZ);
        }
      }


    } else if(state == landing){
      // for some reason landing produces a lock and reboot required for supervisor
      DEBUG_PRINT("Current State: landing\n");

      vTaskDelay(10);
      setHoverSetpoint(&setpoint, logGetFloat(idFlowX), logGetFloat(idFlowY), 0.1, 0);
      commanderSetSetpoint(&setpoint, 3);
      state = init;

    } else{

      DEBUG_PRINT("ERROR WITH STATE HANDLING\n");

      vTaskDelay(10);
      memset(&setpoint, 0, sizeof(setpoint_t));
      commanderSetSetpoint(&setpoint, 3);
    }
  }
}
 