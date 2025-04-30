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

// defining constants 
#define DEBUG_MODULE "APPLEADER"

// define the ids of each node in the network
#define NETWORK_TOPOLOGY {.size = 2, .devices_ids = {230, 231} } // Maximum size of network is 20 by default
//#define NETWORK_TOPOLOGY {.size = 4, .devices_ids = {0, 1, 2, 3} } // Maximum size of network is 20 by default

#define LEADER_ID 230
#define FOLLOWER_1_ID 231

#define START 1
#define SEND_DATA 2
#define LAND 3
#define SQUARE_FORM 4

// store current id of drone in P2P DTR network
static uint8_t my_id;

// store defined network topology and ids in the network
static dtrTopology topology = NETWORK_TOPOLOGY;

// define received packet for app channel
typedef struct appChannelRX_s {
  int command;
} __attribute__((packed)) appChannelRX;

// define transmit packet for app channel
typedef struct appChannelTX_s {
  int id;
  float x;
  float y;
  float z;
} __attribute__((packed)) appChannelTX;

// function to define position hover setpoint using positional values (x,y,z, yaw)
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

// function to define velocity hover setpoint using velocity values
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
static void transmitData(appChannelTX *txPacket, int id, float x, float y, float z){
  txPacket->id = id;
  txPacket->x = x;
  txPacket->y = y;
  txPacket->z = z;

  appchannelSendDataPacketBlock(txPacket, sizeof(*txPacket)); 
  
}

// function to load/send transmit packet so it transmit current x, y, z of leader drone to follower
void sendLeaderPosition(int targetID, float x, float y, float z){
  dtrPacket transmitSignal;
  transmitSignal.messageType = DATA_FRAME;
  transmitSignal.sourceId = my_id;
  transmitSignal.targetId = targetID;
  
  memcpy(&transmitSignal.data[0], &x, sizeof(float));         
  memcpy(&transmitSignal.data[4], &y, sizeof(float));         
  memcpy(&transmitSignal.data[8], &z, sizeof(float));       

  transmitSignal.dataSize = 3 * sizeof(float);  
  transmitSignal.packetSize = DTR_PACKET_HEADER_SIZE + transmitSignal.dataSize;

  bool res = dtrSendPacket(&transmitSignal);

  if (res) {
      DTR_DEBUG_PRINT("Send Leader Position\n");
  } else {
      DEBUG_PRINT("Didn't Send Leader Position\n");
  }

}

// function to load/send transmit packet so it transmit command to follower drone
void sendCommandToFollower(int targetID, int command){
  DEBUG_PRINT("trying to send command\n");
  dtrPacket transmitSignal;
  transmitSignal.messageType = DATA_FRAME;
  transmitSignal.sourceId = my_id;
  transmitSignal.targetId = targetID;

  // Copy full 4-byte integer into the data buffer
  memcpy(&transmitSignal.data[0], &command, sizeof(int));

  transmitSignal.dataSize = sizeof(int); // Now sending 4 bytes, not 1
  transmitSignal.packetSize = DTR_PACKET_HEADER_SIZE + transmitSignal.dataSize;

  bool res = dtrSendPacket(&transmitSignal);

  if (res) {
      DTR_DEBUG_PRINT("Send Command to Follower\n");
  } else {
      DEBUG_PRINT("Didn't Send Command to Follower\n");
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

typedef enum {
  leader,
  follower_1,
  leader_send,
  init_send
} P2pState;

// store current id of drone in P2P DTR network
//static uint8_t my_id;

// store defined network topology and ids in the network
//static dtrTopology topology = NETWORK_TOPOLOGY;

// leader starts in init state
static State state = init;

// leader starts in nothing command
static Command command = nothing;

static P2pState p2pState = init_send;

static float receivedX;
static float receivedY;
static float receivedZ;

void appMain() {

  vTaskDelay(3000);
 
  static appChannelRX rxPacket;
  static appChannelTX txPacket;
  //static dtrPacket received_packet;
  static setpoint_t setpoint;

  logVarId_t idFlowX = logGetVarId("stateEstimate", "x");
  logVarId_t idFlowY = logGetVarId("stateEstimate", "y");
  logVarId_t idFlowZ = logGetVarId("stateEstimate", "z");
  //paramVarId_t idFlowDeck = paramGetVarId("deck", "bcFlow2");

  dtrPacket receivedPacket;
  //bool firstCommand = true;

  // set self id for network
	my_id = dtrGetSelfId();

	// enable P2P DTR network
	dtrEnableProtocol(topology);

  DEBUG_PRINT("ID: %d\n", my_id);

  vTaskDelay(2000);
  // register the callback function so that the CF can receive packets as well
	p2pRegisterCB(p2pcallbackHandler);

  DEBUG_PRINT("Current ID: %d\n", my_id);

  Command previousCommand = nothing;

  // reset kalman filter
  estimatorKalmanInit();

  while(1) {

    //uint8_t flowDeckOn = paramGetUint(idFlowDeck);

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
            sendCommandToFollower(0xFF,START);
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

        sendLeaderPosition(0xFF, logGetFloat(idFlowX), logGetFloat(idFlowY), logGetFloat(idFlowZ));

        switch (command) {
          case square:
            state = square_formation;
            previousCommand = nothing;
            p2pState = init_send;
            break;
          case rhombus:
            state = rhombus_formation;
            previousCommand = nothing;
            p2pState = init_send;
            break;
          case triangle:
            state = triangle_formation;
            previousCommand = nothing;
            p2pState = init_send;
            break;
          case land:
            state = landing;
            break;
          default:
              DEBUG_PRINT("NO VALID COMMAND SENT, AT STATE: standby\n");
        }
        
      }

    } else if(state == square_formation){ 

      //DEBUG_PRINT("Current State: square_formation\n");

      switch(previousCommand){
        case right:
          vTaskDelay(10);
          setVelocityHoverSetpoint(&setpoint, 0, -0.1, 0, 0);
          commanderSetSetpoint(&setpoint, 3);
          break;
        case left:
          vTaskDelay(10);
          setVelocityHoverSetpoint(&setpoint, 0, 0.1, 0, 0);
          commanderSetSetpoint(&setpoint, 3);
          break;
        case back:
          vTaskDelay(10);
          setVelocityHoverSetpoint(&setpoint, -0.1, 0, 0, 0);
          commanderSetSetpoint(&setpoint, 3);
          break;
        case forward:
          vTaskDelay(10);
          setVelocityHoverSetpoint(&setpoint, 0.1, 0, 0, 0);
          commanderSetSetpoint(&setpoint, 3);
          break;
        default:
          vTaskDelay(10);
          setVelocityHoverSetpoint(&setpoint, 0, 0, 0, 0);
          commanderSetSetpoint(&setpoint, 3);
          break;
      }

      if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 0)) {
        command = (int)rxPacket.command;

        switch (command) {
          case rhombus:
            vTaskDelay(10);
            setVelocityHoverSetpoint(&setpoint, 0, 0, 0, 0);
            commanderSetSetpoint(&setpoint, 3);
            state = rhombus_formation;
            break;
          case triangle:
            vTaskDelay(10);
            setVelocityHoverSetpoint(&setpoint, 0, 0, 0, 0);
            commanderSetSetpoint(&setpoint, 3);
            state = triangle_formation;
            break;
          case right:
            vTaskDelay(10);
            setVelocityHoverSetpoint(&setpoint, 0, -0.1, 0, 0);
            commanderSetSetpoint(&setpoint, 3);
            previousCommand = right;
            break;
          case left:
            vTaskDelay(10);
            setVelocityHoverSetpoint(&setpoint, 0, 0.1, 0, 0);
            commanderSetSetpoint(&setpoint, 3);
            previousCommand = left;
            break;
          case back:
            vTaskDelay(10);
            setVelocityHoverSetpoint(&setpoint, -0.1, 0, 0, 0);
            commanderSetSetpoint(&setpoint, 3);
            previousCommand = back;
            break;
          case forward:
            vTaskDelay(10);
            setVelocityHoverSetpoint(&setpoint, 0.1, 0, 0, 0);
            commanderSetSetpoint(&setpoint, 3);
            previousCommand = forward;
            break;
          case stop:
            vTaskDelay(10);
            setVelocityHoverSetpoint(&setpoint, 0, 0, 0, 0);
            commanderSetSetpoint(&setpoint, 3);
            previousCommand = stop;
            break;
          case land:
            vTaskDelay(10);
            setVelocityHoverSetpoint(&setpoint, 0, 0, 0, 0);
            commanderSetSetpoint(&setpoint, 3);
            state = landing;
            break;
          default:
              DEBUG_PRINT("NO VALID COMMAND SENT, AT STATE: square_formation\n");
        }

      }
      
      switch(p2pState){
        case init_send:
          // send all followers the square formation command
          sendCommandToFollower(0xFF, SQUARE_FORM);
          p2pState = leader;
          break;
        case leader:
          // send leader position to app
          transmitData(&txPacket, my_id, logGetFloat(idFlowX), logGetFloat(idFlowY), logGetFloat(idFlowZ));
          p2pState = follower_1;

          // send follower 1 drone command to send position to leader
          sendCommandToFollower(FOLLOWER_1_ID, SEND_DATA);
          break;
        case follower_1:
          if(dtrGetPacket(&receivedPacket, 10)){
            DEBUG_PRINT("Received packet from Follower 1 drone\n");
            if(receivedPacket.dataSize == 3*sizeof(float) && receivedPacket.sourceId == FOLLOWER_1_ID && receivedPacket.targetId == my_id){
              memcpy(&receivedX, &receivedPacket.data[0], sizeof(float));
              memcpy(&receivedY, &receivedPacket.data[4], sizeof(float));
              memcpy(&receivedZ, &receivedPacket.data[8], sizeof(float));

              transmitData(&txPacket, FOLLOWER_1_ID, receivedX, receivedY, receivedZ);
              DEBUG_PRINT("x: %f, y: %f, z: %f\n", (double)receivedX, (double)receivedY, (double)receivedZ);
              p2pState = leader_send;
            } 
          }
          break;
        case leader_send:
          sendLeaderPosition(0xFF, logGetFloat(idFlowX), logGetFloat(idFlowY), logGetFloat(idFlowZ));
          p2pState = leader;
          break;
      }

    } else if(state == rhombus_formation){
      
    } else if(state == landing){
      // for some reason landing produces a lock and reboot required for supervisor
      //DEBUG_PRINT("Current State: landing\n");

      sendCommandToFollower(0xFF,LAND);

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
 