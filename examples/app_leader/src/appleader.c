/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * appchanel_test.c: Demonstrate the appchanel functionality
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

#include "radiolink.h"
#include "configblock.h"
 
#include "debug.h"

#include "log.h"
#include "param.h"

#include "token_ring.h"
#include "DTR_p2p_interface.h"
 
#define DEBUG_MODULE "APPLEADER"

// define the ids of each node in the network
#define NETWORK_TOPOLOGY {.size = 4, .devices_ids = {0, 1, 2, 3} } // Maximum size of network is 20 by default
 
struct testPacketRX {
  int command;
} __attribute__((packed));
 
struct testPacketTX {
  float x;
  float y;
  float z;
} __attribute__((packed));

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

  DEBUG_PRINT("Waiting for activation ...\n");
 
  struct testPacketRX rxPacket;
  struct testPacketTX txPacket;
  static setpoint_t setpoint;

  logVarId_t idFlowX = logGetVarId("stateEstimate", "x");
  logVarId_t idFlowY = logGetVarId("stateEstimate", "y");
  logVarId_t idFlowZ = logGetVarId("stateEstimate", "z");
  paramVarId_t idFlowDeck = paramGetVarId("deck", "bcFlow2");

  while(1) {

    uint8_t flowDeckOn = paramGetUint(idFlowDeck);

    if(state==init) {
      // wait for start command from the computer 
      if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), APPCHANNEL_WAIT_FOREVER)) {
        
        // print out command 
        command = (int)rxPacket.command;
        DEBUG_PRINT("Command received: %d\n", command);
        
        if(command == start){
          // prepare the transmit packet and send back to computer
          if(flowDeckOn){
            txPacket.x = logGetFloat(idFlowX);
            txPacket.y = logGetFloat(idFlowY);
            txPacket.z = logGetFloat(idFlowZ);
            
            appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
          }
          state = standby;
          setHoverSetpoint(&setpoint, 0, 0, 0.5, 0);
          commanderSetSetpoint(&setpoint, 3);
        }
      }
    } else if(state == standby){
      // it seems delay is required from the crazyflie from crashing
      vTaskDelay(10);
      setHoverSetpoint(&setpoint, 0, 0, 0.5, 0);
      commanderSetSetpoint(&setpoint, 3);

      if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 0)) {
        command = (int)rxPacket.command;
        DEBUG_PRINT("Command received: %d\n", command);

        if(command == square){
          state = square;
          if(flowDeckOn){
            txPacket.x = logGetFloat(idFlowX);
            txPacket.y = logGetFloat(idFlowY);
            txPacket.z = logGetFloat(idFlowZ);
              
            appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket)); 
          }
        }
      }

    } else if(state == square_formation){ 
      vTaskDelay(10);
      setHoverSetpoint(&setpoint, 0.2, 0.2, 0.5, 0);
      commanderSetSetpoint(&setpoint, 3);

      if(flowDeckOn){
        txPacket.x = logGetFloat(idFlowX);
        txPacket.y = logGetFloat(idFlowY);
        txPacket.z = logGetFloat(idFlowZ);
          
        appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket)); 
      }


    } else{
      DEBUG_PRINT("nothing happening");
      vTaskDelay(10);
      memset(&setpoint, 0, sizeof(setpoint_t));
      commanderSetSetpoint(&setpoint, 3);
    }
  }
}
 