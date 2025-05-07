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
#include "crtp_commander_high_level.h"
#include "pptraj.h"
#include "deck_analog.h"
#include "deck_constants.h"

// defining constants 
#define DEBUG_MODULE "SWARMLEADER"

#define LEADER_ID 230
#define FOLLOWER_1_ID 231

#define START 1
#define SEND_DATA 2
#define LAND 3
#define SQUARE_FORM 4

// define received command packet for app channel
typedef struct appChannelRX_command_s {
  int command;
} __attribute__((packed)) appChannelRXCommand;

// define transmit data packet for app channel
typedef struct appChannelTX_data {
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
} __attribute__((packed)) appChannelTXData;

// define transmit command packet for app channel
typedef struct appChannelTX_command {
  int command;
} __attribute__((packed)) appChannelTXCommand;

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

// // function to define velocity hover setpoint using velocity values
// static void setVelocityHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawRate)
// {

//   setpoint->mode.z = modeAbs;
//   setpoint->position.z = z;

//   setpoint->mode.x = modeVelocity;
//   setpoint->mode.y = modeVelocity;
//   setpoint->mode.yaw = modeVelocity;
//   setpoint->velocity.x = vx;
//   setpoint->velocity.y = vy;
//   setpoint->attitudeRate.yaw = yawRate;
//   setpoint->velocity_body = true;
// }

// define transmit data packet function based off positional values (FlowX, FlowY, FlowZ, FlowRoll, FlowPitch, FlowYaw)
static void transmitData(appChannelTXData *txPacket, float x, float y, float z, float roll, float pitch, float yaw){
  txPacket->x = x;
  txPacket->y = y;
  txPacket->z = z;
  txPacket->roll = roll;
  txPacket->pitch = pitch;
  txPacket->yaw = yaw;

  appchannelSendDataPacketBlock(txPacket, sizeof(*txPacket)); 
  
}

// define transmit command packet function based off command value 
static void transmitCommand(appChannelTXCommand *txPacket, int command){
  txPacket->command = command;

  appchannelSendDataPacketBlock(txPacket, sizeof(*txPacket)); 
  
}

typedef enum {
  init,
  takingOff,
  standBy
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

// leader starts in init state
static State state = init;

// leader starts in nothing command
static Command command = nothing;

void appMain() {

  vTaskDelay(M2T(3000));
 
  static appChannelRXCommand rxPacket;
  static appChannelTXData txPacketData;
  static appChannelTXCommand txPacketCommand;
  
  static setpoint_t setpoint;

  logVarId_t idFlowX = logGetVarId("stateEstimate", "x");
  logVarId_t idFlowY = logGetVarId("stateEstimate", "y");
  logVarId_t idFlowZ = logGetVarId("stateEstimate", "z");
  logVarId_t idFlowRoll = logGetVarId("stateEstimate", "roll");
  logVarId_t idFlowPitch = logGetVarId("stateEstimate", "pitch");
  logVarId_t idFlowYaw = logGetVarId("stateEstimate", "yaw");

  estimatorKalmanInit();
  vTaskDelay(3000);

  while(1) {


    if (state == init) {

      DEBUG_PRINT("Current State: init\n");

      // wait for start command from the computer 
      // note it is important to keep this one as APPCHANNEL_WAIT_FOREVER for some reason
      if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), APPCHANNEL_WAIT_FOREVER)) {

        command = (int)rxPacket.command;

        // reset kalman filter
        estimatorKalmanInit();

        switch (command) {
          case start:
            state = standBy;
            setHoverSetpoint(&setpoint, 0, 0, 0.5, 0);
            commanderSetSetpoint(&setpoint, 3);
            transmitCommand(&txPacketCommand, START);
            break;
          default:
            DEBUG_PRINT("NO VALID COMMAND SENT FOR STATE: init\n");
        }
      
      }

    } else if (state == standBy) {
      vTaskDelay(10);
      setHoverSetpoint(&setpoint, 0, 0, 0.5, 0);
      commanderSetSetpoint(&setpoint, 3);
      transmitData(&txPacketData, logGetFloat(idFlowX), logGetFloat(idFlowY), logGetFloat(idFlowZ), logGetFloat(idFlowRoll), logGetFloat(idFlowPitch), logGetFloat(idFlowYaw));  

    } else {

      DEBUG_PRINT("ERROR WITH STATE HANDLING\n");

      vTaskDelay(10);
      memset(&setpoint, 0, sizeof(setpoint_t));
      commanderSetSetpoint(&setpoint, 3);
    }
  }
}
 