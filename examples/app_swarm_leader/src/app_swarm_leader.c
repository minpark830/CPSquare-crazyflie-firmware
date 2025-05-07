#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "radiolink.h"
#include "configblock.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "crtp_commander_high_level.h"
#include "commander.h"
#include "pptraj.h"
#include "estimator_kalman.h"
#include "deck_analog.h"
#include "deck_constants.h"

#define DEBUG_MODULE "SWARM_LEADER"

paramVarId_t idCommanderEnHighLevel;
paramVarId_t idResetKalman;

static void resetKalman() { 
  paramSetInt(idResetKalman, 1); 
}
static void enableHighlevelCommander() {
  paramSetInt(idCommanderEnHighLevel, 1); 
}

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawRate)
{

  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;

  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->mode.yaw = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;
  setpoint->attitudeRate.yaw = yawRate;
  setpoint->velocity_body = true;
}


// State machine
typedef enum {
  idle,
  takeOff,
  standBy,
  land,
} State;

static State state = idle;

void appMain()
{

  static setpoint_t setpoint;

  // Setting Ids for logging
  logVarId_t idX = logGetVarId("stateEstimate", "x");
  logVarId_t idY = logGetVarId("stateEstimate", "y");
  logVarId_t idZ = logGetVarId("stateEstimate", "z");
  logVarId_t idYaw = logGetVarId("stateEstimate", "yaw");

  static uint8_t command_takeoff;
  static uint8_t command_land;

  command_takeoff = 0;
  command_land = 0;
  
  static float x_log;  
  static float y_log;
  static float z_log;
  static float yaw_log;
  static uint8_t my_id; 

  static P2PPacket p_reply;
  p_reply.port=0x00;
  uint64_t address = configblockGetRadioAddress();
  my_id =(uint8_t)((address) & 0x00000000ff);

  LOG_GROUP_START(pose)
  LOG_ADD_CORE(LOG_FLOAT, drone_x, &x_log)
  LOG_ADD_CORE(LOG_FLOAT, drone_y, &y_log)
  LOG_ADD_CORE(LOG_FLOAT, drone_z, &z_log)
  LOG_ADD_CORE(LOG_FLOAT, drone_yaw, &yaw_log)
  LOG_ADD_CORE(LOG_UINT8, drone_id, &my_id)
  LOG_GROUP_STOP(pose)

  PARAM_GROUP_START(commands)
  PARAM_ADD_CORE(PARAM_UINT8, takeoff, &command_takeoff)
  PARAM_ADD_CORE(PARAM_UINT8, land, &command_land)
  PARAM_GROUP_STOP(commands)

  idCommanderEnHighLevel = paramGetVarId("commander", "enHighLevel");
  idResetKalman = paramGetVarId("kalman", "resetEstimation");
  paramVarId_t idCommandTakeOff = paramGetVarId("commands", "takeoff");
  paramVarId_t idCommandLand = paramGetVarId("commands", "land"); 

  resetKalman();
  enableHighlevelCommander();
  vTaskDelay(M2T(8000));

  while(1) {
    // constantly update x, y, z, and yaw
    x_log = logGetFloat(idX);
    y_log = logGetFloat(idY);
    z_log = logGetFloat(idZ);
    yaw_log = logGetFloat(idYaw);

    if (state == idle) {

      if (command_takeoff == 0) {
        command_takeoff = paramGetInt(idCommandTakeOff);
        vTaskDelay(M2T(100));
      }

      if (command_takeoff == 1) {
        vTaskDelay(M2T(1000));
        crtpCommanderHighLevelTakeoff(0.5, 2.5);
        vTaskDelay(M2T(4000));
        state = takeOff;
      }

    } 
    else if (state == takeOff) {
      if(crtpCommanderHighLevelIsTrajectoryFinished()){
        state = standBy;
      }
    } 
    else if (state == standBy) {
      if (command_land == 0) {
        command_land = paramGetInt(idCommandLand);
        vTaskDelay(M2T(100));
      }

      if (command_land == 1) {
        vTaskDelay(M2T(1000));
        state = land;
      }

      setHoverSetpoint(&setpoint, 0, 0, 0.5f, 0);
      commanderSetSetpoint(&setpoint, 3);
    }  
    else if (state == land) {
      crtpCommanderHighLevelLand(0.03, 3.0); 
      vTaskDelay(M2T(2000));
      return;
    }

  }
}
