/*
 * wallfollowing_multirange_onboard.h
 *
 *  Created on: Aug 7, 2018
 *      Author: knmcguire
 */

 #ifndef SRC_WALLFOLLOWING_MULTIRANGER_ONBOARD_H_
 #define SRC_WALLFOLLOWING_MULTIRANGER_ONBOARD_H_
 #include <stdint.h>
 #include <stdbool.h>
 
 typedef enum
 {
     forward,
     hover,
     turnToFindWall,
     turnToAlignToWall,
     forwardAlongWall,
     rotateAroundWall,
     rotateInCorner,
     findCorner
 } StateWF;
 
 StateWF wallFollower(float *cmdVelX, float *cmdVelY, float *cmdAngW, float frontRange, float sideRange, float currentHeading,
    int directionTurn, float timeOuter, float position_leader_x, float position_leader_y,float relative_positions_x,
    float relative_positions_y, float leader_yaw, float yaw);
 
static void commandForwardInFormationVelocity(float position_leader_x, float position_leader_y, float leader_yaw, 
    float *yaw, float *mylocation_x,float *mylocation_y);
 
 void wallFollowerInit(float refDistanceFromWallNew, float maxForwardSpeed_ref, StateWF initState);
 #endif /* SRC_WALLFOLLOWING_MULTIRANGER_ONBOARD_H_ */
 