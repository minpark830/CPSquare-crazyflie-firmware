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
 
 typedef enum {
    square,
    rhombus,
    triangle
  } Command;
    
  
   typedef enum {
    leader,
    follower_1,
    follower_2,
    follower_3
  } currentDrone;

 void initDesiredPosition(currentDrone commandouter, Command positionouter);
 
 void calculatePosition(float position_leader_x, float position_leader_y, float leader_yaw, 
    float *yaw, float *mylocation_x,float *mylocation_y, float timeNow);
 #endif /* SRC_WALLFOLLOWING_MULTIRANGER_ONBOARD_H_ */
 