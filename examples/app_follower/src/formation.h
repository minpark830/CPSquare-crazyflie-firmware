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
 
 #define LEADER_ID 231
 #define FOLLOWER_1_ID 232
 #define FOLLOWER_2_ID 230
 #define FOLLOWER_3_ID 233
  
 typedef enum {
    squareInner,
    rhombusInner,
    triangleInner
  } Commandouter;
    
  


 void initDesiredPosition(Commandouter commandouter, int my_id);
 
 void calculatePosition(float position_leader_x, float position_leader_y, float *mylocation_x, float *mylocation_y, float timeNow);
 #endif /* SRC_WALLFOLLOWING_MULTIRANGER_ONBOARD_H_ */
 