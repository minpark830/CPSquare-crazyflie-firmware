/*
 * wall_follower_multi_ranger_onboard.c
 *
 *  Created on: Aug 7, 2018
 *      Author: knmcguire

The same wallfollowing strategy was used in the following paper:

 @article{mcguire2019minimal,
  title={Minimal navigation solution for a swarm of tiny flying robots to explore an unknown environment},
  author={McGuire, KN and De Wagter, Christophe and Tuyls, Karl and Kappen, HJ and de Croon, Guido CHE},
  journal={Science Robotics},
  volume={4},
  number={35},
  year={2019},
  publisher={Science Robotics}

 */

 #include "formation.h"
 #include <math.h>
 
 // variables
 
 // PID controller variables
 static float dt = 0.0; // Time step 
 float k_p = 0.9; // Proportional gain
 float Ki = 0.5;  // Integral gain
 float Kd = 0.01; // Derivative gain
 float integral_error_x = 0.0;
 float integral_error_y = 0.0;
 float integral_error_yaw = 0.0;
 float previous_error_x = 0.0;
 float previous_error_y = 0.0;
 float previous_error_yaw = 0.0;
 float prevous_time = 0.0;
 float min_distance = 0.5;
 float relative_position_x = 0.0;
 float relative_position_y = 0.0;
 
 
void initDesiredPosition(Commandouter commandouter, int my_id){
  switch (commandouter) {
    case squareInner:
      switch (my_id) {
        case FOLLOWER_1_ID:
          relative_position_x = 0.2f;
          relative_position_y = 0.0;
        break;
        case FOLLOWER_2_ID:
          relative_position_x = 0.0;
          relative_position_y = -0.2f;
        break;
        case FOLLOWER_3_ID:
          relative_position_x = 0.2f;
          relative_position_y = -0.2f;
        break;
        default:
          DEBUG_PRINT("NO VALID COMMAND SENT, AT STATE: standby\n");
      }
      break;
    case rhombusInner:
      switch (my_id) {
        case FOLLOWER_1_ID:
          relative_position_x = 0.2f;
          relative_position_y = 0.0;
        break;
        case FOLLOWER_2_ID:
          relative_position_x = 0.0;
          relative_position_y = -0.2f;
        break;
        case FOLLOWER_3_ID:
          relative_position_x = 0.2f;
          relative_position_y = -0.2f;
        break;
        default:
          DEBUG_PRINT("NO VALID COMMAND SENT, AT STATE: standby\n");
      }
      break;
    case triangleInner:
      switch (my_id) {
        case FOLLOWER_1_ID:
          relative_position_x = 0.2f;
          relative_position_y = 0.0;
        break;
        case FOLLOWER_2_ID:
          relative_position_x = 0.0;
          relative_position_y = -0.2f;
        break;
        case FOLLOWER_3_ID:
          relative_position_x = 0.2f;
          relative_position_y = -0.2f;
        break;
        default:
          DEBUG_PRINT("NO VALID COMMAND SENT, AT STATE: standby\n");
      }
      break;
    default:
      DEBUG_PRINT("NO VALID COMMAND SENT, AT STATE: standby\n");
  }
}
 
void calculatePosition(float position_leader_x, float position_leader_y, float *mylocation_x, float *mylocation_y, float timeNow)
 {
    dt = timeNow - prevous_time;
    float desired_position_x = position_leader_x + relative_position_x;
    float error_x = desired_position_x - *mylocation_x;
    // float distance_x = position_leader_x- *mylocation_x;
    // Maintain minimum distance
    // if (distance_x < min_distance){
    //     float scale = 0.075;
    //     error_x = error_x * (1/ (distance_x*scale));
    // }
    // PID control
    integral_error_x += integral_error_x * dt;
    float derivative_error_x = (error_x - previous_error_x) / dt;
    float control_input_x = k_p * error_x + Ki * integral_error_x + Kd * derivative_error_x;
    *mylocation_x += (control_input_x * dt);
    previous_error_x = error_x;

    float desired_position_y = position_leader_y + relative_position_y;
    float error_y = desired_position_y - *mylocation_y;
    // float distance_y = position_leader_y - *mylocation_y;
    // Maintain minimum distance
    // if (distance_y < min_distance){
    //     float scale = 0.075;
    //     error_y = error_y * (1/ (distance_y*scale));
    // }
    // PID control
    integral_error_y = integral_error_y + integral_error_y * dt;
    float derivative_error_y = (error_y - previous_error_y) / dt;
    float control_input_y = k_p * error_y + Ki * integral_error_y + Kd * derivative_error_y;
    *mylocation_y += (control_input_y * dt);
    previous_error_y = error_y;

    // float desired_position_yaw = leader_yaw;
    // float error_yaw = desired_position_yaw - *yaw;
    
    // // PID control
    // integral_error_yaw = integral_error_yaw + integral_error_yaw * dt;
    // float derivative_error_yaw = (error_yaw - previous_error_yaw) / dt;
    // float control_input_yaw = k_p * error_yaw + Ki * integral_error_yaw + Kd * derivative_error_yaw;
    // *yaw += (control_input_yaw * dt);
    // previous_error_yaw = error_yaw;

    prevous_time = timeNow;
 }
