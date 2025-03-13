/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
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
 * p2p_DTR_app.c - App layer application of simple demonstration of the 
 * Dynamic Token Ring Protocol used on top of the P2P.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"

#define DEBUG_MODULE "APPFOLLOWER"
#include "debug.h"

#include "token_ring.h"
#include "DTR_p2p_interface.h"

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

// define the ids of each node in the network
#define NETWORK_TOPOLOGY {.size = 4, .devices_ids = {0, 1, 2, 3} } // Maximum size of network is 20 by default

// store current id of drone in P2P DTR network
static uint8_t my_id;

// store defined network topology and ids in the network
static dtrTopology topology = NETWORK_TOPOLOGY;

// store leader drone current position
static float leaderX;
static float leaderY;
static float leaderZ;

// function to load transmit packet so it transmit current x, y, z of follower drone
void loadTXPackets(float x, float y, float z){
	dtrPacket transmitSignal;
	transmitSignal.messageType = DATA_FRAME;
	transmitSignal.sourceId = my_id;

	// create a array of 3 floats and copy it to transmit packet data
	float followerPos[3] = {x, y, z};
	memcpy(transmitSignal.data, followerPos, sizeof(followerPos));
	
	transmitSignal.dataSize = sizeof(followerPos);
	// transmit to leader drone, id of 0 (first to join network)
	transmitSignal.targetId = 0;
	transmitSignal.packetSize = DTR_PACKET_HEADER_SIZE + transmitSignal.dataSize;

	bool res;
	res = dtrSendPacket(&transmitSignal);
	if (res){
		DTR_DEBUG_PRINT("Packet sent to DTR protocol\n");
	}
	else{
		DEBUG_PRINT("Packet not sent to DTR protocol\n");
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

void appMain(){
	
	// set self id for network
	my_id = dtrGetSelfId();

	// enable P2P DTR network
	dtrEnableProtocol(topology);

	vTaskDelay(2000);

	// register the callback function so that the CF can receive packets as well
	p2pRegisterCB(p2pcallbackHandler);

	dtrPacket received_packet;
	uint32_t start = T2M(xTaskGetTickCount());
	while(1){
		dtrGetPacket(&received_packet, portMAX_DELAY);
		uint32_t dt = T2M(xTaskGetTickCount()) - start;

		// uint8_t array to string conversion
		char data[received_packet.dataSize + 1];
		for (int i = 0; i < received_packet.dataSize; i++){
			data[i] = received_packet.data[i];
		}
		data[received_packet.dataSize] = '\0';
		
		DEBUG_PRINT("Received data from %d : %s  --> Time elapsed: %lu msec\n",received_packet.sourceId, data, dt);
		start = T2M(xTaskGetTickCount());


		if (strcmp(data, "Hello World") == 0){
			loadResponse();
		}
	}
}
