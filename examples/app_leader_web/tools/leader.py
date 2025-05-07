import argparse
import asyncio
import logging
import websockets
from enum import Enum
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.position_hl_commander import PositionHlCommander

# python leader.py radio://0/0/2M/E7E7E7E7E1

# Only output errors from cflib
logging.basicConfig(level=logging.ERROR)

class State(Enum):
    INIT = 1
    TAKEOFF = 2
    LANDING = 3
    STANDBY = 4
    RIGHT = 5
    LEFT = 6
    FORWARD = 7
    BACK = 8
    UP = 9
    DOWN = 10


current_state = State.INIT
latest_data = {}

# WebSocket URI for the leader server
WS_URI = "ws://127.0.0.1:8000/ws/leader"

def simple_log_setup(scf, log_config):
    cf = scf.cf
    cf.log.add_config(log_config)

    def log_callback(timestamp, data, logconf):
        global latest_data
        latest_data = data
        print(f"[{timestamp}] Logged data: {data}")

    log_config.data_received_cb.add_callback(log_callback)

    if log_config.valid:
        log_config.start()
        print("Logging started.")
    else:
        print("Logging config invalid or missing variables.")

async def telemetry_loop(websocket):
    """Send telemetry data periodically to WebSocket server"""
    while True:
        if latest_data:
            message = {
                "state": current_state.name,
                "x": latest_data.get("stateEstimate.x", 0),
                "y": latest_data.get("stateEstimate.y", 0),
                "z": latest_data.get("stateEstimate.z", 0),
                "roll": latest_data.get("stabilizer.roll", 0),
                "pitch": latest_data.get("stabilizer.pitch", 0),
                "yaw": latest_data.get("stabilizer.yaw", 0),
            }
            await websocket.send(str(message))
        await asyncio.sleep(0.1)
    
def reset_estimator(scf):
    scf.cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)

async def listen_for_commands(scf):
    global current_state

    async with websockets.connect(WS_URI) as websocket:
        print("Connected to WebSocket server at /ws/leader")

        # Start telemetry sending task
        telemetry_task = asyncio.create_task(telemetry_loop(websocket))

        while True:
            try:
                command = await websocket.recv()
                print(f"Received command: {command}")

                if command.lower() == "takeoff":
                    current_state = State.TAKEOFF
                elif command.lower() == "land":
                    current_state = State.LANDING
                elif command.lower() == "standby":
                    current_state = State.STANDBY
                else:
                    print("Unknown command")

                await websocket.send(f"ACK: {current_state.name}")

            except websockets.ConnectionClosed:
                print("WebSocket connection closed")
                telemetry_task.cancel()
                break

async def state_machine_loop(commander, scf):
    global current_state

    while True:
        if current_state == State.TAKEOFF:
            reset_estimator(scf)
            print("[FSM] Taking off...")
            commander.take_off(0.5)
            current_state = State.STANDBY

        elif current_state == State.LANDING:
            print("[FSM] Landing...")
            commander.land()
            current_state = State.INIT

        elif current_state == State.STANDBY:
            print("[FSM] Standing by.")
            await asyncio.sleep(0.1)
        
        elif current_state == State.RIGHT:
            print("[FSM] To the Right.")
            commander.right(0.1)
        
        elif current_state == State.LEFT:
            print("[FSM] To the Left.")
            commander.left(0.1)

        await asyncio.sleep(0.1)

def parse_arguments():
    parser = argparse.ArgumentParser(description="Connect to a Crazyflie using its URI.")
    parser.add_argument("uri", type=str, help="URI of the Crazyflie (e.g., radio://0/80/2M/E7E7E7E7E7)")
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_arguments()
    URI = args.uri

    # Initialize Crazyflie drivers
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=200)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')
    lg_stab.add_variable('stateEstimate.x', 'float')
    lg_stab.add_variable('stateEstimate.y', 'float')
    lg_stab.add_variable('stateEstimate.z', 'float')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        simple_log_setup(scf, lg_stab)
        reset_estimator(scf)

        leader = PositionHlCommander(scf)
        loop = asyncio.get_event_loop()
        tasks = asyncio.gather(
            listen_for_commands(scf),
            state_machine_loop(leader, scf)
        )
        loop.run_until_complete(tasks)