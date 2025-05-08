import argparse
import asyncio
import logging
import websockets
from enum import Enum
import time
import ast

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.syncLogger import SyncLogger

# python follower.py radio://1/70/2M/E7E7E7E7E2
# python follower.py radio://2/50/2M/E7E7E7E7E3
# python follower.py radio://3/60/2M/E7E7E7E7E4

# Only output errors from cflib
logging.basicConfig(level=logging.ERROR)

class Follower(Enum):
    FOLLOWER1 = 1
    FOLLOWER2 = 2
    FOLLOWER3 = 3

class State(Enum):
    INIT = 1
    TAKEOFF = 2
    LANDING = 3
    STANDBY = 4
    FORM_LINE = 5
    FORM_TRIANGLE = 6


current_state = State.INIT
latest_data = {}

# WebSocket URI for the leader server
WS_URI = None
current_follower = None

current_leader_x = None
current_leader_y = None
current_leader_z = None

form_line = False
form_triangle = False


def simple_log_setup(scf, log_config):
    cf = scf.cf
    cf.log.add_config(log_config)

    def log_callback(timestamp, data, logconf):
        global latest_data
        latest_data = data
        #print(f"[{timestamp}] Logged data: {data}")

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

async def listen_for_commands(scf):
    global current_state
    global current_leader_x
    global current_leader_y
    global current_leader_z

    async with websockets.connect(WS_URI) as websocket:
        print("Connected to WebSocket server at /ws/followers/id")

        # Start telemetry sending task
        telemetry_task = asyncio.create_task(telemetry_loop(websocket))

        while True:
            try:
                command = await websocket.recv()
                #print(f"Received : {command}")

                if command.lower() == "takeoff":
                    current_state = State.TAKEOFF
                elif command.lower() == "land":
                    current_state = State.LANDING
                elif command.lower() == "form_line":
                    current_state = State.FORM_LINE
                else:
                    #print("Unknown command")
                    try:
                        position_data = ast.literal_eval(command)  # Safely parse the data
                        if isinstance(position_data, dict):
                            # Check if the parsed data is a dictionary (safety check)
                            # Extract the position data in [x, y, z, roll, pitch, yaw] format
                            current_leader_x = position_data.get('x', 0)
                            current_leader_y = position_data.get('y', 0)
                            current_leader_z = position_data.get('z', 0)
                                
                        else:
                            print("Received data is not a dictionary. Ignoring.")
                    except Exception as e:
                        print(f"Error parsing data: {e}")
                        print(command)

                #await websocket.send(f"ACK: {current_state.name}")

            except websockets.ConnectionClosed:
                print("WebSocket connection closed")
                telemetry_task.cancel()
                break

async def state_machine_loop(commander):
    global current_state
    global current_leader_x
    global current_leader_y
    global current_leader_z

    while True:
        if current_state == State.TAKEOFF:
            print("[FSM] Taking off...")
            commander.take_off(0.5)
            current_state = State.STANDBY

        elif current_state == State.LANDING:
            print("[FSM] Landing...")
            commander.land()
            current_state = State.INIT

        elif current_state == State.STANDBY:
            print("[FSM] Standing by.")
            if form_line:
                commander.go_to(current_leader_x, current_leader_y - 0.2, current_leader_z)
            elif form_triangle:
                commander.go_to(current_leader_x, current_leader_y - 0.2, current_leader_z)
            await asyncio.sleep(0.1)
        
        elif current_state == State.FORM_LINE:
            print("[FSM] Form Line.")
            form_line = True
            form_triangle = False
            current_state = State.STANDBY
            
        elif current_state == State.FORM_TRIANGLE:
            print("[FSM] Form Triangle.")
            form_line = False
            form_triangle = True
            current_state = State.STANDBY

        await asyncio.sleep(0.1)

def reset_estimator(scf):
    scf.cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    scf.cf.param.set_value('kalman.resetEstimation', '0')
    _wait_for_position_estimator(scf.cf)


def _wait_for_position_estimator(cf):
    print('Waiting for estimator to find position...', end='\r')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(cf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                print('Waiting for estimator to find position...success!')
                break

def parse_arguments():
    parser = argparse.ArgumentParser(description="Connect to a Crazyflie using its URI.")
    parser.add_argument("uri", type=str, help="URI of the Crazyflie (e.g., radio://0/80/2M/E7E7E7E7E7)")
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_arguments()
    URI = args.uri

    match(URI):
        case "radio://1/70/2M/E7E7E7E7E2": 
            WS_URI = f"ws://127.0.0.1:8000/ws/followers/0"
            current_follower = Follower.FOLLOWER1
        case "radio://2/50/2M/E7E7E7E7E3":
            WS_URI = f"ws://127.0.0.1:8000/ws/followers/1"
            current_follower = Follower.FOLLOWER2
        case "radio://3/60/2M/E7E7E7E7E4":
            WS_URI = f"ws://127.0.0.1:8000/ws/followers/2"
            current_follower = Follower.FOLLOWER3

    # Initialize Crazyflie drivers
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')
    lg_stab.add_variable('stateEstimate.x', 'float')
    lg_stab.add_variable('stateEstimate.y', 'float')
    lg_stab.add_variable('stateEstimate.z', 'float')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        reset_estimator(scf)
        simple_log_setup(scf, lg_stab)
        

        follower = PositionHlCommander(scf)
        loop = asyncio.get_event_loop()
        tasks = asyncio.gather(
            listen_for_commands(scf),
            state_machine_loop(follower)
        )
        loop.run_until_complete(tasks)