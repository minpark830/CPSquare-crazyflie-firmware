from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
import uvicorn
import ast
import json
import os

app = FastAPI()

app.mount("/static", StaticFiles(directory=os.path.join(os.path.dirname(__file__), "static")), name="static")

connected_followers = {}
connected_leader = None

follower_1_positions = []
follower_2_positions = []
leader_positions = []

@app.get("/", response_class=HTMLResponse)
async def get_positions_page():
    html_content = """
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Positions of Leader and Followers</title>
        <script src="https://cdn.jsdelivr.net/npm/plotly.js-dist@2.17.0"></script>
    </head>
    <body>
        <h1>CPSquare Crazyflie Experimental Framework</h1>
        <div id="positionsChart" style="width: 100%; height: 100%;"></div>
        <h2>🎮 Leader Control Panel</h2>
        <div class="button-grid">
            <button onclick="sendCommand('takeoff')">🚀 Take Off</button>
            <button onclick="sendCommand('land')">🛬 Land</button>
            <button onclick="sendCommand('right')">➡️ Right</button>
            <button onclick="sendCommand('left')">⬅️ Left</button>
            <button onclick="sendCommand('forward')">⬆️ Forward</button>
            <button onclick="sendCommand('back')">⬇️ Back</button>
            <button onclick="sendCommand('up')">🔼 Up</button>
            <button onclick="sendCommand('down')">🔽 Down</button>
            <button onclick="sendCommand('form_line')">📏 Form Line</button>
            <button onclick="sendCommand('form_triangle')">🔺Form Triangle</button>
        </div>

        <div class="logo">
            <img src="/static/cpsquare-logo.png" alt="CPSquare Logo" width="75">
        </div>

        <b>&copy; Copyright of CPSquare Lab 2025</b>

        <script>
            function sendCommand(cmd) {
                fetch(`/commands/${cmd}`, {
                    method: "POST"
                })
                .then(response => response.text())
                .then(result => {
                    alert(result);
                    console.log("Command response:", result);
                })
                .catch(error => {
                    console.error("Error sending command:", error);
                    alert("Failed to send command.");
                });
            }
        </script>

        <script>
            // Initial empty data for all positions
            var traceLeader = {
                x: [],
                y: [],
                z: [],
                mode: 'markers+lines',
                type: 'scatter3d',
                name: 'Leader',
                marker: {color: 'blue'}
            };

            var traceFollower1 = {
                x: [],
                y: [],
                z: [],
                mode: 'markers+lines',
                type: 'scatter3d',
                name: 'Follower 1',
                marker: {color: 'red'}
            };

            var traceFollower2 = {
                x: [],
                y: [],
                z: [],
                mode: 'markers+lines',
                type: 'scatter3d',
                name: 'Follower 2',
                marker: {color: 'green'}
            };

            var layout = {
                title: 'Positions of Leader and Followers',
                scene: {
                    xaxis: {title: 'X Position [m]'},
                    yaxis: {title: 'Y Position [m]'},
                    zaxis: {title: 'Z Position [m]'}
                }
            };

            // Initialize plot with empty data
            Plotly.newPlot('positionsChart', [traceLeader, traceFollower1, traceFollower2], layout);

            // Function to fetch and update chart with new data
            function refreshChart() {
                fetch('/positions')
                    .then(response => response.json())
                    .then(data => {
                        var leaderPositions = data.leader_positions;
                        var follower1Positions = data.follower1_positions;
                        var follower2Positions = data.follower2_positions;

                        // Update the chart with the new positions
                        var xLeader = leaderPositions.map(pos => pos[0]);
                        var yLeader = leaderPositions.map(pos => pos[1]);
                        var zLeader = leaderPositions.map(pos => pos[2]);

                        var xFollower1 = follower1Positions.map(pos => pos[0]);
                        var yFollower1 = follower1Positions.map(pos => pos[1]);
                        var zFollower1 = follower1Positions.map(pos => pos[2]);

                        var xFollower2 = follower2Positions.map(pos => pos[0]);
                        var yFollower2 = follower2Positions.map(pos => pos[1]);
                        var zFollower2 = follower2Positions.map(pos => pos[2]);

                        // Update the chart with the new positions
                        Plotly.update('positionsChart', {
                            x: [xLeader, xFollower1, xFollower2],
                            y: [yLeader, yFollower1, yFollower2],
                            z: [zLeader, zFollower1, zFollower2]
                        });
                    })
                    .catch(error => console.error('Error fetching positions:', error));
            }

            // Refresh the chart every 2 seconds
            setInterval(refreshChart, 2000);
        </script>
    </body>
    </html>
    """
    return HTMLResponse(content=html_content)

@app.get("/positions", response_class=JSONResponse)
async def get__positions():
    return {
        "leader_positions": leader_positions,
        "follower1_positions": follower_1_positions,
        "follower2_positions": follower_2_positions
    }

@app.websocket("/ws/leader")
async def websocket_endpoint(websocket: WebSocket):
    global connected_leader
    await websocket.accept()
 
    connected_leader = websocket
    try:
        while True:
            data = await websocket.receive_text()
            #print(f"Received from client: {data}")
            try:
                position_data = ast.literal_eval(data)  # Safely parse the data
                if isinstance(position_data, dict):
                    # Check if the parsed data is a dictionary (safety check)
                    # Extract the position data in [x, y, z, roll, pitch, yaw] format
                    position = [
                        position_data.get('x', 0),
                        position_data.get('y', 0),
                        position_data.get('z', 0),
                        position_data.get('roll', 0),
                        position_data.get('pitch', 0),
                        position_data.get('yaw', 0)
                    ]
                    leader_positions.append(position)  # Store the position as a list
                    print(f"Stored telemetry data: {position}")
                else:
                    print("Received data is not a dictionary. Ignoring.")
            except Exception as e:
                broadcast_to_followers(data)
                print(f"Broadcast to followers: {data}")
                print(f"Error parsing data: {e}")
    except WebSocketDisconnect:
        connected_leader = None
        print("Client disconnected")


@app.websocket("/ws/followers/{id}")
async def websocket_endpoint(websocket: WebSocket, id: str):
    await websocket.accept()

    connected_followers[id] = websocket
    try:
        while True:
            data = await websocket.receive_text()
            try:
                position_data = ast.literal_eval(data)  # Safely parse the data
                if isinstance(position_data, dict):
                    # Check if the parsed data is a dictionary (safety check)
                    # Extract the position data in [x, y, z, roll, pitch, yaw] format
                    position = [
                        position_data.get('x', 0),
                        position_data.get('y', 0),
                        position_data.get('z', 0),
                        position_data.get('roll', 0),
                        position_data.get('pitch', 0),
                        position_data.get('yaw', 0)
                    ]
                    match id:
                        case "0":
                            follower_1_positions.append(position)  # Store the position as a list
                            print(f"Stored telemetry data: {position}")
                        case "1":
                            follower_2_positions.append(position)  # Store the position as a list
                            print(f"Stored telemetry data: {position}")
                else:
                    print("Received data is not a dictionary. Ignoring.")
            except Exception as e:
                print(f"Error parsing data: {e}")
            # constantly send most updated leader position
            if leader_positions:
                latest_position = leader_positions[-1]
                message = json.dumps({
                    "x": latest_position[0],
                    "y": latest_position[1],
                    "z": latest_position[2],
                    "roll": latest_position[3],
                    "pitch": latest_position[4],
                    "yaw": latest_position[5],
                })
                await websocket.send_text(message)
    except WebSocketDisconnect:
        connected_followers.pop(id, None)
        print("Client disconnected")

async def broadcast_to_followers(message: str):
    for websocket in connected_followers.values():
        await websocket.send_text(message)


@app.post("/commands/{command}")
async def send_command(request: Request, command: str):
    global connected_leader

    if connected_leader:
        await connected_leader.send_text(command)
        return f"Sent command to leader: {command}"
    else:
        return "No leader connected."


if __name__ == "__main__":
    uvicorn.run("server:app", host="127.0.0.1", port=8000, reload=True)