import requests
import time

API_URL = "http://127.0.0.1:8000/leader/position"

def poll_leader_position(interval=2):
    print(f"Polling {API_URL} every {interval} seconds. Press Ctrl+C to stop.\n")
    while True:
        try:
            response = requests.get(API_URL)
            response.raise_for_status()
            data = response.json()

            if not data:
                print("[No leader position data yet]")
            else:
                latest = data[-1]
                print(f"Latest Leader Position - X: {latest[0]:.2f}, Y: {latest[1]:.2f}, Z: {latest[2]:.2f}")
        except requests.exceptions.RequestException as e:
            print(f"Error fetching data: {e}")
        
        time.sleep(interval)

if __name__ == "__main__":
    poll_leader_position()