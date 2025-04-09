import tkinter as tk
import cflib
from cflib.crazyflie import Crazyflie
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np
import struct
from threading import Thread
import time
import os
from PIL import Image, ImageTk    

class App:
    
    def __init__(self, root):
        self.cf = Crazyflie()
        self.cf.connected.add_callback(self.connected)
        self.cf.disconnected.add_callback(self.disconnected)
        self.cf.connection_failed.add_callback(self.connection_failed)
        self.cf.connection_lost.add_callback(self.connection_lost)

        self.cf.appchannel.packet_received.add_callback(self.app_packet_received)

        self.currentLeader = "None"
        self.receivedData = False
        self.firstTime = True

        self.root = root
        self.root.title("Bitcraze Experimental Framework")
        start_path = os.path.join(os.getcwd(), "examples", "app_leader", "tools", "cpsquare-logo.png")
        print(start_path)
        
        img = Image.open(start_path)  # Replace with the path to your image
        
        # Rescale the image (change size to 300x300 pixels)
        img_resized = img.resize((150, 150))  # Replace (300, 300) with your desired size
        
        # Convert the resized image to a format Tkinter can use
        img_tk = ImageTk.PhotoImage(img_resized)
    
        # Create a label to hold the image
        logo_label = tk.Label(root, image=img_tk)
        logo_label.pack(pady=20)

        # IMPORTANT DONT DELETE need as reference for 
        root.img = img_tk

        self.leader_label = tk.Label(root, text=f"Current Leader: {self.currentLeader}", font=("Arial", 14))
        self.leader_label.pack()

        # Leader Buttons
        leader_frame = tk.Frame(root)
        leader_frame.pack(pady=20)

        connect_button = tk.Button(leader_frame, text="Connect Leader", command=self.connect_to_crazyflie)
        connect_button.pack(side='left', padx=10)

        land_button = tk.Button(leader_frame, text="Land Leader", command=self.land_crazyflie)
        land_button.pack(side='left', padx=10)

        disconnect_button = tk.Button(leader_frame, text="Disconnect Leader", command=self.disconnect_crazyflie)
        disconnect_button.pack(side='left', padx=10)

        command_label = tk.Label(root, text=f"Available Commands", font=("Arial", 14))
        command_label.pack()
        
        # Commands
        command_frame = tk.Frame(root)
        command_frame.pack(pady=20)

        start_button = tk.Button(command_frame, text="Send Start Command", command=self.sendStartCommand)
        start_button.pack(side='left', padx=10)

        square_form_button = tk.Button(command_frame, text="Send Square Formation Command", command=self.sendSquareCommand)
        square_form_button.pack(side='left', padx=10)

        rhombus_form_button = tk.Button(command_frame, text="Send Rhombus Formation Command", command=self.sendSquareCommand)
        rhombus_form_button.pack(side='left', padx=10)

        triangle_form_button = tk.Button(command_frame, text="Send Triangle Formation Command", command=self.sendSquareCommand)
        triangle_form_button.pack(side='left', padx=10)

        control_label = tk.Label(root, text=f"Available Controls", font=("Arial", 14))
        control_label.pack()
        
        # Controls
        control_frame = tk.Frame(root)
        control_frame.pack(pady=20)

        control_font = ("Arial", 15)  # Big font for arrow symbols
        control_size = 4             # Width and height in character units

        # Up Arrow
        up_button = tk.Button(control_frame, text="↑", font=control_font, width=control_size)
        up_button.grid(row=0, column=1, padx=10, pady=10)

        # Left Arrow
        left_button = tk.Button(control_frame, text="←", font=control_font, width=control_size)
        left_button.grid(row=1, column=0, padx=10, pady=10)

        # Right Arrow
        right_button = tk.Button(control_frame, text="→", font=control_font, width=control_size)
        right_button.grid(row=1, column=2, padx=10, pady=10)

        # Down Arrow
        down_button = tk.Button(control_frame, text="↓", font=control_font, width=control_size)
        down_button.grid(row=2, column=1, padx=10, pady=10)

        # Optional: Center Stop Button
        stop_button = tk.Button(control_frame, text="■", font=control_font, width=control_size)
        stop_button.grid(row=1, column=1)


        # Create a Matplotlib figure
        # self.fig, self.ax = plt.subplots()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X Axis [m]')
        self.ax.set_ylabel('Y Axis [m]')
        self.ax.set_zlabel('Z Axis [m]')

        # Create a canvas widget for the figure
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.draw()

        # Place the canvas in the Tkinter window
        self.canvas.get_tk_widget().pack()

    
    def connected(self, uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!

    def connection_failed(self, uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (uri, msg))

    def connection_lost(self, uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (uri, msg))

    def disconnected(self, uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""

    def app_packet_received(self, data):
        (x, y, z) = struct.unpack("<fff", data)
        self.receivedData = True
        self.ax.scatter(x,y,z, marker = 'x', color='b', label='leader')
        if(self.firstTime):
            self.ax.legend()
            self.firstTime = False
        self.canvas.draw()
        print(f"Received coordinates: {x}, {y}, {z}")

    def connect_to_crazyflie(self):
        # Initialize the low-level drivers
        cflib.crtp.init_drivers(enable_debug_driver=False)
        available = cflib.crtp.scan_interfaces()
        print('Crazyflies found:')
        for i in available:
            print(i[0])

        
        if len(available) > 0:
            # Start the connection to the first available Crazyflie
            print(f"Connecting to {available[0][0]} ")
            self.currentLeader = available[0][0]
            self.leader_label.config(text=f"Current Leader: {self.currentLeader}")
            self.cf.open_link(available[0][0])
        # if (1):
        #     self.cf.open_link('radio://0/80/2M/E7E7E7E7E8')
        else:
            print("Error", "No Crazyflies found, cannot run example")

    def disconnect_crazyflie(self):
        self.currentLeader = "None"
        self.leader_label.config(text=f"Current Leader: {self.currentLeader}")
        self.cf.close_link()

    def land_crazyflie(self):
        self.receivedData = False
        self.send_command(3)
        print("Sent Land Command")
        
    def send_command(self, command):
        data = struct.pack("<i", command)
        self.cf.appchannel.send_packet(data)

    def sendStartCommand(self):
        
        self.receivedData = False

        while(not self.receivedData):
            self.send_command(1)
            print("Sent Start command")
            time.sleep(1)
    
    def sendSquareCommand(self):
        self.send_command(2)
        print("Sent Square Formation command")

if __name__ == '__main__':
    root = tk.Tk()
    app = App(root)
    root.mainloop()