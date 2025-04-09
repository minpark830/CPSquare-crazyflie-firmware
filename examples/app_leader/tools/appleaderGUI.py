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
        self.leader_label.pack(pady=20)

        connect_button = tk.Button(root, text="Connect a Leader", command=self.connect_to_crazyflie)
        connect_button.pack(pady=10)

        disconnect_button = tk.Button(root, text="Disconnect Leader", command=self.disconnect_crazyflie)
        disconnect_button.pack(pady=10)

        start_button = tk.Button(root, text="Send Start Command", command=self.sendStartCommand)
        start_button.pack(pady=10)

        square_form_button = tk.Button(root, text="Send Square Formation Command", command=self.sendSquareCommand)
        square_form_button.pack(pady=10)

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
        self.ax.plot_surface(x,y,z, marker = 'x', color='b', label='leader')
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
        #     self.cf.open_link('radio://0/80/2M/E7E7E7E7E6')
        else:
            print("Error", "No Crazyflies found, cannot run example")

    def disconnect_crazyflie(self):
        self.currentLeader = "None"
        self.leader_label.config(text=f"Current Leader: {self.currentLeader}")
        self.cf.close_link()

    def sendStartCommand(self):
        
        self.receivedData = False

        while(not self.receivedData):
            data = struct.pack("<i", 1)
            self.cf.appchannel.send_packet(data)
            print("Sent start command")
            time.sleep(1)
    
    def sendSquareCommand(self):

        self.receivedData = False

        while(not self.receivedData):
            data = struct.pack("<i", 2)
            self.cf.appchannel.send_packet(data)
            print("Sent start command")
            time.sleep(1)

if __name__ == '__main__':
    root = tk.Tk()
    app = App(root)
    root.mainloop()