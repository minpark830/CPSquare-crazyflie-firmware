import pickle
import tkinter as tk
from tkinter import filedialog, messagebox
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
from PIL import Image, ImageTk 
import os

def load_pickle_data(file_path):
    with open(file_path, "rb") as f:
        data = pickle.load(f)
    return data

def plot_data(data):
    try:
        # Mapping of IDs to roles and colors
        role_info = {
            231: ("Leader", 'blue'),
            232: ("Follower 1", 'green'),
            230: ("Follower 2", 'red'),
            233: ("Follower 3", 'orange')
        }

        # Group data by ID
        grouped_data = {id_: [] for id_ in role_info}

        for id_, x, y, z in data:
            if id_ in grouped_data:
                grouped_data[id_].append((x, y, z))

        fig = plt.figure(facecolor='white')
        ax = fig.add_subplot(111, projection='3d', facecolor='white')

        # Plot each role
        for id_, points in grouped_data.items():
            if points:
                xs, ys, zs = zip(*points)
                label, color = role_info[id_]
                ax.scatter(xs, ys, zs, label=label, color=color, marker='o')

        ax.set_xlabel('X axis [m]')
        ax.set_ylabel('Y axis [m]')
        ax.set_zlabel('Z axis [m]')
        ax.grid(True)
        plt.title("3D Plot from Pickle Data")
        ax.legend(title="Role")
        plt.show()

    except Exception as e:
        messagebox.showerror("Error", f"Failed to plot data:\n{e}")

def open_file():
    file_path = filedialog.askopenfilename(
        title="Select Pickle File",
        filetypes=[("Pickle files", "*.pkl *.pickle"), ("All files", "*.*")]
    )
    if file_path:
        data = load_pickle_data(file_path)
        if data:
            plot_data(data)

root = tk.Tk()
root.title("3D Pickle Data Plotter")
root.configure(bg="white")

start_path = os.path.join(os.getcwd(), "examples", "app_leader", "tools", "cpsquare-logo.png")

img = Image.open(start_path)  # Replace with the path to your image

# Rescale the image (change size to 300x300 pixels)
img_resized = img.resize((150, 150))  # Replace (300, 300) with your desired size

# Convert the resized image to a format Tkinter can use
img_tk = ImageTk.PhotoImage(img_resized)

# Create a label to hold the image
logo_label = tk.Label(root, image=img_tk, bg='white')
logo_label.pack(pady=20)

frame = tk.Frame(root, padx=20, pady=20, bg="white")
frame.pack()

label = tk.Label(frame, text="Load and plot 3D data from a pickle file", font=("Arial", 12), bg="white")
label.pack(pady=10)

button = tk.Button(frame, text="Open Pickle Data File", command=open_file, font=("Arial", 12), width=20)
button.pack(pady=10)

root.mainloop()
