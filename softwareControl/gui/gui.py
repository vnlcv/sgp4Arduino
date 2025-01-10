import tkinter as tk
from tkinter import messagebox, ttk
import tkinter.font as tkFont
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import threading
import sys
import math
import numpy as np

# Set up serial parameters
SERIAL_PORT = 'COM5'  
BAUD_RATE = 9600              

# Initialize lists to store data for plotting
azi_degrees = []
ele_degrees = []

# Set up the serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
except serial.SerialException as e:
    print(f"Could not open serial port: {e}")
    sys.exit(1)

# Initialize the plot with two subplots
fig, ax1 = plt.subplots() #figsize=(8, 6)

ax1 = fig.add_subplot(111, polar=True)
ax1.set_ylim(90,0)  # Limit to the visible sky
ax1.set_title('Sky Plot of Satellite Pass', va='bottom')
ax1.set_theta_zero_location('N')  # Set North at the top
ax1.set_theta_direction(-1)  # Clockwise direction

# Add radial labels with degree symbols
ylabels = ['0°', '10°', '20°', '30°', '40°', '50°', '60°', '70°', '80°', '90°']
ax1.set_yticklabels(ylabels)  # Add labels with degree symbols

# Initial scatter plot for ax1 and line plot for ax2
scatter1 = ax1.scatter([], [], c='blue')

# Function to update each subplot with new data
def update_plot(frame):
    # mask = np.array(ele_degrees) > 0
    if (len(azi_degrees) == len(ele_degrees)):
        ax1.scatter(np.radians(np.array(azi_degrees)), np.array(ele_degrees), c='blue')

    # Update the table
    for item in table.get_children():
        table.delete(item)  # Clear previous row
    if len(azi_degrees) > 0:
        table.insert('', 'end', values=(azi_degrees[-1], ele_degrees[-1]))

def read_serial():
    global running
    running = True
    x_val = 0
    iter = 0
    start_read_bool = False

    while running:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                try:
                    line = line.split(",")
                    # print(line)
                    if len(line) >= 2:
                        if iter >= 2:
                            azi_value = float(line[0])
                            ele_value = float(line[1])
                            # print(azi_value, ele_value)
                            azi_degrees.append(azi_value)
                            ele_degrees.append(ele_value)
                            print(azi_value, ele_value,len(azi_degrees), len(ele_degrees))
                        iter += 1
                except ValueError:
                    print("Invalid data format")
        except serial.SerialException:
            print("Lost connection to serial port")
            break
# Set up animation for both subplots
ani = animation.FuncAnimation(fig, update_plot, blit=False)

# Tkinter setup
root = tk.Tk()
root.title("Real-Time Serial Plot with Mixed Subplots")

# Embed the matplotlib figure in Tkinter
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# Create a frame for the table
table_frame = tk.Frame(root)
table_frame.pack(fill=tk.BOTH, expand=True)

# Define a larger font for the Treeview widget
font_large = tkFont.Font(family="Helvetica", size=16)
font_heading = tkFont.Font(family="Helvetica", size=16, weight="bold")

# Set up the table (Treeview) to display Value 2
table = ttk.Treeview(table_frame, columns=("Azimuth", "Elevation"), show="headings")
table.heading("Azimuth", text="Azimuth")
table.heading("Elevation", text="Elevation")
table.column("Azimuth", anchor="center", width=150)
table.column("Elevation", anchor="center", width=150)
table.pack(fill=tk.BOTH, expand=True)

# Start reading serial in a new thread
serial_thread = threading.Thread(target=read_serial)
serial_thread.daemon = True
serial_thread.start()

# Define a safe shutdown function
def safe_shutdown():
    global running
    running = False
    root.quit()
    root.destroy()
    ser.close()


tk.Button(root, text="Quit",command=safe_shutdown).pack()

# Catch Ctrl+C for safe shutdown
def on_close():
    if messagebox.askokcancel("Quit", "Do you want to quit?"):
        safe_shutdown()

root.protocol("WM_DELETE_WINDOW", on_close)

# Run Tkinter main loop
try:
    tk.mainloop()
except KeyboardInterrupt:
    safe_shutdown()