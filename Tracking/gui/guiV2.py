import tkinter as tk
from tkinter import messagebox, ttk
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import threading
import sys
import math
import numpy as np

def find_theta_AU(d):
    k = 135.82178  # deg
    x = 174.9451   # mm
    L = 31.5209    # mm
    A = 161.5258   # mm
    kmt = math.acos((L**2 + x**2 - (A+d)**2)/(2*L*x)) * 180/ math.pi
    theta =  k - kmt
    return 90 - theta

def find_theta_AL(d):
    k =  80.47653
    x = 257.4232
    L = 53.4489
    A = 212.5258   # mm
    kmt = math.acos((L**2 + x**2 - (A+d)**2)/(2*L*x)) * 180/ math.pi
    theta =  k - kmt
    return theta

# Set up serial parameters
SERIAL_PORT = 'COM7'  # Update as needed
BAUD_RATE = 9600              # Update as needed

# Initialize lists to store data for plotting
data_x = []
azi_degrees = []
ele_degrees = []
data_y2 = []
data_y3 = []

# Set up the serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
except serial.SerialException as e:
    print(f"Could not open serial port: {e}")
    sys.exit(1)

# Initialize the plot with two subplots
fig, ax1 = plt.subplots() #figsize=(8, 6)

ax1 = fig.add_subplot(111, polar=True)
ax1.set_ylim(0, 90)  # Limit to the visible sky
ax1.set_title('Sky Plot of Satellite Pass', va='bottom')
ax1.set_theta_zero_location('N')  # Set North at the top
ax1.set_theta_direction(-1)  # Clockwise direction

# Initial scatter plot for ax1 and line plot for ax2
scatter1 = ax1.scatter([], [], c='blue')
# line2, = ax2.plot([], [], 'r-', lw=2, label="Error Theta_AU")
# line3, = ax2.plot([], [], 'k-', lw=2, label="Error Theta_AL")


# Configure plot settings
# ax1.set_title("Real-Time Data from Sensor 1 (Scatter)")
# ax2.set_title("Real-Time Data from Sensor 2 (Line)")
# # ax1.set_xlabel("Time")
# # ax1.set_ylabel("Value 1")
# ax2.set_xlabel("Time")
# ax2.set_ylabel("Angle Error")
# ax2.set_ylim([-0.25, 0.25])

# Function to update each subplot with new data
def update_plot(frame):
    # if len(data_x) > 0:
    #     ax1.set_xlim(max(0, data_x[-1] - 100), data_x[-1] + 10)
    #     ax2.set_xlim(max(0, data_x[-1] - 100), data_x[-1] + 10)
    # global ax2
    # if len(data_x) > 0:
    #     ax2.set_xlim(0, max(data_x))
    
    # Update scatter plot on ax1
    # ax1.collections.clear()

    mask = np.array(ele_degrees) > 0

    ax1.scatter(np.radians(np.array(azi_degrees)[mask]), 90 - np.array(ele_degrees)[mask], c='blue')

    # Update the table with Value 2 and time
    # Update the table with only the latest Value 2 and time
    for item in table.get_children():
        table.delete(item)  # Clear previous row
    #table.insert('', 'end', values=(x_val, y_val2))  # Add latest data
    if len(azi_degrees) > 0:
        table.insert('', 'end', values=(azi_degrees[-1], ele_degrees[-1]))
    # print(azi_degrees[-1], ele_degrees[-1], data_y2[-1])

    # Update line plot on ax2
    # line2.set_data(data_x, data_y2)
    # line3.set_data(data_x, data_y3)
    # ax2.relim()
    # ax2.autoscale_view()

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
                    # Assume data is a single float per line
                    # print(line)
                    # print(line)
                    if line.strip() == "Start":
                        start_read_bool = 1
                    if (start_read_bool):
                        line = line.split(",")
                        # print(line)
                        if len(line) >= 11:
                            d_AU_2_a = float(line[10])
                            d_AU_2_r = float(line[5])
                            d_AL_2_a = float(line[11])
                            d_AL_2_r = float(line[6])
                            theta_AU_a =  find_theta_AU(d_AU_2_a)
                            theta_AU_r = find_theta_AU(d_AU_2_r)
                            theta_AL_a = find_theta_AL(d_AL_2_a)
                            theta_AL_r = find_theta_AL(d_AL_2_r)
                            err_val_2 = theta_AL_a - theta_AL_r
                            y_val = theta_AU_a - theta_AU_r
                            x_val = int(line[0])
                            if iter >= 3:
                                azi_value = float(line[7])
                                ele_value = float(line[8])
                                # print(x_val, y_val, azi_value, ele_value)
                                data_x.append(x_val)
                                data_y2.append(y_val)
                                data_y3.append(err_val_2)
                                azi_degrees.append(azi_value)
                                ele_degrees.append(ele_value)
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

# Set up the table (Treeview) to display Value 2
table = ttk.Treeview(table_frame, columns=("Azimuth", "Elevation"), show="headings")
table.heading("Azimuth", text="Azimuth")
table.heading("Elevation", text="Elevation")
table.column("Azimuth", anchor="center", width=100)
table.column("Elevation", anchor="center", width=100)
table.pack(fill=tk.BOTH, expand=True)


# Start reading serial in a new thread
serial_thread = threading.Thread(target=read_serial)
serial_thread.daemon = True
serial_thread.start()

# Define a safe shutdown function
def safe_shutdown():
    global running
    running = False
    ser.close()
    root.quit()
    root.destroy()

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





# import tkinter as tk
# from tkinter import messagebox
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# import serial
# import threading
# import sys
# import numpy as np
# import math
# import time
# def round_up_to_2_sig_figs(number):
#     # Handle case for zero
#     if number == 0:
#         return 0
#     # Calculate the order of magnitude (i.e., how many digits before the decimal point)
#     magnitude = math.floor(math.log10(abs(number)))
#     # Scale the number so that the first two significant figures are left of the decimal
#     scale = 10 ** (2 - magnitude - 1)
#     # Apply ceiling to always round up
#     return math.ceil(number * scale) / scale

# # Set up serial parameters
# SERIAL_PORT = 'COM7'  # Update as needed
# BAUD_RATE = 9600              # Update as needed

# # Initialize empty lists to store data for plotting
# data_x = []
# data_y = []

# # Set up the serial connection
# try:
#     ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
# except serial.SerialException as e:
#     print(f"Could not open serial port: {e}")
#     sys.exit(1)

# # Initialize the plot
# fig, (ax_sky, ax) = plt.subplots(2, 1)

# # Set up for sky plot
# ax_sky = fig.add_subplot(211, polar=True)
# azi_data_degrees = []
# ele_data_degrees = []
# line, = ax.plot([], [], 'b-', lw=2)
# ax.set_ylim([0, 38])
# ax.set_xlim([0, 15000])
# ax_sky.set_ylim(0, 90)  # Limit to the visible sky
# ax_sky.set_theta_zero_location('N')  # Set North at the top
# ax_sky.set_theta_direction(-1)  # Clockwise direction
# ax_sky.grid(True)
# path = ax_sky.scatter([], [], c='blue')
# # Function to update the plot with new data
# def update_plot(frame):
#     global ax
#     global ax_sky
#     # Dynamic X Axis
#     if len(data_x) > 0:
#         ax.set_xlim(0, max(data_x))
#     #     ax.set_xlim(max(0, data_x[-1] - 100), data_x[-1] + 10)  # Dynamic x-axis
#     line.set_data(data_x, data_y)
#     mask = np.array(ele_data_degrees) > 0
#     # path.set_data(np.radians(np.array(azi_data_degrees)[mask]),  90 - np.array(ele_data_degrees)[mask])
#     ax_sky.collections.clear()

#     # plot new data 
#     ax_sky.scatter(np.radians(np.array(azi_data_degrees)[mask]), 90 - np.array(ele_data_degrees)[mask], 
#                c='blue', label='Satellite Pass', alpha=0.75)

#     # ax.relim()
#     # ax.autoscale_view()
#     return line, path

# # Function to read data from serial in a separate thread
# def read_serial():
#     global running
#     running = True
#     x_val = 0

#     while running:
#         try:
#             line = ser.readline().decode('utf-8').strip()
#             if line:
#                 try:
#                     # Assume data is a single float per line
#                     print(line)
#                     line = line.split(",")
#                     # print(line)
#                     if len(line) >= 10:
#                         y_val = float(line[10])
#                         x_val = int(line[0])
#                         azi_value = float(line[7])
#                         ele_value = float(line[8])
#                         print(x_val, y_val, azi_value, ele_value)
#                         data_x.append(x_val)
#                         data_y.append(y_val)
#                         azi_data_degrees.append(azi_value)
#                         ele_data_degrees.append(ele_value)
#                         x_val += 1
#                 except ValueError:
#                     print("Invalid data format")
#         except serial.SerialException:
#             print("Lost connection to serial port")
#             break

# # Set up animation
# ani = animation.FuncAnimation(fig, update_plot, blit=True)

# # Tkinter setup
# root = tk.Tk()
# root.title("Real-Time Serial Plot")

# # Embed the matplotlib figure in Tkinter
# from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
# canvas = FigureCanvasTkAgg(fig, master=root)
# canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# # Start reading serial in a new thread
# serial_thread = threading.Thread(target=read_serial)
# serial_thread.daemon = True
# serial_thread.start()

# # Define a safe shutdown function
# def safe_shutdown():
#     global running
#     running = False
#     ser.close()
#     root.quit()
#     root.destroy()

# # Catch Ctrl+C for safe shutdown
# def on_close():
#     if messagebox.askokcancel("Quit", "Do you want to quit?"):
#         safe_shutdown()

# root.protocol("WM_DELETE_WINDOW", on_close)

# # Run Tkinter main loop
# try:
#     tk.mainloop()
# except KeyboardInterrupt:
#     safe_shutdown()