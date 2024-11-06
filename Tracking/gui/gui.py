import numpy as np
import matplotlib.pyplot as plt
import serial
import time

# Replace 'COM_PORT' with your Arduino's COM port
# For example, 'COM3' on Windows or '/dev/ttyUSB0' on Linux
SERIAL_PORT = 'COM7'
BAUD_RATE = 38400

# Initialize the serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Give some time for the serial connection to establish

# Create a figure and a polar subplot
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, polar=True)

# Initialize lists to store azimuth and elevation
azimuth_data = []
elevation_data = []

# Set limits for the radial axis (elevation)
ax.set_ylim(0, np.pi / 2)  # Elevation ranges from 0 to 90 degrees (0 to pi/2 in radians)

# Customize the plot
ax.set_title("Satellite Skyplot", va='bottom')

# Set azimuth ticks to start from 0 at North and go clockwise
ax.set_xticks(np.radians(np.arange(0, 360, 30)))  # Set azimuth ticks (0 to 360 degrees)
ax.set_xticklabels(np.arange(0, 360, 30))          # Labels for azimuth ticks

# Set elevation ticks (90 at the center to 0 at the edge)
ax.set_yticks(np.radians(np.arange(0, 91, 15)))    # Elevation ticks
ax.set_yticklabels(np.arange(90, -1, -15))         # Labels for elevation ticks (90 at center, 0 at edge)

# Function to update the plot
def update_plot():
    ax.clear()  # Clear previous plot
    ax.set_ylim(0, np.pi / 2)  # Reset limits for elevation
    ax.set_title("Satellite Skyplot", va='bottom')
    
    # Reset azimuth ticks and labels
    ax.set_xticks(np.radians(np.arange(0, 360, 30)))  # Set azimuth ticks (0 to 360 degrees)
    ax.set_xticklabels(np.arange(0, 360, 30))          # Labels for azimuth ticks

    # Reset elevation ticks and labels
    ax.set_yticks(np.radians(np.arange(0, 91, 15)))    # Reset elevation ticks
    ax.set_yticklabels(np.arange(90, -1, -15))         # Reset elevation labels

    # Convert elevation and azimuth to radians
    elevation_rad = np.radians(90 - np.array(elevation_data))  # 90 degrees at center
    azimuth_rad = np.radians(np.array(azimuth_data))  # Azimuth in radians (0 degrees at North)

    # Scatter plot
    ax.scatter(azimuth_rad, elevation_rad, c='r', s=100, alpha=0.75)  # Red dots for each point

    plt.draw()  # Update the plot
    plt.pause(0.1)  # Pause to allow the plot to update

# Main loop to read data from Arduino and update the plot
try:
    while True:
        # Read a line from the serial
        line = ser.readline().decode('utf-8').strip()  # Read the line, decode, and strip whitespace
        if line:
            # Split the line into azimuth and elevation
            try:
                azimuth, elevation = map(float, line.split(','))
                azimuth_data.append(azimuth)
                elevation_data.append(elevation)

                # Update the plot with the new data
                update_plot()

            except ValueError:
                print(f"Invalid data received: {line}")

except KeyboardInterrupt:
    print("Plotting stopped by user.")

finally:
    ser.close()  # Close the serial connection
    plt.show()  # Keep the plot open at the end
