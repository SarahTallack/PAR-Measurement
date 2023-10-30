import serial
import matplotlib.pyplot as plt
import numpy as np
import random

# Set the serial port and baud rate
serial_port = "COM6"  # Change to your UART device's port
baud_rate = 115200  # Change to match your UART configuration

# Initialize a serial connection
ser = serial.Serial(serial_port, baud_rate)

# Initialize variables for data storage
data_1 = [0, 0, 0, 0, 0, 0, 0, 0, 0]
data_2 = [0, 0, 0, 0]  # Initialize with zeros for the second bar chart

max = 4000

# Define colors for the first subplot
AS_Colors = [
    (0.5, 0, 1),
    (0, 0, 1),
    (0, 1, 1),
    (0, 1, 0),
    (1, 1, 0),
    (1, 0.6, 0),
    (1, 0, 0),
    (0.7, 0, 0),
    (1, 1, 1)
]

# Define colors for the second subplot
VEML_Colors = [
    (0, 0, 1),
    (0, 1, 0),
    (1, 0, 0),
    (1, 1, 1)
]

# Create two bar charts
plt.figure(figsize=(12, 6), facecolor='black')

# Set the chart titles and labels for the first bar chart
ax1 = plt.subplot(1, 2, 1, facecolor='black')
ax1.spines['bottom'].set_color('white')  # Set bottom border color
ax1.spines['top'].set_color('white')  # Set top border color
ax1.spines['right'].set_color('white')  # Set right border color
ax1.spines['left'].set_color('white')  # Set left border color
plt.title("AS7341 Sensor", color = 'white')
plt.xlabel("Channel", color = 'white')
plt.ylabel("Counts", color = 'white')
plt.ylim(0, max)  # Set the y-axis range for the first chart
x_labels_1 = ['CH1', 'CH2', 'CH3', 'CH4', 'CH5', 'CH6', 'CH7', 'CH8', 'CLEAR' ]  # Individual labels for the first chart

bar_chart_1 = plt.bar(x_labels_1, data_1, color=AS_Colors)

# Set the color of x-axis and y-axis numbers to white
plt.xticks(color='white')
plt.yticks(color='white')

# Set the chart titles and labels for the second bar chart
ax2 = plt.subplot(1, 2, 2, facecolor='black')
ax2.spines['bottom'].set_color('white')  # Set bottom border color
ax2.spines['top'].set_color('white')  # Set top border color
ax2.spines['right'].set_color('white')  # Set right border color
ax2.spines['left'].set_color('white')  # Set left border color
plt.title("VEML6040 Sensor", color = 'white')
plt.xlabel("Channel", color = 'white')
plt.ylabel("Counts", color = 'white')
plt.ylim(0, max)  # Set the y-axis range for the second chart
x_labels_2 = ["BLUE", "GREEN", "RED", "WHITE"]  # Individual labels for the second chart
bar_chart_2 = plt.bar(x_labels_2, data_2, color=VEML_Colors)

# Set the color of x-axis and y-axis numbers to white
plt.xticks(color='white')
plt.yticks(color='white')

# Function to update the bar charts with new data
def update_bar_charts(new_data_1, new_data_2):
    for bar_1, value_1 in zip(bar_chart_1, new_data_1):
        bar_1.set_height(value_1)
    for bar_2, value_2 in zip(bar_chart_2, new_data_2):
        bar_2.set_height(value_2)
    plt.pause(0.01)

try:
    while True:
        # Read data from the UART
        uart_data = ser.readline().strip().decode('utf-8')
        
        # Split the data into individual values
        values = uart_data.split(',')
        
        # Initialize lists for the first 9 values and values 11 to 14
        data_1 = [0] * 9
        data_2 = [0, 0, 0, 0]

        # Process the values and convert them to floats (if valid)
        for i, val in enumerate(values):
            try:
                float_val = float(val)
                if i < 9:
                    data_1[i] = float_val
                elif 10 <= i < 14:
                    data_2[i - 10] = float_val
            except ValueError:
                pass  # Ignore non-numeric values

        # Update the bar charts
        update_bar_charts(data_1, data_2)

except KeyboardInterrupt:
    print("Exiting...")

# Close the serial connection
ser.close()