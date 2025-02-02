import tkinter as tk
from tkinter import ttk
import serial
import threading
import matplotlib.pyplot as plt
import numpy as np
import time

# Function to update the color square based on RGB values
def update_color_square():
    r = round(r_var.get())
    g = round(g_var.get())
    b = round(b_var.get())
    color = f'#{r:02x}{g:02x}{b:02x}'
    color_square.config(bg=color)

# Function to send the data string over UART
def send_data():
    r = round(r_var.get())
    g = round(g_var.get())
    b = round(b_var.get())
    brightness = round(brightness_slider.get())
    mode = mode_var.get()
    led_position = led_position_var.get()

    # Create the data string
    data_string = f"{r},{g},{b},{brightness},{mode},{led_position}\n"
    print(f"Sending: {data_string}")  # For debugging

    # Send the data over UART
    try:
        ser.write(data_string.encode('utf-8'))
    except Exception as e:
        print(f"Error sending data: {e}")

# Function to read and plot data from the serial port
def read_uart_and_plot():
    max_value = 4000
    data_1 = [0] * 9
    data_2 = [0] * 4

    AS_Colors = [(0.5, 0, 1), (0, 0, 1), (0, 1, 1), (0, 1, 0), (1, 1, 0), (1, 0.6, 0), (1, 0, 0), (0.7, 0, 0), (1, 1, 1)]
    VEML_Colors = [(0, 0, 1), (0, 1, 0), (1, 0, 0), (1, 1, 1)]

    plt.ion()  # Interactive mode on for real-time plotting
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6), facecolor='black')

    bar_chart_1 = ax1.bar(['CH1', 'CH2', 'CH3', 'CH4', 'CH5', 'CH6', 'CH7', 'CH8', 'CLEAR'], data_1, color=AS_Colors)
    bar_chart_2 = ax2.bar(['BLUE', 'GREEN', 'RED', 'WHITE'], data_2, color=VEML_Colors)

    plt.show(block=False)

    while running:  # Keep updating while running is True
        try:
            if ser.in_waiting:  # Check if there is data available
                uart_data = ser.readline().strip().decode('utf-8')
                values = uart_data.split(',')

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
                for bar_1, value_1 in zip(bar_chart_1, data_1):
                    bar_1.set_height(value_1)
                for bar_2, value_2 in zip(bar_chart_2, data_2):
                    bar_2.set_height(value_2)
                
                fig.canvas.draw()  # Redraw the figure
                fig.canvas.flush_events()  # Allow events to process

            time.sleep(0.1)  # Add a small delay to avoid overloading CPU

        except Exception as e:
            print(f"Error reading data: {e}")
            break

# Initialize serial connection for UART communication
serial_port = "COM8"  # Change to your UART device's port
baud_rate = 115200  # Change to match your UART configuration
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Create the main window
root = tk.Tk()
root.title("UART Control Panel")

# Variables for RGB, mode, and LED position
r_var = tk.IntVar(value=0)
g_var = tk.IntVar(value=255)
b_var = tk.IntVar(value=0)
mode_var = tk.IntVar(value=0)
led_position_var = tk.IntVar(value=0)

# RGB Sliders and Entry Boxes
ttk.Label(root, text="Red:").grid(column=0, row=0, padx=5, pady=5)
r_slider = ttk.Scale(root, from_=0, to=255, orient='horizontal', variable=r_var, command=lambda x: update_color_square())
r_slider.grid(column=1, row=0, padx=5, pady=5)
r_entry = ttk.Entry(root, textvariable=r_var, width=5)
r_entry.grid(column=2, row=0, padx=5)

ttk.Label(root, text="Green:").grid(column=0, row=1, padx=5, pady=5)
g_slider = ttk.Scale(root, from_=0, to=255, orient='horizontal', variable=g_var, command=lambda x: update_color_square())
g_slider.grid(column=1, row=1, padx=5, pady=5)
g_entry = ttk.Entry(root, textvariable=g_var, width=5)
g_entry.grid(column=2, row=1, padx=5)

ttk.Label(root, text="Blue:").grid(column=0, row=2, padx=5, pady=5)
b_slider = ttk.Scale(root, from_=0, to=255, orient='horizontal', variable=b_var, command=lambda x: update_color_square())
b_slider.grid(column=1, row=2, padx=5, pady=5)
b_entry = ttk.Entry(root, textvariable=b_var, width=5)
b_entry.grid(column=2, row=2, padx=5)

# Color Square
color_square = tk.Label(root, text="", bg="black", width=10, height=5)
color_square.grid(column=3, row=0, rowspan=3, padx=10, pady=5)

# Brightness Slider
ttk.Label(root, text="Brightness:").grid(column=0, row=3, padx=5, pady=5)
brightness_slider = ttk.Scale(root, from_=0, to=100, orient='horizontal')
brightness_slider.grid(column=1, row=3, padx=5, pady=5)
brightness_slider.set(75)

# Mode Selector (0: All LEDs on, 1: Single LED on)
ttk.Label(root, text="Mode:").grid(column=0, row=4, padx=5, pady=5)
mode_frame = tk.Frame(root)
mode_frame.grid(column=1, row=4, columnspan=2)
mode_0_button = ttk.Radiobutton(mode_frame, text="All LEDs On (0)", variable=mode_var, value=0)
mode_1_button = ttk.Radiobutton(mode_frame, text="Single LED On (1)", variable=mode_var, value=1)
mode_0_button.grid(row=0, column=0)
mode_1_button.grid(row=0, column=1)

# LED Position Slider (0 to 23)
ttk.Label(root, text="LED Position:").grid(column=0, row=5, padx=5, pady=5)
led_position_slider = ttk.Scale(root, from_=0, to=23, orient='horizontal', variable=led_position_var)
led_position_slider.grid(column=1, row=5, padx=5, pady=5)
led_position_entry = ttk.Entry(root, textvariable=led_position_var, width=5)
led_position_entry.grid(column=2, row=5, padx=5)

# Button to send data
send_button = ttk.Button(root, text="Send Data", command=send_data)
send_button.grid(column=0, row=6, columnspan=3, padx=5, pady=10)

# Thread to handle UART data reading and real-time plotting
running = True
uart_thread = threading.Thread(target=read_uart_and_plot)
uart_thread.daemon = True  # Make sure thread exits when main program exits
uart_thread.start()

# Close serial and stop the thread when the window is closed
def on_closing():
    global running
    running = False
    ser.close()  # Close serial port
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)

# Run the GUI main loop
root.mainloop()
