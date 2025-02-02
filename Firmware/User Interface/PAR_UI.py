import tkinter as tk
from tkinter import ttk
import serial

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

# Open the serial port (replace 'COM8' with your port)
ser = serial.Serial('COM8', 9600, timeout=1)

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
mode_0_button = ttk.Radiobutton(mode_frame, text="All LEDs On", variable=mode_var, value=0)
mode_1_button = ttk.Radiobutton(mode_frame, text="Single LED On", variable=mode_var, value=1)
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

# Run the application
root.mainloop()

# Close the serial port when done
ser.close()
