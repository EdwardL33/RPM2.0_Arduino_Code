from Phidget22.Phidget import *
from Phidget22.Devices.Accelerometer import *
from Phidget22.Devices.TemperatureSensor import *
from Phidget22.Devices.Spatial import *

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from datetime import datetime
import time
import csv
import sys
import statistics
import serial
import threading

# --- Data Storage ---
timestamps = []
x_vals, y_vals, z_vals = [], [], []
temperature_vals = []
latest_temperature = [0]
x_mean, y_mean, z_mean, norm = [0], [0], [0], [0]
latest_outer_encoder = [""]
latest_inner_encoder = [""]
latest_outer_rpm = [""]
latest_inner_rpm = [""]
latest_payload_rpm = [""]




# --- CSV Setup ---
filename = f"accel_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
csv_file = open(filename, mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["Timestamp", "Accel_X", "Accel_Y", "Accel_Z", "Temperature", "outer_encoder", "inner_encoder", "outer_rpm", "inner_rpm", "payload_rpm"])

# --- Serial Setup ---
ser = serial.Serial('COM3', 115200, timeout=0.01)
serial_lock = threading.Lock()


def send_arduino_command(command_char):
    if not command_char:
        return

    char_to_send = command_char[0]
    try:
        with serial_lock:
            ser.write(char_to_send.encode('utf-8'))
            ser.flush()
        print(f"Sent command: {char_to_send}")
    except serial.SerialException as e:
        print(f"Failed to send command '{char_to_send}': {e}")


def command_input_worker():
    print("Command input ready. Type one character and press Enter to send it to Arduino.")
    while True:
        try:
            user_input = input().strip()
        except EOFError:
            return

        if not user_input:
            continue

        send_arduino_command(user_input[0])


def read_encoder_line():
    with serial_lock:
        if ser.in_waiting <= 0:
            return

        raw = ser.readline().decode('utf-8', errors='ignore').strip()
    if not raw:
        return

    parts = raw.split(',')
    if len(parts) not in (2, 4, 5):
        return

    latest_outer_encoder[0] = parts[0].strip()
    latest_inner_encoder[0] = parts[1].strip()
    if len(parts) >= 4:
        latest_outer_rpm[0] = parts[2].strip()
        latest_inner_rpm[0] = parts[3].strip()
    if len(parts) == 5:
        latest_payload_rpm[0] = parts[4].strip()

def wait_for_temperature_stabilization(sensor):
    print("Waiting for temperature to reach at least 50°C...")
    while True:
        current_temp = sensor.getTemperature()
        print(f"Current temperature: {current_temp:.2f}°C")
        if current_temp >= 50:
            print(f"Temperature reached {current_temp:.2f}°C")
            return
        time.sleep(0.25)


# --- Phidget Event Handlers ---
def onAccelerationChange(self, acceleration, timestamp):
    ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    x, y, z = acceleration
    temp = latest_temperature[0]
    read_encoder_line()
    outer_encoder = latest_outer_encoder[0]
    inner_encoder = latest_inner_encoder[0]
    outer_rpm = latest_outer_rpm[0]
    inner_rpm = latest_inner_rpm[0]
    payload_rpm = latest_payload_rpm[0]

    x_vals.append(x)
    y_vals.append(y)
    z_vals.append(z)
    timestamps.append(ts)
    temperature_vals.append(temp)

    
    # Trim to last 100 points
    if len(x_vals) > 100:
        x_vals.pop(0)
        y_vals.pop(0)
        z_vals.pop(0)
        temperature_vals.pop(0)
        timestamps.pop(0)
    
    csv_writer.writerow([ts, x, y, z, temp, outer_encoder, inner_encoder, outer_rpm, inner_rpm, payload_rpm])

def onTemperatureChange(self, temperature):
    latest_temperature[0] = temperature

# --- Phidget Setup ---
accelerometer = Accelerometer()
temp_sensor = TemperatureSensor()
spatial = Spatial()

# --- RPM 1 Phidget ---
# accelerometer.setDeviceSerialNumber(721291) 
# temp_sensor.setDeviceSerialNumber(721291)
# spatial.setDeviceSerialNumber(721291)

# --- RPM 2 Phidget ---
accelerometer.setDeviceSerialNumber(721166) # 721166
temp_sensor.setDeviceSerialNumber(721166)
spatial.setDeviceSerialNumber(721166)

accelerometer.setOnAccelerationChangeHandler(onAccelerationChange)
temp_sensor.setOnTemperatureChangeHandler(onTemperatureChange)


accelerometer.openWaitForAttachment(5000)
temp_sensor.openWaitForAttachment(5000)
spatial.openWaitForAttachment(5000)

spatial.setHeatingEnabled(True)

# Optional: Faster data rate
accelerometer.setDataInterval(20)
wait_for_temperature_stabilization(temp_sensor)

# --- Plot Setup ---
fig, ax = plt.subplots(2, 1, figsize=(10, 6))
fig.tight_layout(pad=3)

command_thread = threading.Thread(target=command_input_worker, daemon=True)
command_thread.start()

def animate(i):
    if not x_vals:
        print("No data yet.")
        return

    ax[0].clear()
    ax[0].plot(x_vals, label='X')
    ax[0].plot(y_vals, label='Y')
    ax[0].plot(z_vals, label='Z')
    ax[0].set_title("Accelerometer")
    ax[0].legend()
    ax[0].grid(True)

    ax[1].clear()
    ax[1].plot(temperature_vals, label='Temperature', color='orange')
    ax[1].set_title("Temperature")
    ax[1].legend()
    ax[1].grid(True)
# --- Start Live Plot ---
ani = animation.FuncAnimation(fig, animate, interval=500, cache_frame_data=False)
sys.modules[__name__].ani = ani  # Prevent garbage collection

plt.show()

# --- Cleanup ---
accelerometer.close()
temp_sensor.close()
ser.close()
csv_file.close()