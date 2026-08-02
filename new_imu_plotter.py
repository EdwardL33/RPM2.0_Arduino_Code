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

# --- Data Storage ---
timestamps = []
x_vals, y_vals, z_vals = [], [], []
temperature_vals = []
latest_temperature = [0]
x_mean, y_mean, z_mean, norm = [0], [0], [0], [0]

# --- NEW ADDITIONS START: Running Average Variables ---
collecting_integration_data = False
start_time = None
total_x, total_y, total_z = 0.0, 0.0, 0.0
total_samples = 0
current_running_g = 0.0

milestone_targets = {
    "10 Min": 600,
    "1 Hour": 3600,
    "10 Hours": 36000,
    "24 Hours": 86400,
    "1 Week": 604800
}
milestone_results = {k: None for k in milestone_targets.keys()}
# --- NEW ADDITIONS STOP ---


# --- CSV Setup ---
filename = f"accel_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
csv_file = open(filename, mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["Timestamp", "Accel_X", "Accel_Y", "Accel_Z", "Temperature"])

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
    # --- NEW ADDITIONS START: Globals for Integration ---
    global total_x, total_y, total_z, total_samples, current_running_g
    # --- NEW ADDITIONS STOP ---

    ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    x, y, z = acceleration
    temp = latest_temperature[0]

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
    
    csv_writer.writerow([ts, x, y, z, temp])

    # --- NEW ADDITIONS START: Calculate Running Average ---
    if collecting_integration_data:
        total_x += x
        total_y += y
        total_z += z
        total_samples += 1

        mean_x = total_x / total_samples
        mean_y = total_y / total_samples
        mean_z = total_z / total_samples
        
        current_running_g = (mean_x**2 + mean_y**2 + mean_z**2)**0.5

        # Check and lock in milestones
        elapsed = time.time() - start_time
        for name, threshold in milestone_targets.items():
            if elapsed >= threshold and milestone_results[name] is None:
                milestone_results[name] = current_running_g
    # --- NEW ADDITIONS STOP ---

def onTemperatureChange(self, temperature):
    latest_temperature[0] = temperature

# --- Phidget Setup ---
accelerometer = Accelerometer()
temp_sensor = TemperatureSensor()
spatial = Spatial()

accelerometer.setDeviceSerialNumber(721166)
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

# --- NEW ADDITIONS START: Start Integration Timer ---
start_time = time.time()
collecting_integration_data = True
# --- NEW ADDITIONS STOP ---

# --- Plot Setup ---
fig, ax = plt.subplots(2, 1, figsize=(10, 6))
fig.tight_layout(pad=3)

# --- NEW ADDITIONS START: UI Setup for Stats Corner ---
# Adjust figure to make room for the text box on the right
plt.subplots_adjust(right=0.75)
stats_box = fig.text(0.77, 0.85, "", fontsize=10, fontfamily='monospace',
                     verticalalignment='top',
                     bbox=dict(boxstyle='round,pad=0.5', facecolor='lightgrey', alpha=0.5))
# --- NEW ADDITIONS STOP ---

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
    
    # --- NEW ADDITIONS START: Lock Y-Axis Bounds ---
    ax[0].set_ylim(-1.5, 1.5)
    # --- NEW ADDITIONS STOP ---

    ax[1].clear()
    ax[1].plot(temperature_vals, label='Temperature', color='orange')
    ax[1].set_title("Temperature")
    ax[1].legend()
    ax[1].grid(True)
    
    # --- NEW ADDITIONS START: Update Stats Corner ---
    if not collecting_integration_data:
        stats_text = "Warming up..."
    else:
        stats_text =  "Time-Avg Gravity\n"
        stats_text += "-" * 18 + "\n\n"
        stats_text += f"Current: {current_running_g:.5f} g\n\n"
        
        for name in milestone_targets.keys():
            val = milestone_results[name]
            display_val = f"{val:.5f} g" if val is not None else "Pending..."
            stats_text += f"{name:<9}: {display_val}\n"
            
    stats_box.set_text(stats_text)
    # --- NEW ADDITIONS STOP ---

# --- Start Live Plot ---
ani = animation.FuncAnimation(fig, animate, interval=500, cache_frame_data=False)
sys.modules[__name__].ani = ani  # Prevent garbage collection

plt.show()

# --- Cleanup ---
accelerometer.close()
temp_sensor.close()
csv_file.close()
