import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd
import sys

# ==========================================
# 1. CONTROLLER SELECTION MENU
# ==========================================
print("="*40)
print("  DC MOTOR CONTROLLER TRACKING SYSTEM")
print("="*40)
print("[1] INVERSE BASED (Command Tracking)")
print("[2] LOOP-SHAPING (Disturbance Rejection)")
print("[3] 2-DoF (Best of Both Worlds)")
print("[4] H_inf (Robust Control)")
print("="*40)

choice = input("Select Controller (1-4): ").strip()

controller_names = {
    "1": "Inverse-Based",
    "2": "Loop-Shaping",
    "3": "2-DoF",
    "4": "H-Infinity"
}

if choice not in controller_names:
    print("Invalid choice. Exiting...")
    sys.exit()

controller_title = controller_names[choice]
print(f"\nSelected: {controller_title}. Starting connection...")

# ==========================================
# 2. SERIAL SETTINGS (Change the COM port!)
# ==========================================
PORT = 'COM8'
BAUD_RATE = 9600

time_data, target_data, rpm_data, pwm_data = [], [], [], []
disturbance_data = []   # binary log: 1 = disturbance applied, 0 = none

try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=0.1)
    # Wait 2 seconds for the Arduino to reset (very important)
    time.sleep(2)

    # Send the Mode to the Arduino (e.g. "M3\n")
    command = f"M{choice}\n"
    ser.write(command.encode('utf-8'))

    # Send initial target (20 RPM)
    ser.write("T20.0\n".encode('utf-8'))

    print(f"Connected to {PORT} successfully!")
    print("Live Plotting Started. Close window to STOP and SAVE.")
except Exception as e:
    print(f"Error connecting to {PORT}. Is the IDE Serial Monitor closed?\n{e}")
    sys.exit()

# ==========================================
# 3. DISTURBANCE CAPTURE (hold 'D' on the plot window)
# ==========================================
# While the 'D' key is held down on the plot window, every incoming
# data sample is tagged with disturbance = 1. When released it returns to 0.
# NOTE: the plot window must have keyboard focus for this to register.
# This press/release logic is reliable on Windows; on Linux/X11 you may
# need a timeout-based scheme due to key auto-repeat generating fake releases.
disturbance_active = False

def on_key_press(event):
    global disturbance_active
    if event.key in ('d', 'D'):
        disturbance_active = True

def on_key_release(event):
    global disturbance_active
    if event.key in ('d', 'D'):
        disturbance_active = False

# ==========================================
# 4. PLOT SETUP WITH DYNAMIC TITLES
# ==========================================
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 11), sharex=True)
fig.canvas.manager.set_window_title(f'DC Motor Live - {controller_title}')

# Register the keyboard handlers for disturbance capture
fig.canvas.mpl_connect('key_press_event', on_key_press)
fig.canvas.mpl_connect('key_release_event', on_key_release)

line_target, = ax1.plot([], [], 'k--', label='Target RPM', linewidth=2)
line_rpm, = ax1.plot([], [], 'b-', label='Filtered RPM', linewidth=2)
line_pwm, = ax2.plot([], [], 'r-', label='Controller PWM', linewidth=2)
line_dist, = ax3.plot([], [], 'm-', label='Disturbance', linewidth=2,
                      drawstyle='steps-post')

ax1.set_title(f'Motor Speed Control - [{controller_title} Controller]',
              fontsize=14, fontweight='bold')
ax1.set_ylabel('Speed (RPM)')
ax1.legend()
ax1.grid(True, linestyle='--', alpha=0.7)

ax2.set_title('Control Effort (PWM)', fontsize=12)
ax2.set_ylabel('PWM Signal')
ax2.axhline(0, color='black', linewidth=1)
ax2.legend()
ax2.grid(True, linestyle='--', alpha=0.7)

ax3.set_title("Disturbance Flag  (hold 'D' on this window to mark)", fontsize=12)
ax3.set_xlabel('Time (seconds)')
ax3.set_ylabel('Disturbance')
ax3.set_ylim(-0.1, 1.15)
ax3.set_yticks([0, 1])
ax3.set_yticklabels(['0 (OFF)', '1 (ON)'])
ax3.legend(loc='upper right')
ax3.grid(True, linestyle='--', alpha=0.7)

# Live ON/OFF status indicator on the disturbance axis
status_text = ax3.text(0.02, 0.78, '', transform=ax3.transAxes,
                       fontsize=11, fontweight='bold')

dist_fill = None  # holds the shaded fill collection between frames
start_time = time.time()

def update_plot(frame):
    global dist_fill
    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8').strip()
            if "Target:" not in line or "Filtered_RPM:" not in line:
                continue

            parts = line.split(',')
            target = float(parts[0].split(':')[1])
            rpm = float(parts[1].split(':')[1])
            pwm = float(parts[2].split(':')[1])

            current_time = time.time() - start_time
            time_data.append(current_time)
            target_data.append(target)
            rpm_data.append(rpm)
            pwm_data.append(pwm)
            # Tag this sample with the current disturbance state
            disturbance_data.append(1 if disturbance_active else 0)
        except Exception:
            pass

    if len(time_data) > 0:
        line_target.set_data(time_data, target_data)
        line_rpm.set_data(time_data, rpm_data)
        line_pwm.set_data(time_data, pwm_data)
        line_dist.set_data(time_data, disturbance_data)

        # Shade the active disturbance regions for clarity
        if dist_fill is not None:
            dist_fill.remove()
        dist_fill = ax3.fill_between(time_data, disturbance_data, step='post',
                                     color='m', alpha=0.25)

        ax1.set_xlim(0, time_data[-1] + 1)
        ax1.set_ylim(min(0, min(target_data) - 5), max(target_data) + 10)
        ax2.set_ylim(-260, 260)
        # ax3 shares the x-axis with ax1 (sharex=True), so limits track it

    # Update the live ON/OFF indicator
    if disturbance_active:
        status_text.set_text('DISTURBANCE: ON')
        status_text.set_color('red')
    else:
        status_text.set_text('DISTURBANCE: OFF')
        status_text.set_color('green')

    return line_target, line_rpm, line_pwm, line_dist

try:
    ani = animation.FuncAnimation(fig, update_plot, interval=50,
                                  cache_frame_data=False)
    plt.tight_layout()
    plt.show()

except KeyboardInterrupt:
    pass

# ==========================================
# 5. SAVE WITH THE CONTROLLER'S NAME
# ==========================================
finally:
    ser.close()
    if len(time_data) > 0:
        print(f"\nSaving Data for {controller_title}...")

        # Replace spaces/dashes with underscores for clean file names
        safe_name = controller_title.replace(" ", "_").replace("-", "_")

        fig.savefig(f'Response_{safe_name}.png', dpi=300)
        print(f"-> Plot saved as 'Response_{safe_name}.png'")

        df = pd.DataFrame({
            'Time(s)': time_data,
            'Target_RPM': target_data,
            'Filtered_RPM': rpm_data,
            'PWM': pwm_data,
            'Disturbance': disturbance_data
        })
        df.to_csv(f'Data_{safe_name}.csv', index=False)
        print(f"-> Data saved as 'Data_{safe_name}.csv'")
        print("Done! On to the next one!")