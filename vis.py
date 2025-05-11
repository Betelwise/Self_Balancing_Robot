import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import math

# --- PID Controller Class (same as before) ---
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint, dt, output_min, output_max):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.dt = dt
        self.integral = 0.0
        self.previous_error = 0.0
        self.output_min = output_min
        self.output_max = output_max
        # Store P, I, D terms for plotting
        self.P_out = 0
        self.I_out = 0
        self.D_out = 0

    def update(self, current_value):
        error = self.setpoint - current_value
        self.P_out = self.Kp * error
        self.integral += error * self.dt
        integral_limit_factor = 50.0
        if self.Ki != 0:
            max_integral = integral_limit_factor / abs(self.Ki) # abs for safety
            min_integral = -integral_limit_factor / abs(self.Ki)
            self.integral = max(min(self.integral, max_integral), min_integral)
        self.I_out = self.Ki * self.integral
        derivative = (error - self.previous_error) / self.dt
        self.D_out = self.Kd * derivative
        self.previous_error = error
        output = self.P_out + self.I_out + self.D_out
        output = max(min(output, self.output_max), self.output_min)
        return output

# --- Simplified Robot Physics Model (with disturbance input) ---
class RobotModel:
    def __init__(self, initial_angle_deg, dt):
        self.angle_rad = math.radians(initial_angle_deg)
        self.angular_velocity_rad_s = 0.0
        self.dt = dt
        self.motor_effectiveness = 25.0
        self.gravity_effect = 30.0
        self.damping = 0.05 # A bit of natural damping

    def update(self, motor_output, disturbance_torque=0.0):
        accel_gravity = self.gravity_effect * math.sin(self.angle_rad)
        accel_motor = -self.motor_effectiveness * motor_output # Motor corrects angle
        
        total_angular_acceleration = accel_gravity + accel_motor + disturbance_torque
        
        self.angular_velocity_rad_s += total_angular_acceleration * self.dt
        self.angular_velocity_rad_s *= (1 - self.damping * self.dt) # Apply damping
        self.angle_rad += self.angular_velocity_rad_s * self.dt
        
        return math.degrees(self.angle_rad)

# --- Simulation Parameters ---
simulation_duration_s = 15.0
dt = 0.02  # Time step for simulation AND animation frame rate
frames = int(simulation_duration_s / dt)

Kp = 40.0
Ki = 30.0
Kd = 35.0
setpoint_angle_deg = 0.0
initial_angle_deg = 10.0 # Start tilted

motor_output_min = -10.0
motor_output_max = 10.0

# Disturbance parameters
disturbance_start_time_s = 3.0
disturbance_duration_s = 0.2
disturbance_magnitude = 30.0 # Arbitrary torque units

# --- Initialize ---
pid = PIDController(Kp, Ki, Kd, setpoint_angle_deg, dt, motor_output_min, motor_output_max)
robot = RobotModel(initial_angle_deg, dt)

# Data storage for plots
time_history = np.linspace(0, simulation_duration_s, frames)
angle_history = np.zeros(frames)
motor_output_history = np.zeros(frames)
p_term_history = np.zeros(frames)
i_term_history = np.zeros(frames)
d_term_history = np.zeros(frames)

# --- Setup the Figure and Axes for Animation and Plots ---
fig = plt.figure(figsize=(14, 8))
gs = fig.add_gridspec(2, 2) # 2 rows, 2 columns

# Ax1: Robot Animation
ax_robot = fig.add_subplot(gs[:, 0]) # Spans both rows, first column
ax_robot.set_xlim(-1.5, 1.5)
ax_robot.set_ylim(-0.2, 2.2)
ax_robot.set_aspect('equal', adjustable='box')
ax_robot.set_title('Robot Animation')
ax_robot.set_xticks([])
ax_robot.set_yticks([])
ground = ax_robot.axhline(0, color='black', lw=2)

# Robot visual elements
chassis_width = 0.6
chassis_height = 0.3
wheel_radius = 0.15
body_length = 1.0

chassis_center_x = 0
chassis_center_y = wheel_radius + chassis_height / 2

robot_chassis = patches.Rectangle((chassis_center_x - chassis_width/2, wheel_radius), chassis_width, chassis_height, fc='deepskyblue')
ax_robot.add_patch(robot_chassis)
wheel_left = patches.Circle((chassis_center_x - chassis_width/2, wheel_radius), wheel_radius, fc='dimgray')
ax_robot.add_patch(wheel_left)
wheel_right = patches.Circle((chassis_center_x + chassis_width/2, wheel_radius), wheel_radius, fc='dimgray')
ax_robot.add_patch(wheel_right)

# Body line: starts from center top of chassis
body_pivot_x = chassis_center_x
body_pivot_y = wheel_radius + chassis_height
robot_body, = ax_robot.plot([], [], 'r-', lw=3, label='Robot Body') # Line2D object
angle_text = ax_robot.text(0.05, 0.95, '', transform=ax_robot.transAxes, ha='left', va='top')

# Ax2: Angle Plot
ax_angle = fig.add_subplot(gs[0, 1]) # First row, second column
line_angle, = ax_angle.plot([], [], lw=2, label='Angle (deg)')
ax_angle.axhline(setpoint_angle_deg, color='r', linestyle='--', label='Setpoint')
ax_angle.set_ylabel('Angle (degrees)')
ax_angle.set_title('System Response')
ax_angle.legend(loc='upper right')
ax_angle.grid(True)
ax_angle.set_ylim(-max(30, abs(initial_angle_deg)*1.5), max(30, abs(initial_angle_deg)*1.5)) # Dynamic Y lim

# Ax3: Motor Output and PID Terms Plot
ax_motor = fig.add_subplot(gs[1, 1], sharex=ax_angle) # Second row, second column
line_motor, = ax_motor.plot([], [], lw=2, label='Motor Output', color='g')
line_p, = ax_motor.plot([], [], lw=1, label='P Term', linestyle=':', color='purple')
line_i, = ax_motor.plot([], [], lw=1, label='I Term', linestyle=':', color='orange')
line_d, = ax_motor.plot([], [], lw=1, label='D Term', linestyle=':', color='brown')
ax_motor.set_ylabel('Output / Term Value')
ax_motor.set_xlabel('Time (seconds)')
ax_motor.legend(loc='upper right')
ax_motor.grid(True)
ax_motor.set_ylim(motor_output_min*1.5, motor_output_max*1.5)


# --- Animation Initialization Function ---
def init():
    robot_body.set_data([], [])
    angle_text.set_text('')
    
    line_angle.set_data([], [])
    line_motor.set_data([], [])
    line_p.set_data([],[])
    line_i.set_data([],[])
    line_d.set_data([],[])
    
    # Set x-limits for time-based plots
    ax_angle.set_xlim(0, simulation_duration_s)
    ax_motor.set_xlim(0, simulation_duration_s)
    
    return robot_body, angle_text, line_angle, line_motor, line_p, line_i, line_d

# --- Animation Update Function ---
current_angle_deg_global = initial_angle_deg # To pass value to animation

def update(frame_num):
    global current_angle_deg_global # Use the global variable

    t = time_history[frame_num]
    
    # Apply disturbance
    current_disturbance = 0.0
    if t >= disturbance_start_time_s and t < (disturbance_start_time_s + disturbance_duration_s):
        current_disturbance = disturbance_magnitude
        if frame_num > 0 and time_history[frame_num-1] < disturbance_start_time_s: # Print only at start
            print(f"Applying disturbance at t={t:.2f}s")

    # PID Control
    motor_cmd = pid.update(current_angle_deg_global)
    
    # Update Robot Model
    current_angle_deg_global = robot.update(motor_cmd, current_disturbance)
    
    # Store history
    angle_history[frame_num] = current_angle_deg_global
    motor_output_history[frame_num] = motor_cmd
    p_term_history[frame_num] = pid.P_out
    i_term_history[frame_num] = pid.I_out
    d_term_history[frame_num] = pid.D_out

    # Update Robot Body visual
    angle_rad_viz = math.radians(current_angle_deg_global) # Angle for visualization
    body_end_x = body_pivot_x + body_length * math.sin(angle_rad_viz)
    body_end_y = body_pivot_y + body_length * math.cos(angle_rad_viz)
    robot_body.set_data([body_pivot_x, body_end_x], [body_pivot_y, body_end_y])
    angle_text.set_text(f'Angle: {current_angle_deg_global:.2f}°')

    # Update Plots
    # Plot only up to current frame to see them draw live
    current_time_slice = time_history[:frame_num+1]
    line_angle.set_data(current_time_slice, angle_history[:frame_num+1])
    line_motor.set_data(current_time_slice, motor_output_history[:frame_num+1])
    line_p.set_data(current_time_slice, p_term_history[:frame_num+1])
    line_i.set_data(current_time_slice, i_term_history[:frame_num+1])
    line_d.set_data(current_time_slice, d_term_history[:frame_num+1])
    
    # Check for fall
    if abs(current_angle_deg_global) > 85: # Stop animation if it falls too much
        print(f"Robot fell significantly at t={t:.2f}s. Stopping animation.")
        ani.event_source.stop() # Stop the animation

    return robot_body, angle_text, line_angle, line_motor, line_p, line_i, line_d


# --- Create and Run Animation ---
# blit=True can make it smoother but can be finicky with complex plots / text updates
# If animation is problematic, try blit=False
ani = animation.FuncAnimation(fig, update, frames=frames,
                              init_func=init, blit=True, interval=dt*1000, repeat=False)

plt.tight_layout(pad=2.0)
plt.show()

print("Simulation finished.")