import numpy as np
import matplotlib.pyplot as plt
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
        self.p_out, self.i_out, self.d_out = 0,0,0 # For logging

    def update(self, current_value):
        error = self.setpoint - current_value

        self.p_out = self.Kp * error

        self.integral += error * self.dt
        # Simple anti-windup: Clamp integral term contribution
        # This limit is somewhat arbitrary; adjust if I_term gets stuck too high/low
        max_integral_contribution = abs(self.output_max * 0.5) # e.g., integral can contribute up to 50% of max output
        if self.Ki != 0:
             clamped_integral = max(min(self.integral, max_integral_contribution / self.Ki), -max_integral_contribution / self.Ki)
             self.i_out = self.Ki * clamped_integral
        else:
            self.i_out = 0


        derivative = (error - self.previous_error) / self.dt
        self.d_out = self.Kd * derivative

        output = self.p_out + self.i_out + self.d_out
        self.previous_error = error
        output = max(min(output, self.output_max), self.output_min)
        
        return output

    def get_terms(self):
        return self.p_out, self.i_out, self.d_out

# --- Balancing Robot Physics Model ---
class BalancingRobot:
    def __init__(self, initial_angle_deg, dt,
                 mass_kg=1.0,  # Mass of the robot
                 com_height_m=0.1,  # Height of Center of Mass from axle
                 moment_of_inertia_kgm2=0.01, # Moment of inertia about the axle
                 motor_torque_constant=0.5 # Nm per unit of PID output
                ):
        self.angle_rad = math.radians(initial_angle_deg)
        self.angular_velocity_rad_s = 0.0  # rad/s
        self.dt = dt

        self.mass_kg = mass_kg
        self.com_height_m = com_height_m # Center of Mass height from axle
        self.g_ms2 = 9.81 # Acceleration due to gravity
        self.moment_of_inertia_kgm2 = moment_of_inertia_kgm2 # About the wheel axle
        self.motor_torque_constant = motor_torque_constant # Relates PID output to actual motor torque

        # Optional: Add some natural damping (friction, motor back-EMF effects)
        self.damping_coefficient = 0.01 # Small damping

    def update_physics(self, pid_output_command, external_disturbance_torque=0.0):
        # Torque from gravity trying to pull it down
        # Positive angle means tilted "forward/right", gravity pulls it more forward/right
        torque_gravity = self.mass_kg * self.g_ms2 * self.com_height_m * math.sin(self.angle_rad)

        # Torque from motors trying to correct
        # Positive PID output should generate torque to make angle decrease (tilt "backward/left")
        torque_motor = self.motor_torque_constant * pid_output_command # Negative sign depends on convention

        # Damping torque (opposes angular velocity)
        torque_damping = -self.damping_coefficient * self.angular_velocity_rad_s

        # Net torque
        net_torque = torque_gravity + torque_motor + torque_damping + external_disturbance_torque

        # Angular acceleration (alpha = Torque / I)
        angular_acceleration_rad_s2 = net_torque / self.moment_of_inertia_kgm2

        # Update angular velocity and angle (Euler integration)
        self.angular_velocity_rad_s += angular_acceleration_rad_s2 * self.dt
        self.angle_rad += self.angular_velocity_rad_s * self.dt
        
        return math.degrees(self.angle_rad) # Return angle in degrees

    def apply_angular_velocity_disturbance(self, disturbance_rad_s):
        """Directly adds to the angular velocity (like a sharp kick)."""
        self.angular_velocity_rad_s += disturbance_rad_s


# --- Simulation Parameters ---
simulation_time_s = 10.0
dt_s = 0.01 # Simulation time step (and PID dt)

# PID Gains (CRITICAL: These need careful tuning!)
# These values are illustrative and will likely need adjustment for your robot parameters
Kp = 6.0    # Proportional gain (try values 1-20)
Ki = 5.0    # Integral gain (try values 0-10)
Kd = 0.8    # Derivative gain (try values 0-5)

setpoint_angle_deg = 0.0  # Target angle (vertical)
initial_angle_deg = 2.0   # Start with a slight tilt

# Robot Physical Parameters (Adjust these to simulate different robots)
robot_mass_kg = 1.5
robot_com_height_m = 0.15 # Height of Center of Mass from axle
robot_moment_of_inertia_kgm2 = robot_mass_kg * robot_com_height_m**2 # Approximation for a rod pivoted at one end
# More realistic I: For a uniform rectangular body of height H, width W, mass M,
# rotating about an axle at its base: I = M * (H^2/3 + W^2/12) if axle is along width.
# For this simulation, let's use a simpler form or just an estimated value.
# robot_moment_of_inertia_kgm2 = 0.02

# Motor characteristics
# How much torque (Nm) is produced per unit of PID output command
# This bridges the unitless PID output to physical torque.
motor_torque_conversion = 0.1 # e.g. if PID output is 10, motor torque is 1 Nm

# PID output limits (unitless, but scaled by motor_torque_conversion)
pid_output_min = -20.0
pid_output_max = 20.0

# Disturbance parameters
disturbance_time_s = 3.0  # When to apply the "push"
disturbance_strength_rad_s = math.radians(300.0) # Impart an angular velocity of 30 deg/s

# --- Initialize ---
pid = PIDController(Kp, Ki, Kd, setpoint_angle_deg, dt_s, pid_output_min, pid_output_max)
robot = BalancingRobot(initial_angle_deg, dt_s,
                       mass_kg=robot_mass_kg,
                       com_height_m=robot_com_height_m,
                       moment_of_inertia_kgm2=robot_moment_of_inertia_kgm2,
                       motor_torque_constant=motor_torque_conversion)

time_points = np.arange(0, simulation_time_s, dt_s)
angle_history_deg = []
pid_output_history = []
p_term_history, i_term_history, d_term_history = [], [], []
disturbance_applied_flag = False

# --- Simulation Loop ---
current_angle_deg = initial_angle_deg
for t_s in time_points:
    # Apply disturbance
    if t_s >= disturbance_time_s and not disturbance_applied_flag:
        robot.apply_angular_velocity_disturbance(disturbance_strength_rad_s)
        print(f"Disturbance applied at t={t_s:.2f}s")
        disturbance_applied_flag = True

    # Get PID control command
    pid_command = pid.update(current_angle_deg)
    p, i, d = pid.get_terms()

    # Update robot physics
    # For simplicity, we are not using external_disturbance_torque here,
    # but it could be used for continuous disturbances.
    current_angle_deg = robot.update_physics(pid_command)

    # Store data for plotting
    angle_history_deg.append(current_angle_deg)
    pid_output_history.append(pid_command)
    p_term_history.append(p)
    i_term_history.append(i)
    d_term_history.append(d)

    # Stop if robot falls (angle too large)
    if abs(current_angle_deg) > 85: # Adjusted fall threshold
        print(f"Robot fell at t={t_s:.2f}s, angle={current_angle_deg:.2f} degrees")
        # Truncate history arrays to the point of falling
        idx_fall = len(angle_history_deg)
        time_points = time_points[:idx_fall]
        angle_history_deg = angle_history_deg[:idx_fall]
        pid_output_history = pid_output_history[:idx_fall]
        p_term_history = p_term_history[:idx_fall]
        i_term_history = i_term_history[:idx_fall]
        d_term_history = d_term_history[:idx_fall]
        break

# --- Plotting Results ---
fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

# Plot 1: Angle
axs[0].plot(time_points, angle_history_deg, label='Robot Angle (degrees)')
axs[0].axhline(setpoint_angle_deg, color='r', linestyle='--', label='Setpoint (0 deg)')
if disturbance_applied_flag and disturbance_time_s < time_points[-1] :
    axs[0].axvline(disturbance_time_s, color='orange', linestyle='-.', label='Disturbance Applied')
axs[0].set_ylabel('Angle (degrees)')
axs[0].set_title(f'Self-Balancing Robot Simulation (Kp={Kp}, Ki={Ki}, Kd={Kd})')
axs[0].legend()
axs[0].grid(True)
axs[0].set_ylim([-90, 90]) # Set Y-axis limits for angle

# Plot 2: PID Output (Motor Command)
axs[1].plot(time_points, pid_output_history, label='PID Output (Motor Command)', color='g')
axs[1].axhline(0, color='gray', linestyle=':', linewidth=0.8)
axs[1].set_ylabel('PID Output')
axs[1].legend()
axs[1].grid(True)

# Plot 3: PID Components
axs[2].plot(time_points, p_term_history, label='P Term', linestyle=':')
axs[2].plot(time_points, i_term_history, label='I Term', linestyle=':')
axs[2].plot(time_points, d_term_history, label='D Term', linestyle=':')
axs[2].axhline(0, color='gray', linestyle=':', linewidth=0.8)
axs[2].set_ylabel('PID Component Value')
axs[2].set_xlabel('Time (seconds)')
axs[2].legend()
axs[2].grid(True)

plt.tight_layout()
plt.show()