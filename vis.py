import numpy as np
import matplotlib.pyplot as plt
import math

# --- PID Controller Class ---
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint, dt, output_min, output_max):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.dt = dt  # Time step

        self.integral = 0.0
        self.previous_error = 0.0

        self.output_min = output_min
        self.output_max = output_max

    def update(self, current_value):
        error = self.setpoint - current_value

        # Proportional term
        P_out = self.Kp * error

        # Integral term (with anti-windup)
        self.integral += error * self.dt
        # Clamp integral to prevent windup, scaled by Ki to make limits more intuitive
        # These integral_limits are somewhat arbitrary, adjust as needed
        integral_limit_factor = 50.0 # How much the integral term can contribute relative to Kp=1
        if self.Ki != 0:
            max_integral = integral_limit_factor / self.Ki
            min_integral = -integral_limit_factor / self.Ki
            self.integral = max(min(self.integral, max_integral), min_integral)
        I_out = self.Ki * self.integral

        # Derivative term
        derivative = (error - self.previous_error) / self.dt
        D_out = self.Kd * derivative

        # Total output
        output = P_out + I_out + D_out

        # Store error for next iteration
        self.previous_error = error

        # Clamp output to min/max
        output = max(min(output, self.output_max), self.output_min)
        
        return output, P_out, I_out, D_out

# --- Simplified Robot Physics Model ---
class RobotModel:
    def __init__(self, initial_angle_deg, dt):
        self.angle_rad = math.radians(initial_angle_deg)
        self.angular_velocity_rad_s = 0.0  # rad/s
        self.dt = dt

        # These are highly simplified parameters. You'll need to "tune" them
        # to get behavior that feels somewhat like a real robot.
        # Represents how effective the motor is at generating torque/angular acceleration
        self.motor_effectiveness = 25.0 # Units: rad/s^2 per unit of motor output
        # Represents how strongly gravity pulls the robot down (related to m*g*l)
        self.gravity_effect = 30.0      # Units: rad/s^2 (at 90 degrees)

    def update(self, motor_output):
        # Torque (and thus angular acceleration) from gravity
        # Positive angle means tilted right, gravity pulls it further right (positive accel)
        # For self-balancing, gravity effect is usually sin(angle)
        accel_gravity = self.gravity_effect * math.sin(self.angle_rad)

        # Torque (and thus angular acceleration) from motor
        # Positive motor output tries to tilt it left (negative accel)
        accel_motor = -self.motor_effectiveness * motor_output

        # Net angular acceleration
        total_angular_acceleration = accel_gravity + accel_motor

        # Update angular velocity and angle (Euler integration)
        self.angular_velocity_rad_s += total_angular_acceleration * self.dt
        self.angle_rad += self.angular_velocity_rad_s * self.dt

        # Optional: Add some damping (friction)
        # self.angular_velocity_rad_s *= 0.99

        return math.degrees(self.angle_rad) # Return angle in degrees

# --- Simulation Parameters ---
simulation_time = 10.0  # seconds
dt = 0.01              # time step, seconds (should match PID dt)

# PID Gains (CRITICAL: These need tuning!)
# Start with Kp, then add Kd, then Ki if needed.
Kp = 35.0   # Proportional gain (e.g., 10-100)
Ki = 20.0    # Integral gain (e.g., 0-50)
Kd = 25.0    # Derivative gain (e.g., 5-50)

setpoint_angle_deg = 0.0  # Target angle (vertical)
initial_angle_deg = 5.0   # Start with a slight tilt

# Motor output limits (e.g., like Arduino PWM values or normalized)
# Let's use a range that allows for significant control authority
# These are unitless for the simulation but represent motor power
motor_output_min = -10.0 # Max reverse power
motor_output_max = 10.0  # Max forward power

# --- Initialize ---
pid = PIDController(Kp, Ki, Kd, setpoint_angle_deg, dt, motor_output_min, motor_output_max)
robot = RobotModel(initial_angle_deg, dt)

time_points = np.arange(0, simulation_time, dt)
angle_history = []
motor_output_history = []
p_term_history = []
i_term_history = []
d_term_history = []

# --- Simulation Loop ---
current_angle_deg = initial_angle_deg
for t in time_points:
    # Get PID control output
    motor_output, p_out, i_out, d_out = pid.update(current_angle_deg)

    # Update robot model
    current_angle_deg = robot.update(motor_output)

    # Store data for plotting
    angle_history.append(current_angle_deg)
    motor_output_history.append(motor_output)
    p_term_history.append(p_out)
    i_term_history.append(i_out)
    d_term_history.append(d_out)

    # Optional: Stop if robot falls too far
    if abs(current_angle_deg) > 90:
        print(f"Robot fell at t={t:.2f}s, angle={current_angle_deg:.2f} degrees")
        # Truncate history arrays to the point of falling
        idx_fall = len(angle_history)
        time_points = time_points[:idx_fall]
        angle_history = angle_history[:idx_fall]
        motor_output_history = motor_output_history[:idx_fall]
        p_term_history = p_term_history[:idx_fall]
        i_term_history = i_term_history[:idx_fall]
        d_term_history = d_term_history[:idx_fall]
        break

# --- Plotting Results ---
fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

# Plot 1: Angle
axs[0].plot(time_points, angle_history, label='Robot Angle (degrees)')
axs[0].axhline(setpoint_angle_deg, color='r', linestyle='--', label='Setpoint')
axs[0].set_ylabel('Angle (degrees)')
axs[0].set_title('Self-Balancing Robot Simulation')
axs[0].legend()
axs[0].grid(True)

# Plot 2: Motor Output
axs[1].plot(time_points, motor_output_history, label='Motor Output', color='g')
axs[1].axhline(0, color='gray', linestyle=':', linewidth=0.8)
axs[1].set_ylabel('Motor Output (arb. units)')
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
