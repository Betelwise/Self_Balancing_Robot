from graphviz import Digraph

# Create a new directed graph
dot = Digraph(comment='Self-Balancing Robot - Simplified Hardware v2', engine='dot')
# Use splines='line' for straight lines. Default is also straight if not specified.
dot.attr(rankdir='TB', splines='line', nodesep='0.7', ranksep='1.2', bgcolor='white')
dot.attr(compound='true') # Allows edges to connect to clusters

# --- Define Node Styles (Modern Look, White BG compatible) ---
node_attrs = {
    'shape': 'box',
    'style': 'filled,rounded',
    'fontname': 'Arial',
    'fontsize': '11',
    'margin': '0.3,0.15'
}

mcu_style = {'fillcolor': '#E3F2FD', 'color': '#90CAF9', **node_attrs} # Lighter Blue
sensor_style = {'fillcolor': '#FFF9C4', 'color': '#FFEE58', **node_attrs} # Lighter Yellow
driver_style = {'fillcolor': '#FFCDD2', 'color': '#EF9A9A', **node_attrs} # Lighter Red
motor_style = {'shape': 'ellipse', 'fillcolor': '#E1BEE7', 'color': '#CE93D8', **node_attrs} # Lighter Purple
power_style = {'shape': 'cylinder', 'fillcolor': '#C8E6C9', 'color': '#A5D6A7', **node_attrs} # Lighter Green

# --- Define Nodes (Components) ---
with dot.subgraph(name='cluster_control') as c:
    c.attr(label='Control Unit', style='rounded', color='darkgrey', fontname='Arial', fontsize='10') # No fill for cluster
    c.node('ARDUINO', 'Arduino Nano\n(MCU + 5V Source)', **mcu_style) # Indicate Arduino as 5V source
    c.node('MPU6050', 'MPU6050\n(IMU)', **sensor_style)

with dot.subgraph(name='cluster_actuation') as c:
    c.attr(label='Actuation System', style='rounded', color='darkgrey', fontname='Arial', fontsize='10')
    c.node('L298N', 'L298N\nMotor Driver', **driver_style)
    c.node('MOTOR_L', 'Left Motor', **motor_style)
    c.node('MOTOR_R', 'Right Motor', **motor_style)

dot.node('BATTERY', 'Battery\n(7.4V+)', **power_style)

# --- Define Edges (Connections) - Simplified ---

# --- Power Connections ---
# Arduino gets power (either USB not shown, or VIN from battery)
# For this diagram, let's show VIN from Battery as the primary non-USB power.
dot.edge('BATTERY', 'ARDUINO', label='VIN(+), GND(-)', style='solid', color='#4CAF50', penwidth='1.2', fontsize='9', fontcolor='#388E3C')

# Arduino provides 5V and GND to MPU6050
dot.edge('ARDUINO', 'MPU6050', label='5V, GND', color='#1E88E5', penwidth='1.2', fontsize='9', fontcolor='#1565C0')

# Arduino provides 5V (Logic) and GND to L298N
# (Assuming L298N's 5V_EN jumper is OFF)
dot.edge('ARDUINO', 'L298N', label='5V (Logic), GND', color='#1E88E5', penwidth='1.2', fontsize='9', fontcolor='#1565C0')

# Battery provides high power and GND to L298N for motors
dot.edge('BATTERY', 'L298N', label='Motor Supply (+), GND (-)', color='#F44336', penwidth='1.5', fontsize='9', fontcolor='#D32F2F')


# --- Signal Bundles (thicker lines) ---
# MPU6050 to Arduino (I2C + INT)
dot.edge('ARDUINO', 'MPU6050', label='I²C (SDA, SCL)\n+ INT', dir='both', penwidth='2.0', color='#007ACC', fontcolor='#00568F', fontsize='10')

# L298N Control Signals from Arduino
dot.edge('ARDUINO', 'L298N', label='Motor Control Signals\n(6 Wires: ENA/B, IN1-4)', penwidth='2.0', color='#FF5722', fontcolor='#E64A19', fontsize='10')

# L298N to Motors
dot.edge('L298N', 'MOTOR_L', label='Motor Power\n(2 Wires)', penwidth='2.0', color='#673AB7', fontcolor='#512DA8', fontsize='10')
dot.edge('L298N', 'MOTOR_R', label='Motor Power\n(2 Wires)', penwidth='2.0', color='#673AB7', fontcolor='#512DA8', fontsize='10')


# --- Add a general note about Ground ---
dot.node('NOTE_GROUND', 'Note: All GND pins are interconnected to form a common ground.',
         shape='plaintext', fontname='Arial', fontsize='9', fontcolor='#555555', pos="1,0!") # Try to position it at bottom

# --- Render and Save the Diagram ---
output_filename = 'robot_hardware_connections_simplified_v2'
# To ensure straight lines, remove 'splines' or set to 'line'.
# If `splines` is not set, Graphviz defaults to straight lines for 'dot' engine.
dot.graph_attr.pop('splines', None) # Make sure it's not using a previous spline setting

dot.render(output_filename, view=True, format='png')

print(f"Simplified flowchart (v2) saved as {output_filename}.png (and .gv source file)")
const int ENA_PIN = 9;  // PWM for Speed Control Motor A
const int IN1_PIN = 4;  // Direction Control Motor A
const int IN2_PIN = 5;  // Direction Control Motor A

// Motor B (Right Motor)
const int ENB_PIN = 3;  // PWM for Speed Control Motor B
const int IN3_PIN = 6;  // Direction Control Motor B
const int IN4_PIN = 8;  // Direction Control Motor B