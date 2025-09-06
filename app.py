import streamlit as st
import numpy as np
import time
import pandas as pd
from collections import deque
import altair as alt  # Import the new library

# --- Page Configuration ---
st.set_page_config(
    page_title="PID Self-Balancing Robot",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="expanded"
)

# --- PID Controller Class (From your version) ---
class PIDController:
    """Simple PID controller implementation"""
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        
    def compute(self, measurement, dt):
        """Compute PID output"""
        error = measurement - self.setpoint 
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        d_term = self.kd * derivative
        
        # Update previous error
        self.prev_error = error
        
        # Calculate output
        return p_term + i_term + d_term
    
    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.prev_error = 0.0

# --- Robot Class (From your version, slightly adapted) ---
class BalancingRobot:
    """Self-balancing robot simulation"""
    def __init__(self):
        # Physical parameters
        self.mass = 1.0
        self.length = 0.5
        self.gravity = 9.81
        self.damping = 0.1
        
        # State variables
        self.angle = 0.01
        self.angular_velocity = 0.0
        self.position = 0.0
        self.velocity = 0.0
        
        # Control
        self.max_force = 25.0
        
    def apply_disturbance(self, force):
        """Apply external disturbance force"""
        self.angular_velocity += force * 0.1 

    def update(self, control_force, dt):
        """Update robot state using physics equations"""
        sin_theta = np.sin(self.angle)
        cos_theta = np.cos(self.angle)
        
        angular_acc = (self.gravity / self.length) * sin_theta
        angular_acc -= (self.damping / (self.mass * self.length**2)) * self.angular_velocity
        angular_acc -= (control_force * cos_theta) / (self.mass * self.length)
        
        self.angular_velocity += angular_acc * dt
        self.angle += self.angular_velocity * dt
        
        cart_acc = control_force / self.mass
        self.velocity += cart_acc * dt
        self.position += self.velocity * dt
        
        self.velocity *= 0.99

# --- Visualization ---
def draw_robot(robot_state):
    """Generates an SVG string to draw the cart and pole."""
    cart_w, cart_h, pole_w, pole_l = 100, 60, 15, 250
    cart_x = 400 + robot_state['position'] * 150
    # Keep the cart within the visible area of the SVG viewBox
    cart_x = max(cart_w/2 + 10, min(800 - cart_w/2 - 10, cart_x))
    cart_y = 350
    angle_deg = -np.rad2deg(robot_state['angle'])

    # Using width="100%" and viewBox makes the SVG responsive
    return f"""
    <svg width="100%" height="400" viewBox="0 0 800 400" style="background-color:#282c34; border-radius: 10px; border: 1px solid #444;">
        <line x1="0" y1="{cart_y + cart_h/2}" x2="800" y2="{cart_y + cart_h/2}" style="stroke:#555; stroke-width:2; stroke-dasharray: 5,5;" />
        <rect x="{cart_x - cart_w/2}" y="{cart_y - cart_h/2}" width="{cart_w}" height="{cart_h}" rx="5" ry="5" style="fill:#61afef; stroke:#1c1e24; stroke-width:2;" />
        <circle cx="{cart_x - cart_w/4}" cy="{cart_y + cart_h/2}" r="12" fill="#333" stroke="#1c1e24" stroke-width="1.5" />
        <circle cx="{cart_x + cart_w/4}" cy="{cart_y + cart_h/2}" r="12" fill="#333" stroke="#1c1e24" stroke-width="1.5" />
        <g transform="rotate({angle_deg}, {cart_x}, {cart_y})">
            <rect x="{cart_x - pole_w/2}" y="{cart_y - pole_l}" width="{pole_w}" height="{pole_l}" rx="5" ry="5" style="fill:#e06c75; stroke:#1c1e24; stroke-width:2;" />
        </g>
    </svg>
    """

def draw_chart(history_df):
    """Generates an Altair chart from the history dataframe."""
    if history_df.empty:
        return alt.Chart(pd.DataFrame({'time':[], 'Value':[], 'Metric':[]})).mark_line().properties(height=300)

    df_melted = history_df.melt(id_vars=['time'], value_vars=['angle', 'position', 'force'],
                                var_name='Metric', value_name='Value')

    chart = alt.Chart(df_melted).mark_line().encode(
        x=alt.X('time:Q', title='Time (s)'),
        y=alt.Y('Value:Q', title='Value'),
        color=alt.Color('Metric:N', title='Metric', 
                        scale=alt.Scale(domain=['angle', 'force', 'position'],
                                        range=['#e06c75', '#98c379', '#61afef'])),
        tooltip=['time', 'Value', 'Metric']
    ).properties(height=300).interactive()
    
    return chart

# --- Main Application Logic ---
st.title("🤖 Self-Balancing Robot Simulation")
st.markdown("A classic inverted pendulum controlled by a PID controller. Tune the gains and poke the robot to test its stability.")

if 'robot' not in st.session_state:
    st.session_state.robot = BalancingRobot()
    st.session_state.pid = PIDController(50.0, 1.0, 10.0)
    st.session_state.running = False
    st.session_state.history = deque(maxlen=200)
    st.session_state.time = 0.0

# --- Sidebar Controls ---
with st.sidebar:
    st.header("PID Controller Gains")
    kp = st.slider("Proportional (Kp)", 0.0, 100.0, 50.0, 1.0)
    ki = st.slider("Integral (Ki)", 0.0, 50.0, 1.0, 0.1)
    kd = st.slider("Derivative (Kd)", 0.0, 50.0, 10.0, 0.5)

    st.session_state.pid.kp, st.session_state.pid.ki, st.session_state.pid.kd = kp, ki, kd

    st.header("Simulation Control")
    if st.button("▶️ Start / ⏸️ Pause", use_container_width=True, type="primary"):
        st.session_state.running = not st.session_state.running

    if st.button("🔄 Reset Simulation", use_container_width=True):
        st.session_state.robot = BalancingRobot()
        st.session_state.pid.reset()
        st.session_state.running = False
        st.session_state.history = deque(maxlen=200)
        st.session_state.time = 0.0
        st.rerun()

    st.header("Actions")
    col1, col2 = st.columns(2)
    if col1.button("👈 Poke Left", use_container_width=True):
        st.session_state.robot.apply_disturbance(-5.0)
    if col2.button("Poke Right 👉", use_container_width=True):
        st.session_state.robot.apply_disturbance(5.0)

# --- Main Area ---
# Create side-by-side columns with a large gap for padding
col1, col2 = st.columns([1.5, 1], gap="large")

with col1:
    robot_placeholder = st.empty()
with col2:
    chart_placeholder = st.empty()

# --- Simulation & Rendering ---
robot = st.session_state.robot
pid = st.session_state.pid
dt = 0.02

if st.session_state.running:
    control_signal = pid.compute(robot.angle, dt)
    control_force = np.clip(control_signal, -robot.max_force, robot.max_force)
    robot.update(control_force, dt)
    
    st.session_state.time += dt
    st.session_state.history.append({
        'time': st.session_state.time,
        'angle': np.rad2deg(robot.angle),
        'position': robot.position,
        'force': control_force
    })

    if abs(robot.angle) > np.radians(45):
        st.session_state.running = False
        st.toast("💥 Robot fell over! Resetting.", icon="🔥")
        time.sleep(1)
        st.session_state.robot = BalancingRobot()
        st.session_state.pid.reset()
        st.session_state.history = deque(maxlen=200)
        st.session_state.time = 0.0
        st.rerun()

# --- Always Draw ---
robot_state_dict = {'angle': robot.angle, 'position': robot.position}
robot_placeholder.markdown(draw_robot(robot_state_dict), unsafe_allow_html=True)

history_df = pd.DataFrame(list(st.session_state.history))
chart_placeholder.altair_chart(draw_chart(history_df), use_container_width=True)

if st.session_state.running:
    time.sleep(dt)
    st.rerun()


