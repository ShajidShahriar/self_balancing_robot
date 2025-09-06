

# PID Self-Balancing Robot Simulation

A web-based, interactive simulation of a classic inverted pendulum problem, representing a self-balancing robot. This application is built with Python and Streamlit, allowing users to tune a PID controller in real-time and observe its effect on the robot's stability.

**Live Demo:** [https://selfbalancingrobot-s6ulbezssyo6aufvat6bcn.streamlit.app/](https://selfbalancingrobot-s6ulbezssyo6aufvat6bcn.streamlit.app/)

## Description

This project simulates the physics of an inverted pendulum on a cart. The goal is to keep the pendulum (the "robot") upright by applying a horizontal force, which is calculated by a Proportional-Integral-Derivative (PID) controller. The application provides an intuitive visual interface to understand the core principles of control systems and the role of PID gains in system stability and response.

## Features

  * **Real-time Physics Simulation:** The robot's angle and position are updated based on a simplified physics model.
  * **Interactive PID Tuning:** Adjust Proportional (Kp), Integral (Ki), and Derivative (Kd) gains using sliders and immediately see the impact on the robot's behavior.
  * **External Disturbance:** Apply a "poke" force to test the controller's ability to recover from external disturbances.
  * **Live Data Visualization:** An Altair chart plots the robot's angle, position, and the control force over time.
  * **Simulation Controls:** Start, pause, and reset the simulation at any time.

## Technologies Used

  * **Backend:** Python
  * **Web Framework:** Streamlit
  * **Data Manipulation:** Pandas, NumPy
  * **Charting:** Altair

## How to Run Locally

To run this simulation on your local machine, follow these steps:

1.  **Clone the repository:**

    ```bash
    git clone https://github.com/ShajidShahriar/self_balancing_robot.git
    
    ```

2.  **Create and activate a virtual environment (recommended):**

    ```bash
    python -m venv venv
    # On Windows
    venv\Scripts\activate
    # On macOS/Linux
    source venv/bin/activate
    ```

3.  **Install the required dependencies:**

    ```bash
    pip install -r requirements.txt
    ```

4.  **Run the Streamlit application:**

    ```bash
    streamlit run app.py
    ```

The application will open in your default web browser.
