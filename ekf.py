
import math
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import sys
from matplotlib.patches import Ellipse

# matplotlib.style.use('ggplot')
matplotlib.style.use('default')

# Simulation parameters
dt = 0.2  # delta_t, in second (s)
total_simulation_time = 65.0  # total simulation time, in second (s)

# Process noise covariance matrix
Q = np.diag([0.15, 0.15, np.deg2rad(1.0), 1.2]) 

# Observation noise covariance matrix
R = np.diag([1.5, 1.5]) ** 2

def read_sensors(x_true, u):

    control_input_noise = np.diag([1.20, np.deg2rad(45.0)]) ** 2
    gps_noise = np.diag([1.0, 1.0]) ** 2

    x_true = motion_function_f(x_true, u)

    # Add noise to gps x-y position
    z = observation_function_h(x_true) + gps_noise @ np.random.randn(2, 1)

    # Add noise to control input
    u = u + control_input_noise @ np.random.randn(2, 1)

    return x_true, z, u

def motion_function_f(x, u):

    x = np.array ([[x[0,0] + u[0,0] * math.cos(x[2,0])* dt], [x[1,0] + u[0,0] * math.sin(x[2,0])* dt], [x[2,0] + u[1,0] * dt], [u[0,0]] ])

    return x

def observation_function_h(x):

    z = x[:2]

    return z

#jacobian of motion function f
def jacobian_f(x, u): 

    Jf = np.array([[1, 0, -u[0,0] * math.sin(x[2,0]) * dt, math.cos(x[2,0]) * dt], [0, 1, u[0,0] * math.cos(x[2,0]) * dt, math.sin(x[2,0]) * dt], [0, 0, 1, 0], [0, 0, 0, 1]])

    return Jf

#jacobian of observation function h
def jacobian_h(x):

    Jh = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

    return Jh

def ekf(x_est, P_est, z, u):

    # Predict
    x_pred = motion_function_f(x_est, u) #The next state 'x_pred' is predicted using the motion model
    F = jacobian_f(x_pred, u) #The jacobian matrix of the modtion model 'Jf' is cacluated
    P_pred = F @ P_est @ F.T + Q #The covariance matrix 'P_pred' is updated 

    # Update
    H = jacobian_h(x_pred)  # Calculates the Jacobian matrix of the observation model JH
    z_pred = observation_function_h(x_pred)  # Predicts sensor readings based on the predicted state z_pred
    K = P_pred @ H.T @ np.linalg.inv(H @ P_pred @ H.T + R)  # Calculates the Kalman gain
    x_est = x_pred + K @ (z - z_pred) # Updates the state estimate 'x_est' using the Kalman gain 'K'
    P_est = (np.eye(len(x_est)) - K @ H) @ P_pred  # Updates the covariance matrix 'P_est'
    return x_est, P_est

def run_ekf():
    print(" Simulation has started ...")

    # Initialize state Vector and covariance
    x_est = np.array([5.0, 5.0, 0.0, 0.0]).reshape(4, 1) # Initial estimated state
    x_true = np.array([5.0, 5.0, 0.0, 0.0]).reshape(4, 1) # Initial true state
    P_est = 1.15 * np.eye(4)  # Initial covariance matrix.

    # Initialise lists to store visualisation data
    x_est_array = []
    x_true_array = []
    z_array = []

    #Setting the size of the graph to be made
    plt.figure(figsize=(50, 30))

    for _ in np.arange(0, total_simulation_time, dt):

        # Control input 
        u = np.array([[0.95], [-0.085]]) # v is in m/s and omega is grad/s.

        # Get sensor readings
        x_true, z, u = read_sensors(x_true, u)

        # Apply EKF
        x_est, P_est = ekf(x_est, P_est, z, u)

        # Store visualisation data
        x_est_array.append(x_est.flatten())
        x_true_array.append(x_true.flatten())
        z_array.append(z.flatten())

        # Visualize the estimated covariance ellipse
        visualize_covariance_ellipse(x_est, P_est)

    # Converts visualisation lists to arrays
    x_est_array = np.array(x_est_array).T
    x_true_array = np.array(x_true_array).T
    z_array = np.array(z_array).T

    # Plot (ground-truth) True Trajectory
    plt.plot(x_true_array[0], x_true_array[1], color='blue')
    # Plot Estimated Trajectory
    plt.plot(x_est_array[0], x_est_array[1], color='red')
    # Plot Observation Trajectory
    plt.scatter(z_array[0], z_array[1], color='green', marker='o', s=30)

    # Set axis labels and legend
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.legend()
    plt.show()

    return x_est, P_est

def visualize_covariance_ellipse(x_est, P_est):  #This function is for visualising the estimated covariance ellipse

    # Extract position and covariance information
    x, y = x_est[0, 0], x_est[1, 0]
    cov_matrix = P_est[:2, :2]

    # Calculate eigenvalues and eigenvectors of the covariance matrix
    eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)

    # Calculate ellipse angle
    angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))

    # Create an ellipse patch
    ellipse = Ellipse((x, y), 2 * np.sqrt(eigenvalues[0]), 2 * np.sqrt(eigenvalues[1]),
                      angle=angle, edgecolor='orange', facecolor='none', linestyle='dashed', linewidth=2)

    # Plot the ellipse on the current plot
    plt.gca().add_patch(ellipse)


# Run the run_ekf() function
x_est, P_est = run_ekf()