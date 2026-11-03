import numpy as np
import matplotlib.pyplot as plt

#Define Robot simulation parameters
v_left = 0.15
v_right = 0.25
wheel_radius = 0.030
wheel_radius_odom = 0.031
# Define a small noise standard deviation in radians.
noise_std_dev = 0.001  # radians  
wheel_base = 0.20
total_time = 12.57  # Total simulation time in seconds
dt = 0.01  # Time step in seconds

#Apply random seed for reproducibility
SEED = 22  # Set to None for random seed, or set to an integer for reproducibility
rng = np.random.default_rng(SEED)

def calculate_velocity(v_left, v_right, wheel_base):
    v = (v_left + v_right) / 2
    omega = (v_right - v_left) / wheel_base
    return v, omega

def update_pose(x, y, theta, v, omega, dt):
    #calculate the new x position
    new_x = x + v * np.cos(theta) * dt

    #calculate the new y position
    new_y = y + v * np.sin(theta) * dt

    #calculate the new heading
    new_theta = theta + omega * dt

    #return the updated state
    return new_x, new_y, new_theta

def update_pose_exact(x, y, theta, v, omega, dt):

    # Check whether the robot is turning.
    # If omega is not approximately zero,
    if np.abs(omega) > 1e-9:
    # use the circular-motion equations.
        # Calculate the new x position.
        new_x = x + (v / omega) * (np.sin(theta + omega * dt) - np.sin(theta))
        # Calculate the new y position.
        new_y = y - (v / omega) * (np.cos(theta + omega * dt) - np.cos(theta))
        
    else:
    # Otherwise, the robot is traveling straight.

        # Calculate the new x position.
        new_x = x + v * np.cos(theta) * dt
        # Calculate the new y position.
        new_y = y + v * np.sin(theta) * dt
    
    # Calculate the new heading.
    new_theta = theta + omega * dt
    # Return new_x, new_y, new_theta.
    return new_x, new_y, new_theta

def simulate_robot(v_left, v_right, wheel_base, dt, total_time):
    #Initialize the robot's position
    x, y, theta = 0.0, 0.0, 0.0
    #Calculate the number of simulation steps
    time_steps = int(round(total_time / dt))

    #Create lists to store x and y initial positions
    x_history = [0.0]
    y_history = [0.0]
    # Later we should also store theta history
    theta_history = [0.0]

    #Calculate the robot's linear and angular velocities
    v, omega = calculate_velocity(v_left, v_right, wheel_base)
    #Loop through each time step and update the robot's position
    for i in range(time_steps):
        x, y, theta = update_pose(x, y, theta, v, omega, dt)
        x_history.append(x)
        y_history.append(y)
        theta_history.append(theta)
    #Return the position history
    return x_history, y_history, theta_history

def calculate_odometry(delta_phi_left, delta_phi_right, wheel_radius, wheel_base):
    # Step 1: Calculate left wheel displacement

    delta_s_left = delta_phi_left * wheel_radius  # \left wheel displacement
    # Step 2: Calculate right wheel displacement
    delta_s_right = delta_phi_right * wheel_radius  # right wheel displacement
    # Step 3: Calculate robot center displacement
    delta_s = (delta_s_left + delta_s_right) / 2  # robot center displacement
    # Step 4: Calculate robot heading change
    delta_theta = (delta_s_right - delta_s_left) / wheel_base  # robot heading change
    # Step 5: Return both values
    return delta_s, delta_theta

def update_odometry_pose(x, y, theta, delta_s, delta_theta):

    # Step 1: Calculate the midpoint heading
    mid_theta = theta + delta_theta / 2
    # Step 2: Calculate the new x position
    new_x = x + delta_s * np.cos(mid_theta)
    # Step 3: Calculate the new y position
    new_y = y + delta_s * np.sin(mid_theta)
    # Step 4: Calculate the new heading
    new_theta = theta + delta_theta

    # Step 5: Return all three updated values
    return new_x, new_y, new_theta

def simulate_odometry(v_left, v_right, wheel_radius, wheel_base, dt, total_time, wheel_radius_odom, noise_std_dev):
    # Initialize the ground-truth pose
    x_true, y_true, theta_true = 0.0, 0.0, 0.0
    # Initialize the odometry-estimated pose
    x_odom, y_odom, theta_odom = 0.0, 0.0, 0.0
    # Initialize x and y ground truth histories
    x_true_history = [0.0]
    y_true_history = [0.0]
    theta_true_history = [0.0]

    # Initialize x and y odometry histories
    x_odom_history = [0.0]
    y_odom_history = [0.0]
    theta_odom_history = [0.0]

    # Calculate the number of time steps
    time_steps = int(round(total_time / dt))

    # Calculate linear and angular velocities
    v, omega = calculate_velocity(v_left, v_right, wheel_base)
  
    # Run the simulation loop
    for i in range(time_steps):
        # Update ground truth using wheel velocities
        x_true, y_true, theta_true = update_pose_exact(x_true, y_true, theta_true, v, omega, dt)
        x_true_history.append(x_true)
        y_true_history.append(y_true)
        theta_true_history.append(theta_true)

        # Calculate wheel displacements
        delta_phi_left = v_left * dt / wheel_radius
        delta_phi_right = v_right * dt / wheel_radius


        # Generate a random noise value for the left wheel.
        noise_left = rng.normal(0, noise_std_dev)
        # Generate a random noise value for the right wheel.    
        noise_right = rng.normal(0, noise_std_dev)
        # Add the left noise value to delta_phi_left.
        delta_phi_left_measured = delta_phi_left + noise_left
        # Add the right noise value to delta_phi_right.
        delta_phi_right_measured = delta_phi_right + noise_right

        # MEASURED angular displacements simulate simplified measurement noise.
        # into calculate_odometry().
        delta_s, delta_theta = calculate_odometry(delta_phi_left_measured, delta_phi_right_measured, wheel_radius_odom, wheel_base)
        # Calculate odometry ideal values without noise for comparison
        # delta_s, delta_theta = calculate_odometry(delta_phi_left, delta_phi_right, wheel_radius, wheel_base)
        # Update odometry pose
        x_odom, y_odom, theta_odom = update_odometry_pose(x_odom, y_odom, theta_odom, delta_s, delta_theta)
        x_odom_history.append(x_odom)
        y_odom_history.append(y_odom)
        theta_odom_history.append(theta_odom)
    return x_true_history, y_true_history, theta_true_history, x_odom_history, y_odom_history, theta_odom_history

def calculate_position_error(x_true_history, y_true_history, x_odom_history, y_odom_history):
    # Convert the four lists into numpy arrays for easier calculations
    x_true_array = np.array(x_true_history)
    y_true_array = np.array(y_true_history)
    x_odom_array = np.array(x_odom_history)
    y_odom_array = np.array(y_odom_history)

    # Calculate the x-position error at each time step
    x_pose_error = x_true_array - x_odom_array
    # Calculate the y-position error at each time step
    y_pose_error = y_true_array - y_odom_array
    # Use the Euclidean distance equation to calculate the total position error at every time step.  
    # The Euclidean distance is calculated as the square root of the sum of the squares of the x and y position errors.
    total_pose_error = np.sqrt(x_pose_error**2 + y_pose_error**2)
    return total_pose_error

def calculate_heading_error(theta_true_history, theta_odom_history):

    # Convert both heading-history lists into NumPy arrays.
    theta_true_array = np.array(theta_true_history)
    theta_odom_array = np.array(theta_odom_history)
    
    # Calculate the difference between the true
    # and estimated heading at every time step.
    heading_error = theta_true_array - theta_odom_array
    # Normalize the heading differences to the
    # interval [-pi, pi] using:
    # np.arctan2(np.sin(error), np.cos(error))
    normalized_heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
    # Return the normalized heading-error array.
    return normalized_heading_error

        #####Functions for controll action #######
def calculate_heading_control(theta_target, theta_odom, Kp):
    # Calculate the signed difference between the target
    # heading and the estimated heading.
    theta_error = theta_target - theta_odom
    
    # Normalize the heading difference to [-pi, pi].
    # Hint: Use np.arctan2(), np.sin(), and np.cos().
    normalized_error = np.arctan2(np.sin(theta_error), np.cos(theta_error))
   
    # Calculate the commanded angular velocity:
    # omega_command = Kp * normalized_error
    omega_command = Kp * normalized_error
   
    # Return omega_command.
    return omega_command

def calculate_wheel_velocities(v_command, omega_command, wheel_base):
    # STEP 1:
    # Calculate the left wheel velocity from
    # v_command, omega_command, and wheel_base.
    v_left = v_command - (omega_command * wheel_base / 2)
    # STEP 2:
    # Calculate the right wheel velocity.
    v_right = v_command + (omega_command * wheel_base / 2)
    # STEP 3:
    # Return both wheel velocities.
    return v_left, v_right

x_true_history, y_true_history, theta_true_history, x_odom_history, y_odom_history, theta_odom_history = simulate_odometry(v_left, v_right, wheel_radius, wheel_base, dt, total_time, wheel_radius_odom, noise_std_dev)

position_error = calculate_position_error(x_true_history, y_true_history, x_odom_history, y_odom_history)
normalized_heading_error = calculate_heading_error(theta_true_history, theta_odom_history)

max_error = np.max(position_error)
final_error = position_error[-1]
max_absolute_heading_error = np.max(np.abs(normalized_heading_error))
final_absolute_heading_error = np.abs(normalized_heading_error[-1])

max_error_millimeters = max_error * 1000
final_error_millimeters = final_error * 1000
# print("Maximum Position Error:", max_error_millimeters, "mm")
# print("Final Position Error:", final_error_millimeters, "mm")
# print("Maximum Absolute Heading Error:", np.rad2deg(max_absolute_heading_error), "degrees")
# print("Final Absolute Heading Error:", np.rad2deg(final_absolute_heading_error), "degrees")
time_history = np.arange(len(position_error)) * dt
position_error_millimeters = position_error * 1000

###########Control Action Simulation ##########
# omega_command = calculate_heading_control(np.deg2rad(179), np.deg2rad(-179), 2.0)
# print("Commanded angular velocity:", omega_command, "rad/s")
v_left_command, v_right_command = calculate_wheel_velocities(0, omega_command = 1.0, wheel_base=0.2)
print("Commanded left wheel velocity:", v_left_command, "m/s")
print("Commanded right wheel velocity:", v_right_command, "m/s")
# fig = plt.figure(figsize=(10, 6))
# plt.plot(time_history, position_error_millimeters, label='Position Error', color='red', linestyle='-.')
# plt.title("Robot Position Error Over Time")
# plt.xlabel("Time (s)")
# plt.ylabel("Position Error (mm)")
# plt.legend()
# plt.grid(True)
# plt.show()

# fig = plt.figure(figsize=(10, 6))
# plt.plot(x_true_history, y_true_history, label='Ground Truth', color='blue')
# plt.plot(x_odom_history, y_odom_history, label='Odometry', color='orange', linestyle='--')
# plt.title("Robot Trajectory Ground Truth vs Odometry")
# plt.xlabel("X Position (m)")
# plt.ylabel("Y Position (m)")
# plt.legend()
# plt.grid(True)
# plt.axis('equal')
# plt.show()
# delta_s, delta_theta = calculate_odometry(1.0, 1.5, 0.03, 0.2)
# x, y, theta = update_odometry_pose(0.0, 0.0, 0.0, delta_s, delta_theta)
# print("X:", x)
# print("Y:", y)
# print("Heading:", theta)
# print("Robot displacement :", delta_s, "m")
# print("Heading change:", delta_theta, "rad")

# v, omega = calculate_velocity(0.15, 0.25, 0.20)
# x, y, theta = 0.0, 0.0, 0.0

# dt = 0.1

# x, y, theta = update_pose(x, y, theta, v, omega, dt)
# print("Step 1:", x, y, theta)

# x, y, theta = update_pose(x, y, theta, v, omega, dt)
# print("Step 2:", x, y, theta)
# print("Linear velocity:", v, "m/s")
# print("Angular velocity:", omega, "rad/s") 

# x_history, y_history, theta_history = simulate_robot(0.15, 0.25, 0.20, 0.01, 12.57)
# time_history = np.linspace(0, total_time, int(round(total_time / dt)) + 1)

# plt.plot(x_history, y_history)
# plt.title("Robot Trajectory")
# plt.xlabel("X Position (m)")
# plt.ylabel("Y Position (m)")
# plt.axis('equal')
# plt.grid(True)
# plt.show()

# plt.plot(time_history, theta_history)
# plt.title("Robot Heading Over Time")
# plt.xlabel("Time (s)")
# plt.ylabel("Heading (rad)")
# plt.grid(True)
# plt.show()
