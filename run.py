from robot import Robot
import matplotlib.pyplot as plt
import numpy as np

# Configuration Variables
xMin = -2
xMax = 5
yMin = -6
yMax = 6
gridSizeA = 1
gridSizeB = 0.1
filenameLandmarks = "ds1_Landmark_Groundtruth.dat"
figsize=(5,8)        #(3.2, 4)

# Create robot objects with large and small grid sizes
robot_large_grid = Robot(gridSizeA, xMin, xMax, yMin, yMax)
robot_small_grid = Robot(gridSizeB, xMin, xMax, yMin, yMax)

# Import landmark locations to both robot objects
robot_large_grid.import_landmark_GT(filenameLandmarks)
robot_small_grid.import_landmark_GT(filenameLandmarks)



# QUESTION 3

# CASE A
# Create plot
fig, ax1 = plt.subplots(figsize=figsize)
# Run A* navigation to generate path
robot_large_grid.a_star_navigation_interface([0.5, -1.5], [0.5, 1.5], ax=ax1, title='Robot Path, Large Grid, A*, Case A (Question 3)', online=False, simulate=False, execute=False)

# CASE B
# Create plot
fig, ax2 = plt.subplots(figsize=figsize)
# Run A* navigation to generate path
robot_large_grid.a_star_navigation_interface([4.5, 3.5], [4.5, -1.5], ax=ax2, title='Robot Path, Large Grid, A*, Case B (Question 3)', online=False, simulate=False, execute=False)

# CASE C
# Create plot
fig, ax3 = plt.subplots(figsize=figsize)
# Run A* navigation to generate path
robot_large_grid.a_star_navigation_interface([-0.5, 5.5], [1.5, -3.5], ax=ax3, title='Robot Path, Large Grid, A*, Case C (Question 3)', online=False, simulate=False, execute=False)



# QUESTION 5

# CASE A
# Create plot
fig, ax4 = plt.subplots(figsize=figsize)
# Run A* online navigation to generate path
robot_large_grid.a_star_navigation_interface([0.5, -1.5], [0.5, 1.5], ax=ax4, title='Robot Path, Large Grid, A* Online, Case A (Question 5)', online=True, simulate=False, execute=False)

# CASE B
# Create plot
fig, ax5 = plt.subplots(figsize=figsize)
# Run A* online navigation to generate path
robot_large_grid.a_star_navigation_interface([4.5, 3.5], [4.5, -1.5], ax=ax5, title='Robot Path, Large Grid, A* Online, Case B (Question 5)', online=True, simulate=False, execute=False)

# CASE C
# Create plot
fig, ax6 = plt.subplots(figsize=figsize)
# Run A* online navigation to generate path
robot_large_grid.a_star_navigation_interface([-0.5, 5.5], [1.5, -3.5], ax=ax6, title='Robot Path, Large Grid, A* Online, Case C (Question 5)', online=True, simulate=False, execute=False)



# QUESTION 7

# CASE A
# Create plot
fig, ax7 = plt.subplots(figsize=figsize)
# Run A* online navigation to generate path
robot_small_grid.a_star_navigation_interface([2.45, -3.55], [0.95, -1.55], ax=ax7, title='Robot Path, Small Grid, A* Online, Case A (Question 7)', online=True, simulate=False, execute=False)

# CASE B
# Create plot
fig, ax8 = plt.subplots(figsize=figsize)
# Run A* online navigation to generate path
robot_small_grid.a_star_navigation_interface([2.45, -3.55], [0.95, -1.55], ax=ax8, title='Robot Path, Small Grid, A* Online, Case B (Question 7)', online=True, simulate=False, execute=False)

# CASE C
# Create plot
fig, ax9 = plt.subplots(figsize=figsize)
# Run A* online navigation to generate path
robot_small_grid.a_star_navigation_interface([-0.55, 1.45], [1.95, 3.95], ax=ax9, title='Robot Path, Small Grid, A* Online, Case C (Question 7)', online=True, simulate=False, execute=False)



# QUESTION 9

# CASE A, NO NOISE
# Create plot
fig, ax10 = plt.subplots(figsize=figsize)
# Run A* online navigation to generate path and simulate robot motion
robot_small_grid.a_star_navigation_interface([2.45, -3.55], [0.95, -1.55], ax=ax10, title='Simulated Robot Motion, No Noise, Case A (Question 9)', online=True, simulate=True, execute=False)

# CASE B, NO NOISE
# Create plot
fig, ax11 = plt.subplots(figsize=figsize)
# Run A* online navigation to generate path and simulate robot motion
robot_small_grid.a_star_navigation_interface([4.95, -0.05], [2.45, 0.25], ax=ax11, title='Simulated Robot Motion, No Noise, Case B (Question 9)', online=True, simulate=True, execute=False)

# CASE C, NO NOISE
# Create plot
fig, ax12 = plt.subplots(figsize=figsize)
# Run A* online navigation to generate path and simulate robot motion
robot_small_grid.a_star_navigation_interface([-0.55, 1.45], [1.95, 3.95], ax=ax12, title='Simulated Robot Motion, No Noise, Case C (Question 9)', online=True, simulate=True, execute=False)

# CASE A, WITH NOISE
# Create plot
fig, ax13 = plt.subplots(figsize=figsize)
# Run A* online navigation to generate path and simulate robot motion
robot_small_grid.a_star_navigation_interface([2.45, -3.55], [0.95, -1.55], ax=ax13, title='Simulated Robot Motion, Noise, Case A (Question 9)', online=True, simulate=True, execute=False, omegaNoise=np.pi/4)

# CASE B, WITH NOISE
# Create plot
fig, ax14 = plt.subplots(figsize=figsize)
# Run A* online navigation to generate path and simulate robot motion
robot_small_grid.a_star_navigation_interface([4.95, -0.05], [2.45, 0.25], ax=ax14, title='Simulated Robot Motion, Noise, Case B (Question 9)', online=True, simulate=True, execute=False, omegaNoise=np.pi/4)

# CASE C, WITH NOISE
# Create plot
fig, ax15 = plt.subplots(figsize=figsize)
# Run A* online navigation to generate path and simulate robot motion
robot_small_grid.a_star_navigation_interface([-0.55, 1.45], [1.95, 3.95], ax=ax15, title='Simulated Robot Motion, Noise, Case C (Question 9)', online=True, simulate=True, execute=False, omegaNoise=np.pi/4)



# QUESTION 10

# CASE A, SMALL GRID
# Create plot
fig, ax16 = plt.subplots(figsize=figsize)
# Move the robot using A* online planning
robot_small_grid.a_star_navigation_interface([2.45, -3.55], [0.95, -1.55], ax=ax16, title='Robot Motion, Small Grid, Case A (Question 10)', online=True, simulate=False, execute=True, omegaNoise=np.pi/4)

# CASE B, SMALL GRID
# Create plot
fig, ax17 = plt.subplots(figsize=figsize)
# Move the robot using A* online planning
robot_small_grid.a_star_navigation_interface([4.95, -0.05], [2.45, 0.25], ax=ax17, title='Robot Motion, Small Grid, Case B (Question 10)', online=True, simulate=False, execute=True, omegaNoise=np.pi/4)

# CASE C, SMALL GRID
# Create plot
fig, ax18 = plt.subplots(figsize=figsize)
# Move the robot using A* online planning
robot_small_grid.a_star_navigation_interface([-0.55, 1.45], [1.95, 3.95], ax=ax18, title='Robot Motion, Small Grid, Case C (Question 10)', online=True, simulate=False, execute=True, omegaNoise=np.pi/4)



# QUESTION 11

# CASE A, LARGE GRID
# Create plot
fig, ax19 = plt.subplots(figsize=figsize)
# Move the robot using A* online planning
robot_large_grid.a_star_navigation_interface([2.45, -3.55], [0.95, -1.55], ax=ax19, title='Robot Motion, Large Grid, Case A (Question 11)', online=True, simulate=False, execute=True, omegaNoise=np.pi/4)

# CASE B, LARGE GRID
# Create plot
fig, ax20 = plt.subplots(figsize=figsize)
# Move the robot using A* online planning
robot_large_grid.a_star_navigation_interface([4.95, -0.05], [2.45, 0.25], ax=ax20, title='Robot Motion, Large Grid, Case B (Question 11)', online=True, simulate=False, execute=True, omegaNoise=np.pi/4)

# CASE C, LARGE GRID
# Create plot
fig, ax21 = plt.subplots(figsize=figsize)
# Move the robot using A* online planning
robot_large_grid.a_star_navigation_interface([-0.55, 1.45], [1.95, 3.95], ax=ax21, title='Robot Motion, Large Grid, Case C (Question 11)', online=True, simulate=False, execute=True, omegaNoise=np.pi/4)


# Display plots
plt.show()