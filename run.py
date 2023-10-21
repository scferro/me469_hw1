from robot import Robot
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

# Configuration Variables
xMin = -2
xMax = 5
yMin = -6
yMax = 6
gridSizeA = 1
gridSizeB = 0.1
filenameLandmarks = "ds0_Landmark_Groundtruth.dat"

robot_large_grid = Robot(gridSizeA, xMin, xMax, yMin, yMax)
robot_large_grid.import_landmark_GT(filenameLandmarks)
robot_large_grid.mark_occupied_cells()

# QUESTION 3

# CASE A
robot_large_grid.a_star([0.5, -1.5], [0.5, 1.5])
# Create plot
fig, ax = plt.subplots(figsize=(8, 8))
plt.xlim(xMin, xMax)
plt.ylim(yMin, yMax)
plt.grid(which='minor')
plt.grid(which='major')
robot_large_grid.plot_landmark_pos(ax)
robot_large_grid.plot_start_goal(ax)
ax.set_aspect(1)
ax.set(xlabel='X Position (m)', ylabel='Y Position (m)', title='Robot Path, Case A (Question 3)')
ax.xaxis.set_major_locator(MultipleLocator(1))
ax.yaxis.set_major_locator(MultipleLocator(1))
ax.xaxis.set_minor_locator(MultipleLocator(gridSizeA))
ax.yaxis.set_minor_locator(MultipleLocator(gridSizeA))
ax.legend()

# CASE B
robot_large_grid.reset_robot_path_data()
robot_large_grid.a_star([4.5, 3.5], [4.5, -1.5])
# Create plot
fig, bx = plt.subplots(figsize=(8, 8))
plt.xlim(xMin, xMax)
plt.ylim(yMin, yMax)
plt.grid(which='minor')
plt.grid(which='major')
robot_large_grid.plot_landmark_pos(bx)
robot_large_grid.plot_start_goal(bx)
bx.set_aspect(1)
bx.set(xlabel='X Position (m)', ylabel='Y Position (m)', title='Robot Path, Case B (Question 3)')
bx.xaxis.set_major_locator(MultipleLocator(1))
bx.yaxis.set_major_locator(MultipleLocator(1))
bx.xaxis.set_minor_locator(MultipleLocator(gridSizeA))
bx.yaxis.set_minor_locator(MultipleLocator(gridSizeA))
bx.legend()

# CASE C
robot_large_grid.reset_robot_path_data()
robot_large_grid.a_star([-0.5, 5.5], [1.5, -3.5])
# Create plot
fig, cx = plt.subplots(figsize=(8, 8))
plt.xlim(xMin, xMax)
plt.ylim(yMin, yMax)
plt.grid(which='minor')
plt.grid(which='major')
robot_large_grid.plot_landmark_pos(cx)
robot_large_grid.plot_start_goal(cx)
cx.set_aspect(1)
cx.set(xlabel='X Position (m)', ylabel='Y Position (m)', title='Robot Path, Case C (Question 3)')
cx.xaxis.set_major_locator(MultipleLocator(1))
cx.yaxis.set_major_locator(MultipleLocator(1))
cx.xaxis.set_minor_locator(MultipleLocator(gridSizeA))
cx.yaxis.set_minor_locator(MultipleLocator(gridSizeA))
cx.legend()


# QUESTION 5

# CASE A
robot_large_grid.reset_robot_path_data()
robot_large_grid.a_star([0.5, -1.5], [0.5, 1.5])
# Create plot
fig, dx = plt.subplots(figsize=(8, 8))
plt.xlim(xMin, xMax)
plt.ylim(yMin, yMax)
plt.grid(which='minor')
plt.grid(which='major')
robot_large_grid.plot_landmark_pos(dx)
robot_large_grid.plot_start_goal(dx)
dx.set_aspect(1)
dx.set(xlabel='X Position (m)', ylabel='Y Position (m)', title='Robot Path, Case A (Question 5)')
dx.xaxis.set_major_locator(MultipleLocator(1))
dx.yaxis.set_major_locator(MultipleLocator(1))
dx.xaxis.set_minor_locator(MultipleLocator(gridSizeA))
dx.yaxis.set_minor_locator(MultipleLocator(gridSizeA))
dx.legend()

# CASE B
robot_large_grid.reset_robot_path_data()
robot_large_grid.a_star([4.5, 3.5], [4.5, -1.5])
# Create plot
fig, ex = plt.subplots(figsize=(8, 8))
plt.xlim(xMin, xMax)
plt.ylim(yMin, yMax)
plt.grid(which='minor')
plt.grid(which='major')
robot_large_grid.plot_landmark_pos(ex)
robot_large_grid.plot_start_goal(ex)
ex.set_aspect(1)
ex.set(xlabel='X Position (m)', ylabel='Y Position (m)', title='Robot Path, Case B (Question 5)')
ex.xaxis.set_major_locator(MultipleLocator(1))
ex.yaxis.set_major_locator(MultipleLocator(1))
ex.xaxis.set_minor_locator(MultipleLocator(gridSizeA))
ex.yaxis.set_minor_locator(MultipleLocator(gridSizeA))
ex.legend()

# CASE C
robot_large_grid.reset_robot_path_data()
robot_large_grid.a_star([-0.5, 5.5], [1.5, -3.5])
# Create plot
fig, fx = plt.subplots(figsize=(8, 8))
plt.xlim(xMin, xMax)
plt.ylim(yMin, yMax)
plt.grid(which='minor')
plt.grid(which='major')
robot_large_grid.plot_landmark_pos(fx)
robot_large_grid.plot_start_goal(fx)
fx.set_aspect(1)
fx.set(xlabel='X Position (m)', ylabel='Y Position (m)', title='Robot Path, Case C (Question 5)')
fx.xaxis.set_major_locator(MultipleLocator(1))
fx.yaxis.set_major_locator(MultipleLocator(1))
fx.xaxis.set_minor_locator(MultipleLocator(gridSizeA))
fx.yaxis.set_minor_locator(MultipleLocator(gridSizeA))
fx.legend()

plt.show()


# QUESTION 7

robot_small_grid = Robot(gridSizeB, xMin, xMax, yMin, yMax)
robot_small_grid.import_landmark_GT(filenameLandmarks)
robot_small_grid.mark_occupied_cells()

# CASE A
robot_small_grid.a_star([2.45, -3.55], [0.95, -1.55])
# Create plot
fig, dx = plt.subplots(figsize=(8, 8))
plt.xlim(xMin, xMax)
plt.ylim(yMin, yMax)
plt.grid(which='minor')
plt.grid(which='major')
robot_small_grid.plot_landmark_pos(dx)
robot_small_grid.plot_start_goal(dx)
dx.set_aspect(1)
dx.set(xlabel='X Position (m)', ylabel='Y Position (m)', title='Robot Path, Case A (Question 5)')
dx.xaxis.set_major_locator(MultipleLocator(1))
dx.yaxis.set_major_locator(MultipleLocator(1))
dx.xaxis.set_minor_locator(MultipleLocator(gridSizeB))
dx.yaxis.set_minor_locator(MultipleLocator(gridSizeB))
dx.legend()

# CASE B
robot_small_grid.reset_robot_path_data()
robot_small_grid.a_star([4.5, 3.5], [4.5, -1.5])
# Create plot
fig, ex = plt.subplots(figsize=(8, 8))
plt.xlim(xMin, xMax)
plt.ylim(yMin, yMax)
plt.grid(which='minor')
plt.grid(which='major')
robot_small_grid.plot_landmark_pos(ex)
robot_small_grid.plot_start_goal(ex)
ex.set_aspect(1)
ex.set(xlabel='X Position (m)', ylabel='Y Position (m)', title='Robot Path, Case B (Question 5)')
ex.xaxis.set_major_locator(MultipleLocator(1))
ex.yaxis.set_major_locator(MultipleLocator(1))
ex.xaxis.set_minor_locator(MultipleLocator(gridSizeB))
ex.yaxis.set_minor_locator(MultipleLocator(gridSizeB))
ex.legend()

# CASE C
robot_small_grid.reset_robot_path_data()
robot_small_grid.a_star([-0.5, 5.5], [1.5, -3.5])
# Create plot
fig, fx = plt.subplots(figsize=(8, 8))
plt.xlim(xMin, xMax)
plt.ylim(yMin, yMax)
plt.grid(which='minor')
plt.grid(which='major')
robot_small_grid.plot_landmark_pos(fx)
robot_small_grid.plot_start_goal(fx)
fx.set_aspect(1)
fx.set(xlabel='X Position (m)', ylabel='Y Position (m)', title='Robot Path, Case C (Question 5)')
fx.xaxis.set_major_locator(MultipleLocator(1))
fx.yaxis.set_major_locator(MultipleLocator(1))
fx.xaxis.set_minor_locator(MultipleLocator(gridSizeB))
fx.yaxis.set_minor_locator(MultipleLocator(gridSizeB))
fx.legend()

plt.show()