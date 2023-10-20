from robot import Robot
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

# Configuration Variables
xMin = -2
xMax = 5
yMin = -6
yMax = 6
gridSize = 0.5
filenameLandmarks = "ds0_Landmark_Groundtruth.dat"

robot = Robot(gridSize, xMin, xMax, yMin, yMax)

robot.import_landmark_GT(filenameLandmarks)

# Create plot
fig, ax = plt.subplots(figsize=(8, 8))
plt.xlim(xMin, xMax)
plt.ylim(yMin, yMax)
plt.grid(which='minor')
plt.grid(which='major')
robot.draw_map(ax)
robot.plot_landmark_pos(ax)
ax.set_aspect(1)
ax.set(xlabel='X Position (m)', ylabel='Y Position (m)', title='Robot Dead Reckoning Position, Test Data (Question 2)')
ax.xaxis.set_major_locator(MultipleLocator(1))
ax.yaxis.set_major_locator(MultipleLocator(1))
ax.xaxis.set_minor_locator(MultipleLocator(gridSize))
ax.yaxis.set_minor_locator(MultipleLocator(gridSize))
ax.legend()
plt.show()