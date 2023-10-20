import numpy as np
import matplotlib.patches as patches

class Robot:
    def __init__(self, gridSize=1, xMin=-2, xMax=5, yMin=-6, yMax=6):
        self.gridSize = gridSize
        self.xRange = [xMin, xMax]
        self.yRange = [yMin, yMax]
        self.xLen = int((xMax - xMin) / gridSize - 1)
        self.yLen = int((yMax - yMin) / gridSize - 1)
        self.map = np.full((self.xLen, self.yLen), False)
        self.landmarkGT = {}
        self.start = [0,0]
        self.goal = [0,0]
        self.openSet = {}
        self.explored = {}


    def a_star(self, start, goal):
        self.start = start
        self.goal = goal
        self.openSet = {start: 0}
        while openSet 


    def import_landmark_GT(self, filename): 
        # Imports data from specified file. File should be in same directory as the python files
        print('Importing landmark groundtruth data from file "' + filename + '"...')
        data = np.genfromtxt(filename, skip_header=4)
        countMax = len(data)
        count = 0
        newData = {}
        while count < countMax:
            newData[int(data[count][0])] = [data[count][1],data[count][2]]
            count += 1
        print("Landmark groundtruth data import complete!")
        self.landmarkGT = newData
        return newData

    def mark_occupied_cells(self):
        # Marks cells that are occupied by landmarks
        for landmark in self.landmarkGT:
            xIndex = np.floor(landmark[0] * self.gridSize)
            yIndex = np.floor(landmark[1] * self.gridSize)
            self.map[xIndex][yIndex] = True

    def draw_map(self, ax):
        # function to draw the map and show the robot's path
        xPosGT = []
        yPosGT = []
        for pos in self.path:
            try:
                xPosGT.append(pos[0])
                yPosGT.append(pos[1])
            except IndexError as e:
                xPosGT.append(self.start[0])
                yPosGT.append(self.start[1])
        ax.plot(xPosGT, yPosGT, 'k-', label='Robot Ground Truth Position')

    def plot_landmark_pos(self, ax):
        # Plots the locations on the landmarks
        for pos in self.landmarkGT:
            xPos = np.floor(float(self.landmarkGT[pos][0]) / self.gridSize) * self.gridSize
            yPos = np.floor(float(self.landmarkGT[pos][1]) / self.gridSize) * self.gridSize
            print([xPos,yPos])
            ax.add_patch(patches.Rectangle([xPos, yPos], self.gridSize, self.gridSize, fill=True, color='k'))