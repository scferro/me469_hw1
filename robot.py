import numpy as np
import matplotlib.patches as patches

class Robot:
    def __init__(self, gridSize=1, xMin=-2, xMax=5, yMin=-6, yMax=6):
        self.gridSize = gridSize
        self.xRange = [xMin, xMax]
        self.yRange = [yMin, yMax]
        self.xLen = int((xMax - xMin) / gridSize - 1)
        self.yLen = int((yMax - yMin) / gridSize - 1)
        self.landmarkGT = {}
        self.start = [0,0]
        self.goal = [0,0]
        self.openSet = []
        self.occupiedCells = []
        self.f_list = []
        self.g_list = []
        self.stepCost = 1
        self.obsCost = 1000
        self.pathX = []
        self.pathY = []


    def a_star(self, start, goal):
        self.start = start
        self.goal = goal
        self.openSet.append(start)
        self.f_list.append(0)
        self.g_list.append(0)
        atGoal = False
        indexPoint = 0
        while atGoal == False:
            indexPoint = self.f_list.index(min(self.f_list))
            point = self.openSet[indexPoint]
            print(min(self.f_list))
            print(point)
            neighborsX = [point[0] - self.gridSize, point[0], point[0] + self.gridSize]
            neighborsY = [point[1] - self.gridSize, point[1], point[1] + self.gridSize]
            for xPos in neighborsX:
                for yPos in neighborsY:
                    stepCost = 1
                    if self.goal == [xPos, yPos]:
                        atGoal = True
                        self.openSet.append([xPos,yPos])
                        self.f_list.append(0)
                        self.g_list.append(g)
                    elif (xPos == point[0]) and (yPos == point[1]):
                        pass
                    else:
                        try:
                            index = self.occupiedCells.index([xPos, yPos])
                            stepCost = 1000
                        except ValueError as e:
                            pass
                        if (xPos < self.xRange[0]) or (xPos > self.xRange[1]) or (yPos < self.yRange[0]) or (yPos > self.yRange[1]):
                            stepCost = 1000
                        h = ((goal[0] - xPos)**2 + (goal[1] - yPos)**2)**0.5   # Hueristic Equation, h = sqrt(x**2 + y**2)
                        g = self.g_list[indexPoint] + stepCost
                        f = g + h
                        try:
                            indexF = self.openSet.index([xPos,yPos])
                            self.openSet.pop(indexF)
                            self.f_list.pop(indexF)
                            self.g_list.pop(indexF)
                        except ValueError as e:
                            pass
                        self.openSet.append([xPos,yPos])
                        self.f_list.append(f)
                        self.g_list.append(g)
                        print([xPos, yPos, f, g, h])
            print('cycle')
            self.openSet.pop(indexPoint)
            self.f_list.pop(indexPoint)
            self.g_list.pop(indexPoint)
            self.pathX.append(point[0])
            self.pathY.append(point[1])
        self.pathX.append(self.goal[0])
        self.pathY.append(self.goal[1])
      

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
            xPos = (np.floor(self.landmarkGT[landmark][0] / self.gridSize) + 0.5) * self.gridSize
            yPos = (np.floor(self.landmarkGT[landmark][1] / self.gridSize) + 0.5) * self.gridSize
            self.occupiedCells.append([xPos,yPos])
        print(self.occupiedCells)


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
            ax.add_patch(patches.Rectangle([xPos, yPos], self.gridSize, self.gridSize, fill=True, color='k'))


    def plot_start_goal(self, ax):
        # Plots the locations on the start and goal cell
        startX = self.start[0] - (0.5 * self.gridSize)
        startY = self.start[1] - (0.5 * self.gridSize)
        goalX = self.goal[0] - (0.5 * self.gridSize)
        goalY = self.goal[1] - (0.5 * self.gridSize)
        ax.add_patch(patches.Rectangle([startX, startY], self.gridSize, self.gridSize, fill=True, color='g'))
        ax.add_patch(patches.Rectangle([goalX, goalY], self.gridSize, self.gridSize, fill=True, color='b'))
        ax.plot(self.pathX, self.pathY, 'r-', label='Robot Path')


    def reset_robot_path_data(self):
        self.start = [0,0]
        self.goal = [0,0]
        self.openSet = []
        self.f_list = []
        self.g_list = []
        self.pathX = []
        self.pathY = []