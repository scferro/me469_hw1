import numpy as np
import matplotlib.patches as patches
from cell import Cell
import time
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

class Robot:
    def __init__(self, gridSize=1, xMin=-2, xMax=5, yMin=-6, yMax=6):
        '''
        # Description:
        #   Initializes the robot object
        # Inputs:
        #   gridSize: the side length of the grid cells (smaller value => finer grid)
        #   xMin: the minimum X coordinate of the taskspace
        #   xMax: the maximum X coordinate of the taskspace
        #   yMin: the minimum Y coordinate of the taskspace
        #   yMax: the maximum Y coordinate of the taskspace
        '''
        self.gridSize = gridSize
        self.xRange = [xMin, xMax]
        self.yRange = [yMin, yMax]
        self.start = [0,0]
        self.goal = [0,0]
        self.stepCost = 1
        self.obsCost = 1000
        self.robotTargets = []
        self.landmarkGT = {}
        self.occupiedCells = []
        self.knownOccupiedCells = []
        self.odometry = []
        self.simulatedRobotStates = []
        self.robotStates = []


    ############### MAIN FUNCTION ###############

    def a_star_navigation_interface(self, start, goal, ax, title, online=True, simulate=False, execute=False, omegaNoise=0.0):
        '''
        Description:
            The main function to plan and execute paths. Generates plots of paths and robot motion
        Inputs:
            start: The starting position of the robot [x,y]
            goal: The goal position of the robot [x,y]
            ax: An axis to display graphs
            title: The title for the graph
            online: Selects online or offline planning (bool)
            simulate: Selects if robot motion should be simulated for the planned path (bool)
            execute: Selects if the robot should follow the path to the goal (bool)
        '''
        self.reset_robot_path_data()
        self.reset_robot_movement_data()
        if start[0]%self.gridSize == 0.0:
            start[0] += self.gridSize/2
        if start[1]%self.gridSize == 0.0:
            start[1] += self.gridSize/2
        if goal[0]%self.gridSize == 0.0:
            goal[0] += self.gridSize/2
        if goal[1]%self.gridSize == 0.0:
            goal[1] += self.gridSize/2
        self.start = start
        self.goal = goal
        self.plot_landmark_cells(ax)
        self.plot_start_goal(ax)
        if online == True:
            print('Generating path from', start, 'to', goal, 'using A* Online Planning...')
            self.a_star_online(start,goal)
        elif online == False:
            print('Generating path from', start, 'to', goal, 'using A* Planning...')
            self.a_star(start,goal)
        if simulate == False:
            print('Plotting generated path...')
            if execute == False:
                self.plot_robot_targets(ax)
        elif simulate==True:
            if omegaNoise == 0.0:
                print('Simulating robot movement without noise...')
            else:
                print('Simulating robot movement with noise...')
            self.simulate_robot_odom(omegaNoise)
            print('Plotting generated path and simulated robot movement...')
            self.plot_robot_targets(ax)
            self.plot_simulated_movement(ax)
            if self.gridSize < 0.5:
                self.plot_visited_cells(ax)
        if execute == True:
            self.reset_robot_movement_data()
            print('Moving robot...')
            self.move_robot_with_a_star(start, goal, online, omegaNoise)
            print('Plotting robot movement...')
            if self.gridSize < 0.5:
                self.plot_visited_cells(ax)
            self.plot_robot_movement(ax)
        self.grid_config(ax, self.gridSize, title)


    ############### NAVIGATION FUNCTIONS ###############

    def a_star(self, start, goal):
        '''
        Description:
            Runs offline A* search. All obstacle locations are known to the robot
        Inputs:
            start: The starting position of the robot [x,y]
            goal: The goal position of the robot [x,y]
        '''
        self.reset_robot_path_data()
        self.start = start
        self.goal = goal
        self.knownOccupiedCells = self.occupiedCells
        startIndex = self.pos_to_cell(start)
        goalIndex =  self.pos_to_cell(goal)
        targetList = self.find_path(startIndex, goalIndex)
        for point in targetList:
            self.robotTargets.insert(0, point)


    def a_star_online(self, start, goal):
        '''
        Description:
            Runs online A* search. All obstacle locations are not known to the robot until it is in an adjescent cell
        Inputs:
            start: The starting position of the robot [x,y]
            goal: The goal position of the robot [x,y]
        '''
        self.reset_robot_path_data()
        self.start = start
        self.goal = goal
        startIndex = self.pos_to_cell(start)
        goalIndex =  self.pos_to_cell(goal)
        startRemoved = []
        goalRemoved = []
        try:
            index = self.occupiedCells.index(startIndex)
            startRemoved = self.occupiedCells.pop(index)
        except ValueError as e:
            pass
        try:
            index = self.occupiedCells.index(goalIndex)
            goalRemoved = self.occupiedCells.pop(index)
        except ValueError as e:
            pass
        self.robotTargets = [startIndex]
        currentPos = startIndex
        path = []
        while currentPos != goalIndex:
            startIndex = currentPos
            path = self.find_path(startIndex, goalIndex)
            currentPos = path[-2]
            self.robotTargets.append(currentPos)
            path = []
        if startRemoved != []:
            self.occupiedCells.append(startRemoved)
        if goalRemoved != []:
            self.occupiedCells.append(goalRemoved)


    def find_path(self, startCell, goalCell):
        '''
        Description:
           A function to find a path between two cells and around known obstacles using A* search.
        Inputs:
            startCell: The starting cell of the robot [x_index, y_index]
            goalCell: The goal cell of the robot [x_index, y_index]
        '''
        currentCell = startCell
        atGoal = False
        g_cost = 0
        f_cost = ((goalCell[0] - currentCell[0])**2 + (goalCell[1] - currentCell[1])**2)**0.5 * self.gridSize * 0.5             # Hueristic Equation
        mapEvaluated = [[]]
        mapOpen = [[]]
        xLen = int((self.xRange[1] - self.xRange[0]) / self.gridSize)
        yLen = int((self.yRange[1] - self.yRange[0]) / self.gridSize)
        for x in range(xLen):
            mapEvaluated.append([])
            mapOpen.append([])
            for y in range(yLen):
                mapEvaluated[x].append(0)
                mapOpen[x].append(0)
        mapOpen[int(currentCell[0])][int(currentCell[1])] = Cell(int(currentCell[0]), int(currentCell[1]), f_cost, g_cost, False)
        neighborsStart = mapOpen[currentCell[0]][currentCell[1]].neighbors
        for neighbor in neighborsStart:
            try:
                self.occupiedCells.index(neighbor)
                self.knownOccupiedCells.append(neighbor)
            except ValueError as e:
                pass
        while atGoal == False:
            neighbors = mapOpen[currentCell[0]][currentCell[1]].neighbors
            for neighbor in neighbors:
                obsCost = self.stepCost
                try:
                    self.knownOccupiedCells.index(neighbor)
                    obsCost = self.obsCost
                except ValueError as e:
                    pass
                g_cost = mapOpen[currentCell[0]][currentCell[1]].g_cost + obsCost
                f_cost = ((goalCell[0] - neighbor[0])**2 + (goalCell[1] - neighbor[1])**2)**0.5 * self.gridSize + g_cost
                try:
                    if (mapOpen[neighbor[0]][neighbor[1]] == 0) and (mapEvaluated[neighbor[0]][neighbor[1]] == 0):
                        mapOpen[neighbor[0]][neighbor[1]] = Cell(neighbor[0], neighbor[1], f_cost, g_cost, currentCell)
                except IndexError as e:
                    pass
            min_f = 10^100
            cellMin_f = []
            mapEvaluated[currentCell[0]][currentCell[1]] = mapOpen[currentCell[0]][currentCell[1]]
            mapOpen[currentCell[0]][currentCell[1]] = 0
            for row in mapOpen:
                for cell in row:
                    if cell == 0:
                        pass
                    else:
                        if cell.f_cost < min_f:
                            min_f = cell.f_cost
                            cellMin_f = cell
            currentCell = [cellMin_f.pos[0], cellMin_f.pos[1]]
            if currentCell == goalCell:
                atGoal = True
                mapEvaluated[currentCell[0]][currentCell[1]] = mapOpen[currentCell[0]][currentCell[1]]
                mapOpen[currentCell[0]][currentCell[1]] = 0
        path = [goalCell]
        parentCell = mapEvaluated[goalCell[0]][goalCell[1]].parent
        while parentCell != startCell:
            path.append(parentCell)
            parentCell = mapEvaluated[parentCell[0]][parentCell[1]].parent
        path.append(startCell)
        return path


    def move_robot_with_a_star(self, start, goal, online=True, maxOmegaNoise=0.0):
        '''
        Description:
            Uses A* search (online or offline) to move the robot from the start to the goal position 
        Inputs:
            start: The starting position of the robot [x,y]
            goal: The goal position of the robot [x,y]
            online: Seelcts whether online or offline search should be used (bool)
            maxOmegaNoise: The maximum amount of rotation noise added to each step of the robot movement
        '''
        currentTheta = -np.pi/2
        currentAngVel = 0
        currentLinVel = 0
        currentX = start[0]
        currentY = start[1]
        goalCell = self.pos_to_cell(goal)
        startCell = self.pos_to_cell(start)
        currentState = [currentX, currentY, currentTheta, currentLinVel, currentAngVel]
        currentCell = startCell
        while (currentCell[0] != goalCell[0]) or (currentCell[1] != goalCell[1]):
            if online == True:
                self.a_star_online([currentState[0], currentState[1]], goal)
            else:
                self.a_star([currentState[0], currentState[1]], goal)
            target = self.cell_to_pos([self.robotTargets[1][0], self.robotTargets[1][1]])
            newState, newCell, stateList = self.generate_and_exectue_odometry(currentState, target, currentCell, maxOmegaNoise)
            for state in stateList:
                self.robotStates.append(state)
            currentCell = newCell
            currentState = newState


    def simulate_robot_odom(self, maxOmegaNoise=0.0):
        '''
        Description:
            Simulates robot movement along a planned path 
        Inputs:
            maxOmegaNoise: The maximum amount of rotation noise added to each step of the simulated robot movement
        '''
        length = len(self.robotTargets)
        counter = 1
        currentTheta = -np.pi/2
        currentAngVel = 0
        currentLinVel = 0
        currentX = (self.robotTargets[0][0] + 0.5) * self.gridSize + self.xRange[0]
        currentY = (self.robotTargets[0][1] + 0.5) * self.gridSize + self.yRange[0]
        currentState = [currentX, currentY, currentTheta, currentLinVel, currentAngVel]
        while counter < length:
            targetX = (self.robotTargets[counter][0] + 0.5) * self.gridSize + self.xRange[0]
            targetY = (self.robotTargets[counter][1] + 0.5) * self.gridSize + self.yRange[0]
            newState, newCell, stateList = self.generate_and_exectue_odometry(currentState, [targetX, targetY], self.robotTargets[counter-1], maxOmegaNoise)
            for state in stateList:
                self.simulatedRobotStates.append(state)
            if (newCell[0] != self.robotTargets[counter][0]) and (newCell[1] != self.robotTargets[counter][1]):
                pass
            else:
                counter += 1
            currentState = newState


    def generate_and_exectue_odometry(self, initialState, targetPos, initialCell, maxOmegaNoise=0.0):
        maxLinVel = 0.25
        dt = 0.1
        P_theta = 20.0
        I_theta = 0.5
        D_theta = -1.0
        timeElapsed = 0
        omegaCommand = 0
        velCommand = 0
        cumThetaError = 0
        movementThetaTol = 0.1
        stateList = [initialState]
        currentState = initialState
        newState = []
        newCell = False
        currentCellX = initialCell[0]
        currentCellY = initialCell[1]
        while newCell == False:
            thetaTarget = np.arctan2([targetPos[1] - currentState[1]], [targetPos[0] - currentState[0]])[0]
            thetaError = thetaTarget - currentState[2]
            while thetaError < -np.pi:
                thetaError += 2 * np.pi
            while thetaError > np.pi:
                thetaError += -2 * np.pi
            cumThetaError += thetaError * dt
            omegaCommand = (P_theta * thetaError) + (I_theta * cumThetaError) + (D_theta * currentState[4])
            if abs(thetaError) <= movementThetaTol:
                velCommand = maxLinVel 
            else:
                velCommand = 0
            newState = self.execute_odometry(velCommand, omegaCommand, dt, currentState, maxOmegaNoise)
            currentState = newState
            timeElapsed += dt
            newOdom = [velCommand, omegaCommand]
            self.odometry.append(newOdom)
            stateList.append(currentState)
            currentCellX = int(np.floor((currentState[0] - self.xRange[0]) / self.gridSize))
            currentCellY = int(np.floor((currentState[1] - self.yRange[0]) / self.gridSize))
            if (currentCellX != initialCell[0]) or (currentCellY != initialCell[1]):
                newCell = True
        return currentState, [currentCellX, currentCellY], stateList
        

    def execute_odometry(self, velCommand, omegaCommand, timeStep, state, maxOmegaNoise=0.0):
        # this fucntion calculates the new position and orientation of the robot based on the previous location and oriantationa and the command sent to the robot
        maxAngAcc = 5.579
        maxLinAcc = 0.288
        maxDeltaVel = maxLinAcc * timeStep
        maxDeltaOmega = maxAngAcc * timeStep
        x0 = state[0]
        y0 = state[1]
        theta = state[2]
        currentLinVel = state[3]
        currentAngVel = state[4]
        omegaNoise = np.random.normal(0,0.33) * maxOmegaNoise
        omegaNoise = np.clip(omegaNoise, -maxOmegaNoise, maxOmegaNoise)
        deltaVel = velCommand - currentLinVel
        deltaOmega = omegaCommand - currentAngVel
        if deltaVel > maxDeltaVel:
            deltaVel = maxDeltaVel
        elif deltaVel < -maxDeltaVel:
            deltaVel = -maxDeltaVel
        if deltaOmega > maxDeltaOmega:
            deltaOmega = maxDeltaOmega
        elif deltaOmega < -maxDeltaOmega:
            deltaOmega = -maxDeltaOmega
        omega = currentAngVel + deltaOmega + omegaNoise
        vel = currentLinVel + deltaVel
        if omega == 0.0:
            # This is used in cases where angular velocity = 0, calculates new position for a linear move based on current heading, linear velocity, and time step
            deltaX = np.cos(theta) * vel * timeStep
            deltaY = np.sin(theta) * vel * timeStep
            xNew = x0 + deltaX
            yNew = y0 + deltaY
            thetaNew = theta
            return np.array([xNew, yNew, thetaNew, vel, omega])
        else:
            # This is used in cases where angular velocity =/= 0 (turning), calculates new position for a linear move based on current heading, linear and angularvelocity, and time step
            radius = abs(vel/omega) # radius of rotation based on linear and angular speed
            angRot = omega * timeStep # angle change during maneuver based on angular speed
            angRotMod = angRot - (np.pi/2 - theta) # convert angle change to global coordinate system
            if omega > 0: 
                # Calculates the center of rotation for a LH turn
                curveCenterX = x0 + (np.cos(theta + np.pi/2) * radius)
                curveCenterY = y0 + (np.sin(theta + np.pi/2) * radius)
                thetaNew = theta + angRot
                # Calculate a new position based off the center of rotation, the radius of rotation, and the angle of rotation
                xNew = curveCenterX + (np.cos(angRotMod) * radius) 
                yNew = curveCenterY + (np.sin(angRotMod) * radius)
            elif omega < 0: 
                # Calculates the center of rotation for a RH turn
                curveCenterX = x0 + (np.cos(theta - np.pi/2) * radius)
                curveCenterY = y0 + (np.sin(theta - np.pi/2) * radius)
                thetaNew = theta + angRot
                # Calculate a new position based off the center of rotation, the radius of rotation, and the angle of rotation
                xNew = curveCenterX + (np.cos(angRotMod) * -radius) 
                yNew = curveCenterY + (np.sin(angRotMod) * -radius)
            return np.array([xNew, yNew, thetaNew, vel, omega])


    ############### MAP CONFIGURATION FUNCTIONS ###############

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
        self.mark_occupied_cells()
        return newData


    def mark_occupied_cells(self):
        # Marks cells that are occupied by landmarks
        for landmark in self.landmarkGT:
            xPos = int(np.floor((self.landmarkGT[landmark][0] - self.xRange[0]) / self.gridSize))
            yPos = int(np.floor((self.landmarkGT[landmark][1] - self.yRange[0]) / self.gridSize))
            self.occupiedCells.append([xPos,yPos])
        if self.gridSize < 0.3:
            self.expand_occupied_cells()


    def expand_occupied_cells(self):
        modifiers = [-1, 0, 1]
        obsExpanded = []
        for cell in self.occupiedCells:
            for xMod in modifiers:
                for yMod in modifiers:
                    obsCell = [cell[0] + xMod, cell[1] + yMod]
                    try:
                        self.occupiedCells.index(obsCell)
                    except ValueError as e:
                        obsExpanded.append(obsCell)
        for cell in obsExpanded:
            self.occupiedCells.append(cell)


    ############### PLOTTING AND GRAPHING FUNCTIONS ###############

    def plot_landmark_cells(self, ax):
        # Plots the locations on the landmarks
        for cell in self.occupiedCells:
            xPos = cell[0] * self.gridSize + self.xRange[0]
            yPos = cell[1] * self.gridSize + self.yRange[0]
            ax.add_patch(patches.Rectangle([xPos, yPos], self.gridSize, self.gridSize, fill=True, color='k'))


    def plot_visited_cells(self, ax):
        # Plots the locations on the landmarks
        visitedCells = []
        if self.robotStates ==[]:
            states = self.simulatedRobotStates
        else:
            states = self.robotStates
        goalCell = self.pos_to_cell(self.goal)
        startCell = self.pos_to_cell(self.start)
        for pos in states:
            cell = self.pos_to_cell([pos[0], pos[1]])
            try:
                index = visitedCells.index(cell)
            except ValueError as e:
                visitedCells.append(cell)
        visitedCells.pop(0)
        visitedCells.pop(-1)
        for cell in visitedCells:
            xPos = cell[0] * self.gridSize + self.xRange[0]
            yPos = cell[1] * self.gridSize + self.yRange[0]
            ax.add_patch(patches.Rectangle([xPos, yPos], self.gridSize, self.gridSize, fill=True, color='r'))
        ax.scatter([], [], label='Visited Cells', color='r')


    def plot_start_goal(self, ax):
        # Plots the locations on the start and goal cell
        startCell = self.pos_to_cell(self.start)
        goalCell = self.pos_to_cell(self.goal)
        [startX, startY] = self.cell_to_pos(startCell)
        [goalX, goalY] = self.cell_to_pos(goalCell)
        startX += -self.gridSize / 2
        startY += -self.gridSize / 2
        goalX += -self.gridSize / 2
        goalY += -self.gridSize / 2
        ax.add_patch(patches.Rectangle([startX, startY], self.gridSize, self.gridSize, fill=True, color='g'))
        ax.add_patch(patches.Rectangle([goalX, goalY], self.gridSize, self.gridSize, fill=True, color='b'))
        ax.scatter([], [], label='Start Position', color='g')
        ax.scatter([], [], label='Goal Position', color='b')


    def plot_robot_targets(self,ax):
        # function to plot the robot path
        robX = []
        robY = []
        for pos in self.robotTargets:
            xPos = ((pos[0] + 0.5) * self.gridSize) + self.xRange[0]
            yPos = ((pos[1] + 0.5) * self.gridSize) + self.yRange[0]
            robX.append(xPos)
            robY.append(yPos)
        ax.plot(robX, robY, 'r-', label='Robot Planned Path')


    def plot_robot_movement(self,ax):
        # function to plot the robot path
        statesX = []
        statesY = []
        dirsX = []
        dirsY = []
        for pos in self.robotStates:
            xPos = pos[0]
            yPos = pos[1]
            statesX.append(xPos)
            statesY.append(yPos)
            xDir = np.cos(pos[2])
            yDir = np.sin(pos[2])
            dirsX.append(xDir)
            dirsY.append(yDir)
        ax.plot(statesX, statesY, 'b-', label='Robot Movement')
        # plot arrow on each line:
        counter = 0
        stepsToSkip = 20
        for X,Y,dX,dY in zip(statesX, statesY, dirsX, dirsY):
            if counter == stepsToSkip:
                ax.annotate("", xytext=(X,Y),xy=(X+0.001*dX,Y+0.001*dY), 
                arrowprops=dict(arrowstyle="->", color='b'), size = 15)
                counter = 0
            else:
                counter += 1


    def plot_simulated_movement(self,ax):
        # function to plot the robot path
        statesX = []
        statesY = []
        dirsX = []
        dirsY = []
        for pos in self.simulatedRobotStates:
            xPos = pos[0]
            yPos = pos[1]
            statesX.append(xPos)
            statesY.append(yPos)
            xDir = np.cos(pos[2])
            yDir = np.sin(pos[2])
            dirsX.append(xDir)
            dirsY.append(yDir)
        ax.plot(statesX, statesY, 'g-', label='Simulated Robot Movement')
        # plot arrow on each line:
        counter = 0
        stepsToSkip = 20
        for X,Y,dX,dY in zip(statesX, statesY, dirsX, dirsY):
            if counter == stepsToSkip:
                ax.annotate("", xytext=(X,Y),xy=(X+0.001*dX,Y+0.001*dY), 
                arrowprops=dict(arrowstyle="->", color='g'), size = 15)
                counter = 0
            else:
                counter += 1


    def grid_config(self, ax, gridSize, title):
        plt.xlim(self.xRange[0], self.xRange[1])
        plt.ylim(self.yRange[0], self.yRange[1])
        plt.grid(which='minor')
        plt.grid(which='major')
        ax.set_aspect(1)
        ax.set_xlabel('X Position (m)', size=8)
        ax.set_ylabel('Y Position (m)', size=8)
        ax.set_title(title, size=8)
        ax.xaxis.set_major_locator(MultipleLocator(1))
        ax.yaxis.set_major_locator(MultipleLocator(1))
        ax.xaxis.set_minor_locator(MultipleLocator(gridSize))
        ax.yaxis.set_minor_locator(MultipleLocator(gridSize))
        ax.legend(fontsize="8")


    ############### RESET FUNCTIONS ###############

    def reset_robot_path_data(self):
        self.start = [0,0]
        self.goal = [0,0]
        self.robotTargets = []


    def reset_robot_movement_data(self):
        self.odometry = []
        self.robotStates = []
        self.simulatedRobotStates = []
        self.knownOccupiedCells = []


    ############### CONVERSION FUNCTIONS ###############

    def pos_to_cell(self, pos):
        cellX = int(np.floor((pos[0] - self.xRange[0]) / self.gridSize))
        cellY = int(np.floor((pos[1] - self.yRange[0]) / self.gridSize))
        cell = [cellX, cellY]
        return cell


    def cell_to_pos(self, cell):
        posX = ((cell[0] + 0.5) * self.gridSize) + self.xRange[0]
        posY = ((cell[1] + 0.5) * self.gridSize) + self.yRange[0]
        pos = [posX, posY]
        return pos