class Cell:
    def __init__(self, x, y, f_cost, g_cost, parent):
        self.pos = [x,y]
        self.f_cost = f_cost
        self.g_cost = g_cost
        self.parent = parent
        self.neighbors = self.return_neighbors()

    def return_neighbors(self):
        x = self.pos[0]
        y = self.pos[1]
        return [[x-1,y+1],[x,y+1],[x+1,y+1],[x-1,y],[x+1,y],[x-1,y-1],[x,y-1],[x+1,y-1]]