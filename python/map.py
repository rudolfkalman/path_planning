import numpy as np

class Wall:
    def __init__(self, start, end):
        self.start = start
        self.end = end
    def dir_vec(self):
        return self.end - self.start

def create_map(walls):
    map_walls = []
    for wall in walls:
        map_walls.append(Wall(np.array([wall[0][0],wall[0][1]]), np.array([wall[1][0], wall[1][1]])))
        #print(map_walls[0].start)
    return map_walls
