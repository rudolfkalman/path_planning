import numpy as np

# use angle unit rad
class Lidar:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.scan_len = 12 # m
        self.step = np.pi / 360 # rad(1 deg)
        self.rad_min = 0
        self.rad_max = 2 * np.pi

    # return hit points
    def scan(self, robot, walls):
        points = []
        i = self.rad_min
    
        while i < self.rad_max:
            min_dist = np.inf
            min_point = None
            angle = robot.theta + i
            ray = np.array([np.cos(angle), np.sin(angle)])
    
            for wall in walls:
                pos2wall_start = wall.start - robot.pos()
                dir = wall.dir_vec()
    
                den = np.cross(dir, ray)
                if abs(den) < 1e-8:
                    continue
    
                a = np.cross(ray, pos2wall_start) / den
                if a < 0 or a > 1:
                    continue
                
                point = wall.start + a * dir
                if np.dot(point - robot.pos(), ray) > 0: 
                    dist = np.linalg.norm(point - robot.pos())
    
                    if dist < min_dist and dist <= self.scan_len:
                        min_dist = dist
                        min_point = point
    
            if min_point is not None:
                points.append(min_point)
    
            i += self.step
    
        return points
    
