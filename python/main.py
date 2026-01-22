import numpy as np
import pygame
import matplotlib
import matplotlib.pyplot as plt

import path_planning
import map
import lidar

WIDTH, HEIGHT = 800, 800
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

car = path_planning.Car()

car.x = 2
car.y = 2
car.L = 2
car.theta = 0
car.v = 1.0
dt = 0.05
acc = 0.0
#delta = np.deg2rad(20)
delta = 0.0

# map (walls)
walls = [((1,1),(7,1)), ((7,1),(7,5)), ((7,5),(1,5)), ((1,5),(1,1)), ((2,3), (2,4)), ((2,4), (4,4)), ((4, 4), (4,3)), ((4,3), (2,3))]

world_map = map.create_map(walls)
lidar = lidar.Lidar()
lidar.x = car.x
lidar.y = car.y

running = True

def to_screen(p):
    return int(p[0] * 100), int(HEIGHT - p[1] * 100)

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed() 

    if keys[pygame.K_w]:
        acc = 10.0
    elif keys[pygame.K_s]:
        acc = -10.0
    else:
        acc = 0.0

    if keys[pygame.K_a]:
        delta += 0.2
    elif keys[pygame.K_d]:
        delta -= 0.2

    #print(f"delta: {delta}")

    # update car model
    car.update(acc, delta, dt)
    lidar.x = car.x
    lidar.y = car.y
    # draw
    screen.fill((0,0,0))
    # map
    for wall in walls:
        pygame.draw.line(screen, (255,255,255), to_screen(wall[0]), to_screen(wall[1]), 2)
    # car
    pygame.draw.circle(screen, (0,0,255), to_screen((car.x, car.y)), 10)
    # LIDAR (simple)
    start = to_screen((car.x, car.y))
    for angle in np.linspace(0, 2 * np.pi, 360):
        dx = np.cos(car.theta + angle) * lidar.scan_len
        dy = np.sin(car.theta + angle) * lidar.scan_len
        end = to_screen((car.x + dx, car.y + dy))
        pygame.draw.line(screen, (255, 0, 0), start, end, 1)

    hit_points = lidar.scan(car, world_map)
    for point in hit_points:
        pygame.draw.circle(screen, (0, 0, 255), to_screen(point), 10)

    pygame.display.flip()
    clock.tick(240)  # FPS
