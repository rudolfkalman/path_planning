import numpy as np
import pygame
import matplotlib
import matplotlib.pyplot as plt
import path_planning

WIDTH, HEIGHT = 800, 800
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

car = path_planning.Car()

car.x = 200
car.y = 300
car.theta = 0
car.v = 1.0
dt = 0.05
acc = 0.0
#delta = np.deg2rad(20)
delta = 0

# map (walls)
walls = [((100,100),(700,100)), ((700,100),(700,500)), ((700,500),(100,500)), ((100,500),(100,100))]


running = True
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
        delta += 0.02
    elif keys[pygame.K_d]:
        delta -= 0.02

    print(f"delta: {delta}")

    # update car model
    car.update(acc, delta, dt)
    # draw
    screen.fill((0,0,0))
    # map
    for wall in walls:
        pygame.draw.line(screen, (255,255,255), wall[0], wall[1], 2)
    # car
    pygame.draw.circle(screen, (0,0,255), (int(car.x), int(car.y)), 5)
    # LIDAR (simple)
    for angle in np.linspace(0, 2 * np.pi, 90):
        dx = np.cos(car.theta + angle) * 100
        dy = np.sin(car.theta + angle) * 100
        pygame.draw.line(screen, (255,0,0), (int(car.x), int(car.y)), (int(car.x+dx), int(car.y+dy)), 1)

    pygame.display.flip()
    clock.tick(20)  # FPS
