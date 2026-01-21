import pygame
import numpy as np

# 初期設定
WIDTH, HEIGHT = 800, 600
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# 車両状態
car_x, car_y = 400, 300
theta = 0
v = 1.0
L = 50  # wheel base
delta = np.deg2rad(20)
acc = 0.0
dt = 0.05

# 障害物例（線分）
walls = [((100,100),(700,100)), ((700,100),(700,500)), ((700,500),(100,500)), ((100,500),(100,100))]

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # 車両モデル更新
    omega = v * np.tan(delta) / L
    theta += omega * dt
    car_x += v * np.cos(theta) * dt
    car_y += v * np.sin(theta) * dt

    # 描画
    screen.fill((0,0,0))
    # 障害物
    for wall in walls:
        pygame.draw.line(screen, (255,255,255), wall[0], wall[1], 2)
    # 車体
    pygame.draw.circle(screen, (0,0,255), (int(car_x), int(car_y)), 5)
    # LIDAR（簡易例）
    for angle in np.linspace(-np.pi/4, np.pi/4, 10):
        dx = np.cos(theta + angle) * 100
        dy = np.sin(theta + angle) * 100
        pygame.draw.line(screen, (255,0,0), (int(car_x), int(car_y)), (int(car_x+dx), int(car_y+dy)), 1)

    pygame.display.flip()
    clock.tick(20)  # FPS

