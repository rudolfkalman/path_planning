import numpy as np

# car kinematic model (rear wheel reference point)

# x      [m]
# y      [m]
# theta  [rad]
# v      [m/s]
# acc    [m/s^2]
# delta  [rad]
# omega  [rad/s]

# ICR is the reference point of rotate

# L      [m]  this is the length of front to rear wheel
# R      [m]  this is the length of ICR to rear wheel

# theta' = omega = v / R
# tan(delta) = L / R
# rotation rate equation
# theta' = omega = v / R = v tan(delta) / L // this equation transform is to calculate omega without R (only use L and delta)
class Car:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.v = 0
        self.omega = 0
        self.L = 1

    def update(self, acc, delta, dt):
        self.v += acc * dt
        self.omega = self.v * np.tan(delta) / self.L
        self.theta += self.omega * dt
        self.x += self.v * np.cos(self.theta) * dt
        self.y += self.v * np.sin(self.theta) * dt

    def pos(self):
        return np.array(self.x, self.y)
