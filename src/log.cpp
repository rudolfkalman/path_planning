#include "log.hpp"
#include <iostream>
#include <iomanip>

namespace Log{

void log_console_car(double t, const Robot::Car& car) {
  Robot::ControlInput u = car.get_control();
    std::cout << std::fixed << std::setprecision(2)
              << "[t=" << t << "] "
              << " x=" << car.get_pos().x()
              << " y=" << car.get_pos().y()
              << " v=" << car.get_velocity()
              << " theta=" << car.get_theta() * 180 / M_PI << "deg"
              << " acc=" << u.acc
              << " delta=" << u.delta
              << "\r" << std::flush;
}
}
