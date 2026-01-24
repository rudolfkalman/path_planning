#include "log.hpp"
#include <iomanip>
#include <iostream>

namespace Log {
double rad2deg(double rad){
  return rad * 180.0 / M_PI;
}

double deg2rad(double deg){
  return deg * M_PI / 180.0;
}

void flush_console(const std::vector<std::string>& logs) {
    std::cout << "\033[2J\033[H"; // clear console
    for (const auto& s : logs) {
        std::cout << s << "\n";
    }
    std::cout << std::flush;
}

std::string log_car(double t, const Robot::Car &car) {
  Robot::ControlInput u = car.get_control();
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2) << "[CAR] "
     << "[t=" << t << "] "
     << " x=" << car.get_pos().x() << "m"
     << " y=" << car.get_pos().y() << "m"
     << " v=" << rad2deg(car.get_velocity()) << "deg/s"
     << " theta=" << std::fmod(rad2deg(car.get_theta()), 360.0) << "deg"
     << " acc=" << u.acc << "m/s^2"
     << " delta=" << rad2deg(u.delta) << "deg";
  return ss.str();
}

std::string log_imu(Robot::Car &car, Sensor::Imu &imu, double dt) {
  Eigen::Vector2d acc = imu.get_acc(car, dt);
  double angular_velocity = imu.get_angular_velocity(car, dt);
  double orientation = imu.get_orientation(car, dt);
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2) << "[IMU] "
     << " acc_x=" << acc.x() << " acc_y=" << acc.y()
     << " angular_velocity=" << angular_velocity << " orientation" << std::fmod(rad2deg(orientation), 360.0) << "deg";
  return ss.str();
}

std::string log_odom(Sensor::Odom &odom){
  Eigen::Vector2d odom_pos = odom.get_estimated_pos();
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2) << "[ODOM] "
     << " odom_x=" << odom_pos.x()
     << " odom_y=" << odom_pos.y()
     << " odom_theta=" << std::fmod(rad2deg(odom.get_estimated_theta()), 360.0) << "deg";
  return ss.str();
}

} // namespace Log
