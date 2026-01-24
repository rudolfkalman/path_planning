#ifndef SENSOR_HPP
#define SENSOR_HPP

#define _USE_MATH_DEFINES
#include <cmath>

#include "map.hpp"
#include "robot.hpp"
#include "noise.hpp"

#include <Eigen/Dense>
#include <vector>

namespace Sensor {
double cross2(const Eigen::Vector2d &a, const Eigen::Vector2d &b);

class Lidar {
public:
  Lidar() = default;

  std::vector<Eigen::Vector2d> scan(const Robot::Car &robot,
                                    const std::vector<Map::Wall> &walls);

  Eigen::Vector2d get_pos();
  double get_scan_len();
  double get_scan_len_squared();
  double get_step();
  double get_rad_min();
  double get_rad_max();

  void set_pos(double x, double y);
  void set_scan_len(double len);
  void set_step(double step);
  void set_rad_min(double rad_min);
  void set_rad_max(double rad_max);

private:
  double x = 0.0;
  double y = 0.0;
  double scan_len = 12;
  double scan_len_squared = scan_len * scan_len;
  double step = M_PI / 360;
  double rad_min = 0.0;
  double rad_max = 2 * M_PI;
};
class Imu {
public:
  Imu() = default;
  Eigen::Vector2d get_acc(Robot::Car &car, double dt);
  double get_angular_velocity(Robot::Car &car, double dt);
  double get_orientation(Robot::Car &car, double dt);

private:
  Noise::Noise_Generator noise_acc{0.0, 0.08};
  Noise::Noise_Generator noise_angular_velocity{0.0, 0.01};
  Noise::Noise_Generator noise_orientation{0.0, 0.01};
};

class Odom{
public:
  Odom(double x, double y, double theta);

  void update(Robot::Car &car, double dt);
  Eigen::Vector2d get_estimated_pos() const;
  double get_estimated_theta() const;
private:
  double estimate_x = 0.0;
  double estimate_y = 0.0;
  double estimate_theta = 0.0;
  Noise::Noise_Generator noise_dist{0.0, 1.0};
  Noise::Noise_Generator noise_angle{0.0, 0.01};
};

} // namespace Sensor
#endif
