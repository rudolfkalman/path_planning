#ifndef ROBOT_HPP
#define ROBOT_HPP

#define _USE_MATH_DEFINES

#include <Eigen/Dense>

namespace Robot {
class Car {
public:
  Car() = default;
  void update(double acc, double delta, double dt);
  void set_pos(double x, double y);
  void set_theta(double theta);
  void set_velocity(double v);
  void set_omega(double omega);
  void set_L(double L);
  double get_theta() const;
  Eigen::Vector2d get_pos() const;

private:
  double  x = 0.0;
  double  y = 0.0;
  double  theta = 0.0;
  double  v = 0.0;
  double  omega = 0.0;
  double  L = 1;
};
} // namespace Robot

#endif
