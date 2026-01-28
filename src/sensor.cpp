#include "sensor.hpp"
#include <cmath>
#include <limits>

namespace Sensor {
double cross2(const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
  return a.x() * b.y() - a.y() * b.x();
}

std::vector<Eigen::Vector2d> Lidar::scan(const Robot::Car &robot, const std::vector<Map::Wall> &walls) {
  std::vector<Eigen::Vector2d> points;
  for (double i = rad_min; i < rad_max; i += step) {
    double min_dist = std::numeric_limits<double>::infinity();
    Eigen::Vector2d min_point;
    bool min_point_initialized = false;
    double angle = robot.get_theta() + i;
    Eigen::Vector2d ray(std::cos(angle), std::sin(angle));
    for (size_t j = 0; j < walls.size(); j++) {
      const Eigen::Vector2d robot_pos = robot.get_pos();
      const Eigen::Vector2d pos2wall_start = walls[j].get_start() - robot_pos;
      const Eigen::Vector2d dir = walls[j].dir_vec();
      const double den = cross2(dir, ray);
      if (std::abs(den) < 1e-8) {
        continue;
      }
      double a = cross2(ray, pos2wall_start) / den;
      if (a < 0 || a > 1) {
        continue;
      }
      Eigen::Vector2d point = walls[j].get_start() + a * dir;
        if ((point - robot_pos).dot(ray) > 0) {
        double dist = (point - robot_pos).squaredNorm();
        if (dist < min_dist && dist <= scan_len_squared) {
          min_dist = dist;
          min_point = point;
          min_point_initialized = true;
        }
        }
    }

    if (min_point_initialized) {
      Eigen::Vector2d scan_noise(noise.generate_gaussian_noise(), noise.generate_gaussian_noise());
      points.push_back(min_point + scan_noise);
    }
  }
  return points;
}

Eigen::Vector2d Lidar::get_pos(){
  return Eigen::Vector2d(x, y);
}

double Lidar::get_scan_len(){
  return scan_len;
}

double Lidar::get_scan_len_squared(){
  return scan_len_squared;
}

double Lidar::get_step(){
  return step;
}

double Lidar::get_rad_min(){
  return rad_min;
}

double Lidar::get_rad_max(){
  return rad_max;
}

void Lidar::set_pos(double x, double y){
  this->x = x;
  this->y = y;
}

void Lidar::set_scan_len(double len){
  this->scan_len = len;
  this->scan_len_squared = this->scan_len * this->scan_len;
}

void Lidar::set_step(double step){
  this->step = step;
}

void Lidar::set_rad_min(double rad_min){
  this->rad_min = rad_min;
}

void Lidar::set_rad_max(double rad_max){
  this->rad_max = rad_max;
}

Eigen::Vector2d Imu::get_acc(Robot::Car &car, double dt){
  Robot::ControlInput u = car.get_control();
  double measurement_acc_x = u.acc + noise_acc.generate_gaussian_noise();
  double measurement_acc_y = car.get_velocity() * car.get_omega() + noise_acc.generate_gaussian_noise();
  return Eigen::Vector2d(measurement_acc_x, measurement_acc_y);
}

double Imu::get_angular_velocity(Robot::Car &car, double dt){
  double measurement_angular_velocity = car.get_omega() + noise_angular_velocity.generate_gaussian_noise();
  return measurement_angular_velocity;
}

double Imu::get_orientation(Robot::Car &car, double dt){
  double measurement_theta = car.get_theta() + noise_orientation.generate_gaussian_noise();
  return measurement_theta;
}

Odom::Odom(double x, double y, double theta)
  // initial state x, y, theta is estimate_x, estimate_y, estimate_theta
  : estimate_x(x),
    estimate_y(y),
    estimate_theta(theta)
{}

void Odom::update(Robot::Car &car, double dt){
  if(car.get_velocity() < 0.001 && car.get_velocity() > -0.001){
    return;
  }
  double displacement = car.get_velocity() * dt;
  double delta_theta = car.get_omega() * dt;

  displacement = displacement + noise_dist.generate_gaussian_noise();
  delta_theta = delta_theta + noise_angle.generate_gaussian_noise();

  estimate_theta += delta_theta;
  estimate_x += displacement * std::cos(estimate_theta);
  estimate_y += displacement * std::sin(estimate_theta);
}

Eigen::Vector2d Odom::get_estimated_pos() const {
  return Eigen::Vector2d(estimate_x, estimate_y);
}

double Odom::get_estimated_theta() const {
  return estimate_theta;
}

} // namespace Sensor
