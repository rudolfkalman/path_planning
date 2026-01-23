#include <cmath>
#include "robot.hpp"

namespace Robot{
void Car::update(double acc, double delta, double dt){
  this->set_control(acc, delta);
  prev_v = v;
  v += acc * dt;
  prev_omega = omega;
  omega = v * std::tan(delta) / L;
  prev_theta = theta;
  theta += omega * dt;
  prev_x = x;
  prev_y = y;
  x += v * std::cos(theta) * dt;
  y += v * std::sin(theta) * dt;
}

void Car::set_pos(double x, double y){
  this->x = x;
  this->y = y;
}

void Car::set_theta(double theta){
  this->theta = theta;
}

void Car::set_velocity(double v){
  this->v = v;
}

void Car::set_omega(double omega){
  this->omega = omega;
}

void Car::set_L(double L){
  this->L = L;
}

double Car::get_theta() const {
  return theta;
}
double Car::get_velocity() const {
  return v;
}

Eigen::Vector2d Car::get_pos() const {
  return Eigen::Vector2d(x, y);
}

void Car::set_control(double acc, double delta){
  u.acc = acc;
  u.delta = delta;
}

ControlInput Car::get_control() const {
  return u;
}

}
