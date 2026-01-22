#include <cmath>
#include "robot.hpp"
#include <iostream>

namespace Robot{
void Car::update(double acc, double delta, double dt){
  v += acc * dt;
  omega = v * std::tan(delta) / L;
  theta += omega * dt;
  x += v * std::cos(theta) * dt;
  y += v * std::sin(theta) * dt;
  //std::cout << v << std::endl;
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

Eigen::Vector2d Car::get_pos() const {
  return Eigen::Vector2d(x, y);
}

}
