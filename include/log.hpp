#ifndef LOG_HPP
#define LOG_HPP

#include "robot.hpp"
#include "sensor.hpp"

namespace Log{
double rad2deg(double rad);
double deg2rad(double deg);
void flush_console(const std::vector<std::string> &logs);

std::string log_car(double t, const Robot::Car& car);
std::string log_imu(Robot::Car& car, Sensor::Imu &imu, double dt);
std::string log_odom(Sensor::Odom &odom);
}

#endif
