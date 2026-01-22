#ifndef PATH_PLANNING_VISUALIZE_HPP
#define PATH_PLANNING_VISUALIZE_HPP

#define _USE_MATH_DEFINES

#include <Eigen/Dense>
#include <SDL3/SDL.h>
#include <SDL3/SDL_render.h>
#include <vector>

#include "robot.hpp"
#include "sensor.hpp"
#include "map.hpp"

namespace Visualize {
class Visualizer {
public:
  Visualizer(const int window_w, const int window_h);

  int Init();
  void draw_car(Robot::Car &car, double delta);
  void draw_map(std::vector<Map::Wall> &world_map);
  void draw_lidar_scan(Robot::Car &car, Sensor::Lidar &lidar, std::vector<Map::Wall> &world_map);
  void draw_filled_circle(SDL_Renderer* renderer, int cx, int cy, int radius);

  int window_w = 1280;
  int window_h = 720;
  SDL_Window *window = nullptr;
  SDL_Renderer *renderer = nullptr;
  SDL_Event event;
};
} // namespace Visualize

#endif
