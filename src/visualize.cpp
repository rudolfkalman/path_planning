#include "visualize.hpp"
#include <cmath>
#include <iostream>

namespace Visualize {
Visualizer::Visualizer(const int window_w, const int window_h)
    : window_w(window_w), window_h(window_h), window(nullptr),
      renderer(nullptr) {}

int Visualizer::Init() {
  if (!SDL_Init(SDL_INIT_VIDEO)) {
    std::cout << "Failed to Init SDL" << std::endl;
    return 1;
  }

  window =
      SDL_CreateWindow("AutoDriveSimulator", this->window_w, this->window_h, 0);

  if (!window) {
    std::cout << "Failed to Create Window" << std::endl;
    return 1;
  }

  renderer = SDL_CreateRenderer(window, NULL);
  if (!renderer) {
    std::cout << "Failed to Create Renderer" << std::endl;
    return 1;
  }
  SDL_RenderClear(renderer);
  SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
  SDL_RenderPresent(renderer);
  return 0;
}

void Visualizer::draw_filled_circle(SDL_Renderer *renderer, int cx, int cy,
                                    int radius) {
  for (int w = -radius; w <= radius; w++) {
    for (int h = -radius; h <= radius; h++) {
      if (w * w + h * h <= radius * radius) {
        SDL_RenderPoint(renderer, cx + w, cy + h);
      }
    }
  }
}

void Visualizer::draw_car(Robot::Car &car, double delta) {
  Eigen::Vector2d car_pos = car.get_pos();
  double theta = car.get_theta();

  SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
  Visualizer::draw_filled_circle(renderer, car_pos.x() * 100,
                                 window_h - car_pos.y() * 100, 10);
}

void Visualizer::draw_map(std::vector<Map::Wall> &world_map) {
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  for (const Map::Wall &wall : world_map) {
    Eigen::Vector2d s = wall.get_start();
    Eigen::Vector2d e = wall.get_end();
    SDL_RenderLine(renderer, s.x() * 100, window_h - s.y() * 100, e.x() * 100,
                   window_h - e.y() * 100);
  }
}

void Visualizer::draw_lidar_scan(Robot::Car &car, Sensor::Lidar &lidar,
                                 std::vector<Map::Wall> &world_map) {
  SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
  Eigen::Vector2d car_pos = car.get_pos();
  for (double angle = lidar.get_rad_min(); angle < lidar.get_rad_max(); angle+=lidar.get_step()) {
    double dx = std::cos(car.get_theta() + angle) * lidar.get_scan_len();
    double dy = std::sin(car.get_theta() + angle) * lidar.get_scan_len();
    SDL_RenderLine(renderer, car_pos.x() * 100, window_h - car_pos.y() * 100,
                   (car_pos.x() + dx) * 100,
                   window_h - (car_pos.y() + dy) * 100);
  }

  std::vector<Eigen::Vector2d> points = lidar.scan(car, world_map);
  SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
  for (Eigen::Vector2d &p : points) {
    Visualizer::draw_filled_circle(renderer, p.x() * 100,
                                   window_h - p.y() * 100, 10);
  }
}

} // namespace Visualize
