#include <iostream>
#include <tuple>

#include "map.hpp"
#include "path_planning.hpp"
#include "robot.hpp"
#include "sensor.hpp"
#include "visualize.hpp"
#include "log.hpp"
#include <SDL3/SDL.h>

#define WINDOW_W 1280
#define WINDOW_H 720
Visualize::Visualizer visualizer(WINDOW_W, WINDOW_H);

constexpr int TARGET_FPS = 240;
constexpr Uint32 FRAME_MS = 1000 / TARGET_FPS;

constexpr double FIXED_DT = 0.05;

int main() {
  visualizer.Init();

  Robot::Car car;
  car.set_pos(2, 2);
  car.set_L(2);
  car.set_theta(0);
  car.set_velocity(1);
  double acc = 0.0;
  double delta = 0;

  std::vector<Map::Wall> world_map{
      Map::Wall({1, 1}, {7, 1}), Map::Wall({7, 1}, {7, 5}),
      Map::Wall({7, 5}, {1, 5}), Map::Wall({1, 5}, {1, 1}),
      Map::Wall({2, 3}, {2, 4}), Map::Wall({2, 4}, {4, 4}),
      Map::Wall({4, 4}, {4, 3}), Map::Wall({4, 3}, {2, 3})};

  Sensor::Lidar lidar;
  Eigen::Vector2d car_pos = car.get_pos();
  lidar.set_pos(car_pos.x(), car_pos.y());

  Sensor::Imu imu;
  Sensor::Odom odom(car.get_pos().x(), car.get_pos().y(), car.get_theta());

  bool running = true;
  Uint32 prev_time = SDL_GetTicks();
  double accumulator = 0.0;

  double sim_time = 0.0;
  while (running) {
    Uint32 current_time = SDL_GetTicks();
    double frame_time = (current_time - prev_time) / 1000.0;
    accumulator += frame_time;
    prev_time = current_time;

    while (SDL_PollEvent(&visualizer.event)) {
      if (visualizer.event.type == SDL_EVENT_QUIT) {
        running = false;

        SDL_DestroyRenderer(visualizer.renderer);
        SDL_DestroyWindow(visualizer.window);
        SDL_Quit();
        return 0;
      }
    }

    delta = 0.0;
    acc = 0.0;

    const bool *keys = SDL_GetKeyboardState(nullptr);

    if (keys[SDL_SCANCODE_W])
      acc = 1;
    else if (keys[SDL_SCANCODE_S])
      acc = -1;
    else
      acc = 0.0;

    if (keys[SDL_SCANCODE_A])
      delta += M_PI / 6;
    if (keys[SDL_SCANCODE_D])
      delta -= M_PI / 6;

    while (accumulator >= FIXED_DT) {
      // update
      car.update(acc, delta, FIXED_DT);
      odom.update(car, FIXED_DT);

      // log
      std::vector<std::string> logs;
      logs.push_back(Log::log_car(sim_time, car));
      logs.push_back(Log::log_imu(car, imu, FIXED_DT));
      logs.push_back(Log::log_odom(odom));
      Log::flush_console(logs);

      // fix dt
      sim_time += FIXED_DT;
      accumulator -= FIXED_DT;
    }

    car_pos = car.get_pos();
    lidar.set_pos(car_pos.x(), car_pos.y());

    SDL_SetRenderDrawColor(visualizer.renderer, 0, 0, 0, 255);
    SDL_RenderClear(visualizer.renderer);
    visualizer.draw_car(car, delta);
    visualizer.draw_map(world_map);
    visualizer.draw_lidar_scan(car, lidar, world_map, false);

    SDL_RenderPresent(visualizer.renderer);
    Uint32 frame_ms = SDL_GetTicks() - current_time;
    if (frame_ms < FRAME_MS) {
      SDL_Delay(FRAME_MS - frame_ms);
    }
  }
}
