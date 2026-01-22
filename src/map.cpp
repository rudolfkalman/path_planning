#include <Eigen/Dense>
#include <vector>
#include "map.hpp"

namespace Map{
Wall::Wall(const Eigen::Vector2d start,
           const Eigen::Vector2d end)
    : start(start), end(end)

{}

Eigen::Vector2d Wall::get_start() const {
  return start;
}

Eigen::Vector2d Wall::get_end() const {
  return end;
}

Eigen::Vector2d Wall::dir_vec() const {
  return end - start;
}

std::vector<Wall> create_map(const std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>> walls) {
  std::vector<Wall> map_walls;
  map_walls.reserve(walls.size());
  /*for (size_t i = 0; i < walls.size(); i++) {
    Eigen::Vector2d wall_start = std::get<0>(walls[i]);
    Eigen::Vector2d wall_end = std::get<1>(walls[i]);
    map_walls.push_back(Wall(wall_start, wall_end));
  }*/
  for (const auto& [start, end] : walls){
    map_walls.emplace_back(start, end);
  }
  return map_walls;
}
}
