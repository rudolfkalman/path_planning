#ifndef MAP_HPP
#define MAP_HPP

#include <Eigen/Dense>
#include <vector>
#include <tuple>

namespace Map {

class Wall {
public:
  Wall(const Eigen::Vector2d start, const Eigen::Vector2d end);
  Eigen::Vector2d get_start() const;
  Eigen::Vector2d get_end() const;
  Eigen::Vector2d dir_vec() const;

private:
  Eigen::Vector2d start;
  Eigen::Vector2d end;
};

std::vector<Wall> create_map(const std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>& walls);
} // namespace Map

#endif
