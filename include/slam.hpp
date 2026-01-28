#ifndef SLAM_HPP
#define SLAM_HPP

#include <Eigen/Dense>
#include <vector>

namespace Slam {
struct Grid_Cell {
  Eigen::Vector2d mu;
  Eigen::Matrix2d sigma;
  int point_count = 0;
};

struct Point_Cell {
    std::vector<Eigen::Vector2d> points;
};

Grid_Cell Calc_Cov(std::vector<Eigen::Vector2d> &Points);

Eigen::Matrix3d Make_Affine(Eigen::Vector3d dx); // dx -> dx, dy, theta

Eigen::Matrix<double, 2, 3> Calc_Jacobian(Eigen::Vector3d &p);

Eigen::Vector3d Calc_Grad(Eigen::Vector2d &e, Eigen::Matrix2d &cov_inv, Eigen::Matrix<double, 2, 3> &Jacobian);

Eigen::Matrix3d Calc_Hessian(Eigen::Matrix2d &cov_inv, Eigen::Matrix<double, 2, 3> &Jacobian);

class NDT {
public:
  void Scan_Matching(std::vector<Eigen::Vector2d>
                    &Current_Scan); // this return displacement of x, y, theta
private:
  std::vector<Eigen::Vector2d> Prev_Scan;
  std::vector<std::vector<Grid_Cell>> global_map;
  Eigen::Vector3d State;
  double Cell_Size = 0.1;
};

} // namespace Slam

#endif
