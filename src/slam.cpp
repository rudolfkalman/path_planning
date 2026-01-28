#include "slam.hpp"
#include <limits>

namespace Slam{
  
Grid_Cell Calc_Cov(std::vector<Eigen::Vector2d> &Points){

  int len = Points.size();
  if (len == 0) {
    throw std::invalid_argument("Calc_Cov: empty Points");
  }
  Eigen::Vector2d sum = Eigen::Vector2d::Zero();
  for (int i = 0; i < len; i++){
    sum += Points[i];
  }
  Eigen::Vector2d average = sum / len;
  
  Eigen::Vector2d var = Eigen::Vector2d::Zero();
  for(int i = 0; i < len; i++){
    var += (Points[i] - average).array().square().matrix();
  }

  var = var / len;

  double cov = 0.0; 
  for (int i = 0; i < len; i++){
    cov += (Points[i].x() - average.x()) * (Points[i].y() - average.y());
  }
  cov = cov / len;
  Eigen::Matrix2d covariance_matrix(var.x(), cov, cov, var.y());
  return Grid_Cell{average, covariance_matrix, len};
}

Eigen::Matrix3d Make_Affine(Eigen::Vector3d dx){
  Eigen::Matrix3d Affine;
  double theta = dx.z();
  Affine << std::cos(theta), -std::sin(theta), dx.x(),
            std::sin(theta),  std::cos(theta), dx.y(),
            0,              0,             1;
  return Affine;
}

Eigen::Matrix<double, 2, 3> Calc_Jacobian(Eigen::Vector3d &p){
  Eigen::Matrix<double, 2, 3> Jacobian;
  double theta = p.z();
  Jacobian << 1, 0, -std::sin(theta) * p.x() - std::cos(theta) * p.y(),
              0, 1,  std::cos(theta) * p.x() - std::sin(theta) * p.y();
  return Jacobian;
}

Eigen::Vector3d Calc_Grad(Eigen::Vector2d &e, Eigen::Matrix2d &cov_inv, Eigen::Matrix<double, 2, 3> &Jacobian){
  Eigen::Vector3d grad = Jacobian.transpose() * cov_inv * e;
  return grad;
}

Eigen::Matrix3d Calc_Hessian(Eigen::Matrix2d &cov_inv, Eigen::Matrix<double, 2, 3> &Jacobian){
  Eigen::Matrix3d Hessian;
  Hessian << Jacobian.transpose() * cov_inv * Jacobian;
  return Hessian;
}

void NDT::Scan_Matching(std::vector<Eigen::Vector2d> &Current_Scan){
  double Max_x = -std::numeric_limits<double>::infinity();
  double Min_x = std::numeric_limits<double>::infinity();
  double Max_y = -std::numeric_limits<double>::infinity();
  double Min_y = std::numeric_limits<double>::infinity();

  size_t current_size = Current_Scan.size();
  for(size_t i = 0; i < current_size; i++){
    if(Current_Scan[i].x() > Max_x){
      Max_x = Current_Scan[i].x();
    }
    if(Current_Scan[i].x() < Min_x){
      Min_x = Current_Scan[i].x();
    }
    if(Current_Scan[i].y() > Max_y){
      Max_y = Current_Scan[i].y();
    }
    if(Current_Scan[i].y() < Min_y){
      Min_y = Current_Scan[i].y();
    }
  }

  int Grid_Width  = static_cast<int>((Max_x - Min_y) / Cell_Size) + 1;
  int Grid_Height = static_cast<int>((Max_y - Min_y) / Cell_Size) + 1;

  std::vector<std::vector<Point_Cell>> Grid; // this array for to save Point;
  std::vector<std::vector<Grid_Cell>> Grid_Cov; // this array for to save Cov;
  for(int i = 0; i < Grid_Height; i++){
    Grid.push_back(std::vector<Point_Cell>());
    Grid_Cov.push_back(std::vector<Grid_Cell>());
    for(int j = 0; j < Grid_Width; j++){
      Point_Cell point_cell;
      Grid_Cell grid_cell;
      Grid[i].push_back(point_cell);
      Grid_Cov[i].push_back(grid_cell);
    }
  }

  for(size_t i = 0; i < current_size; i++){
    int gx = static_cast<int>((Current_Scan[i].x() - Min_x) / Cell_Size);
    int gy = static_cast<int>((Current_Scan[i].y() - Min_y) / Cell_Size);
    Grid[gy][gx].points.push_back(Current_Scan[i]);
  }

  for(int i = 0; i < Grid_Height; i++){
    for(int j = 0; j < Grid_Width; j++){
      Grid_Cell Cell = Calc_Cov(Grid[i][j].points); // Cell has mean, cov, len of points of cell;
      Grid_Cov[i][j] = Cell;
    }
  }

  // optimization
  Eigen::Vector3d p  = State;
  Eigen::Vector3d dp = Eigen::Vector3d::Zero();
  Eigen::Vector3d prev_dp = Eigen::Vector3d::Constant(Eigen::NumTraits<double>::infinity());
  double eps = 1e-6;
  bool finished = false;

  for(int iter = 0; iter < max_iter; iter++){
    Eigen::Vector3d g = Eigen::Vector3d::Zero();
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();

    Eigen::Matrix<double, 2, 3> Jacobian = Calc_Jacobian(p);
    Eigen::Matrix3d Affine = Calc_Affine(dp);
    for(size_t i = 0; i < Girid_Height; i++){
      for(size_t j = 0; j < Grid_Width; j++){
        int len = Grid[i][j].points.size();
        Grid_Cell cov = Grid_Cov[i][j];
        Grid_Cell cov_inv = cov.ldlt().solve(Eigen::Matrix2d::Identity());
        if(len < 3){
          continue;
        }
        for(int k = 0; k < len; k++){
          Eigen::Vector2d point = Grid[i][j].points[k];
          Eigen::Vector3d point_3d(point.x(), point.y(), 1);
          point_3d = Affine * point_3d;
          point = Eigen::Vector2d(point_3d.x(), point_3d.y()); // rotated point
          Eigen::Vector2d e = point - cov.average
          g += Calc_Grad(e, cov_inv, Jacobian);
          H += Calc_Hessian(cov_inv, Jacobian);
        }
      }
    }
    dp = -H.ldlt.solve(g);
    p = p + dp;

    if(dp.squaredNorm() < eps * eps){
      finished = true;
      break;
    }
    if((prev_dp - dp).squaredNorm() < eps * eps){
      finished = true;
      break;
    }
  }
  if(finished){
    Steate = p
    return dp;
  }
  prev_dp = dp;
  Prev_Scan = Current_Scan;
}

}
