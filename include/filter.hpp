#ifndef FILTER_HPP
#define FILTER_HPP

#include <Eigen/Dense>
#include "robot.hpp"

namespace Filter {

class Ekf {
// EKF(Extended Kalman Filter)
// Prediction Step
// X_k+1|k = F(X_k, U_k+1) + W;
// P_k+1|k = F_k P_k (F_k)^T + Q;

// Update Step
// Y_k+1 = Z_k+1 - H X_k+1|k
// S_k+1 = H P_k+1|k H^T + R
// K_k+1 = P_k+1|k H^T (S_k+1)^{-1}

// hat{X_k+1|k+1} = hat{X_k+1|k} + K_k+1 Y_k+1
// P_k+1|k+1 = (I - K_k+1 H) P_k+1|k

// Back to Prediction Step
// Loop those steps
public:
    // 状態ベクトル: [x, y, theta, v]^T 
    Eigen::Vector4d x;
    // 誤差共分散行列
    Eigen::Matrix4d P;
    // プロセスノイズ共分散 (モデルの不確かさ)
    Eigen::Matrix4d Q;
    // 観測ノイズ共分散 (センサーの不確かさ)
    Eigen::Matrix<double, 1, 1> R_theta; 
    Eigen::Matrix<double, 1, 1> R_v;

    Ekf(double start_x, double start_y, double start_theta, double start_velocity);

    // 予測ステップ: ControlInput (acc, delta) を使用
    void predict(const Robot::ControlInput& u, double dt, double L);

    // 補正ステップ: IMU の方位 (z_theta) を使用
    void update_orientation(double z_theta);
    void update_velocity(double z_v);

    Eigen::Vector2d get_pos() const { return Eigen::Vector2d(x(0), x(1)); }
    double get_theta() const { return x(2); }

};

} // namespace Filter

#endif
