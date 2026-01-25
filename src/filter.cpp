#define _USE_MATH_DEFINE

#include "filter.hpp"
#include <cmath>

namespace Filter {

Ekf::Ekf(double start_x, double start_y, double start_theta, double start_velocity) {
    // 状態の初期化 [x, y, theta, v]
    x << start_x, start_y, start_theta, start_velocity;

    // 誤差共分散行列 P の初期化（最初は少し不確か）
    P = Eigen::Matrix4d::Identity() * 0.1;

    // プロセスノイズ Q の設定（モデルの信頼度）
    // 値が小さいほどモデルを信じ、大きいほどセンサーを信じるようになります
    Q = Eigen::Matrix4d::Identity();
    Q(0, 0) = 0.01; // x
    Q(1, 1) = 0.01; // y
    Q(2, 2) = 0.02; // theta
    Q(3, 3) = 0.1;  // v

    // 観測ノイズ R の設定（IMU 方位の信頼度）
    // センサーの sigma^2 程度に設定します
    R_theta << 0.01; 
    R_v << 0.05;
}

void Ekf::predict(const Robot::ControlInput& u, double dt, double L) {
    double prev_theta = x(2);
    double prev_v = x(3);

    // 1. 状態の予測 (非線形モデル f(x, u))
    // robot.cpp の物理モデルと同じ計算を適用
    double v_new = prev_v + u.acc * dt;
    double omega = v_new * std::tan(u.delta) / L;
    double theta_new = prev_theta + omega * dt;
    
    x(0) += v_new * std::cos(theta_new) * dt;
    x(1) += v_new * std::sin(theta_new) * dt;
    x(2) = theta_new;
    x(3) = v_new;

    // 2. ヤコビ行列 F_k の計算 (df/dx)
    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    // 各要素の偏微分を代入
    F(0, 2) = -v_new * std::sin(theta_new) * dt; // dx/d_theta
    F(0, 3) = std::cos(theta_new) * dt;          // dx/dv
    F(1, 2) = v_new * std::cos(theta_new) * dt;  // dy/d_theta
    F(1, 3) = std::sin(theta_new) * dt;          // dy/dv
    F(2, 3) = (std::tan(u.delta) / L) * dt;      // d_theta/dv

    // 3. 予測誤差共分散の更新
    P = F * P * F.transpose() + Q;
}

void Ekf::update_orientation(double z_theta) {
    // 観測行列 H (theta のみを取り出す [0, 0, 1, 0])
    Eigen::Matrix<double, 1, 4> H;
    H << 0, 0, 1, 0;

    // 観測残差 (Innovation) Y
    double y = z_theta - x(2);

    // 角度の正規化 [-PI, PI]
    // これを忘れると、角度が 180度 を跨いだ瞬間にフィルタが爆発します
    while (y > M_PI)  y -= 2.0 * M_PI;
    while (y < -M_PI) y += 2.0 * M_PI;

    // 残差共分散 S
    double S = (H * P * H.transpose())(0, 0) + R_theta(0, 0);

    // カルマンゲイン K
    Eigen::Vector4d K = P * H.transpose() / S;

    // 状態の修正
    x = x + K * y;

    // 誤差共分散の更新
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    P = (I - K * H) * P;
}

void Ekf::update_velocity(double z_v){
  Eigen::Matrix<double, 1, 4> H;
  H << 0, 0, 0, 1;

  double y = z_v - x(3);

  double S = (H * P * H.transpose())(0, 0) + R_v(0, 0);
  
  Eigen::Vector4d K = P * H.transpose() / S;

  x = x + K * y;
  P = (Eigen::Matrix4d::Identity() - K * H) * P;
}

} // namespace Filter
