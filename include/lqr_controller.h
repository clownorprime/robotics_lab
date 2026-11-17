#pragma once
#include "generic_controller.h"
#include <algorithm>
#include <stdexcept>

namespace mujoco_sim {
class LqrController final : public GenericController {
 public:
  void configure(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b,
                 const Eigen::VectorXd& q_weights,
                 const Eigen::VectorXd& r_weights) {
    if (a.rows() == 0 || a.rows() != a.cols() || b.rows() != a.rows() ||
        b.cols() == 0 || q_weights.size() != a.rows() ||
        r_weights.size() != b.cols() ||
        (q_weights.array() <= 0).any() || (r_weights.array() <= 0).any()) {
      throw std::invalid_argument("Invalid LQR dimensions or weights");
    }
    solve(a, b, q_weights.asDiagonal(), r_weights.asDiagonal());
  }
  Eigen::VectorXd command(const Eigen::VectorXd& error) const override {
    if (error.size() != k_.cols()) throw std::invalid_argument("Invalid LQR state dimension");
    return -k_ * error;
  }
  const Eigen::MatrixXd& gain() const { return k_; }
 private:
  void solve(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b,
             const Eigen::MatrixXd& q, const Eigen::MatrixXd& r) {
    Eigen::MatrixXd p = q;
    bool converged = false;
    for (int i = 0; i < 50000; ++i) {
      Eigen::MatrixXd k = (r + b.transpose()*p*b).ldlt().solve(b.transpose()*p*a);
      Eigen::MatrixXd next = q + a.transpose()*p*a - a.transpose()*p*b*k;
      next = (0.5 * (next + next.transpose())).eval();
      if (!next.allFinite()) throw std::runtime_error("Riccati iteration diverged");
      if ((next-p).norm() < 1e-9 * std::max(1.0, next.norm())) {
        p = next; converged = true; break;
      }
      p = next;
    }
    if (!converged) throw std::runtime_error("Riccati iteration did not converge");
    k_ = (r+b.transpose()*p*b).ldlt().solve(b.transpose()*p*a);
  }
  Eigen::MatrixXd k_;
};
}
