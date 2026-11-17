#pragma once

#include <Eigen/Dense>

namespace mujoco_sim {

class GenericController {
 public:
  virtual ~GenericController() = default;
  virtual Eigen::VectorXd command(const Eigen::VectorXd& error) const = 0;
};

}  // namespace mujoco_sim
