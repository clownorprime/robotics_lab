#pragma once

#include <mujoco/mujoco.h>

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace mujoco_sim {

struct Command {
  std::vector<double> q;
  std::vector<double> dq;
  std::vector<double> tau;
};

// References the owner's current model/data slots so reloads are visible.
struct SimulationContext {
  mjModel*& model;
  mjData*& data;
  std::recursive_mutex& mutex;
};

class Backend {
 public:
  virtual ~Backend() = default;
  virtual void start() = 0;
  virtual void stop() = 0;
};

// Generic ROS 2 backend. Commands and states use Float64MultiArray:
// [q(na), dq(na), tau(na)], where na is the number of actuators.
class Ros2Backend final : public Backend {
 public:
  Ros2Backend(SimulationContext& simulation,
              std::string command_topic, std::string state_topic, double kp, double kd);
  ~Ros2Backend() override;
  void start() override;
  void stop() override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_sim
