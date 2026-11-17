#include "generic_backend.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <chrono>

namespace mujoco_sim {

struct Ros2Backend::Impl {
  Impl(SimulationContext& simulation,
       std::string command, std::string state, double p, double v)
      : simulation(simulation), kp(p), kd(v),
        command_topic(std::move(command)), state_topic(std::move(state)),
        command_q(simulation.model->nu, 0.0), command_dq(simulation.model->nu, 0.0),
        command_tau(simulation.model->nu, 0.0) {}

  SimulationContext& simulation;
  double kp, kd;
  std::string command_topic, state_topic;
  std::vector<double> command_q, command_dq, command_tau;
  std::mutex command_mutex;
  std::atomic<bool> running{false};
  std::thread thread;
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub;
};

Ros2Backend::Ros2Backend(SimulationContext& simulation,
                         std::string command_topic, std::string state_topic, double kp, double kd)
    : impl_(std::make_unique<Impl>(simulation, std::move(command_topic),
                                   std::move(state_topic), kp, kd)) {}

Ros2Backend::~Ros2Backend() { stop(); }

void Ros2Backend::start() {
  if (impl_->running.exchange(true)) return;
  impl_->node = std::make_shared<rclcpp::Node>("mujoco_sim_backend");
  impl_->sub = impl_->node->create_subscription<std_msgs::msg::Float64MultiArray>(
      impl_->command_topic, rclcpp::QoS(10),
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        std::lock_guard<std::recursive_mutex> sim_lock(impl_->simulation.mutex);
        const size_t n = impl_->simulation.model->nu;
        if (msg->data.size() != 3 * n) {
          RCLCPP_WARN(impl_->node->get_logger(), "Command requires %zu values", 3 * n);
          return;
        }
        std::lock_guard<std::mutex> lock(impl_->command_mutex);
        if (impl_->command_q.size() != n) {
          impl_->command_q.assign(n, 0.0);
          impl_->command_dq.assign(n, 0.0);
          impl_->command_tau.assign(n, 0.0);
        }
        std::copy_n(msg->data.begin(), n, impl_->command_q.begin());
        std::copy_n(msg->data.begin() + n, n, impl_->command_dq.begin());
        std::copy_n(msg->data.begin() + 2 * n, n, impl_->command_tau.begin());
      });
  impl_->pub = impl_->node->create_publisher<std_msgs::msg::Float64MultiArray>(
      impl_->state_topic, rclcpp::QoS(10));
  impl_->thread = std::thread([this] {
    while (impl_->running && rclcpp::ok()) {
      rclcpp::spin_some(impl_->node);
      {
        std::lock_guard<std::recursive_mutex> sim_lock(impl_->simulation.mutex);
        std::lock_guard<std::mutex> cmd_lock(impl_->command_mutex);
        const mjModel* m = impl_->simulation.model;
        const mjData* d = impl_->simulation.data;
        if (!m || !d) break;
        for (int i = 0; i < m->nu; ++i) {
          // A generic actuator need not correspond to qpos[i]; use raw command
          // torque here. The controller publishes zero q/dq references.
          impl_->simulation.data->ctrl[i] = impl_->command_tau[i];
        }
        std_msgs::msg::Float64MultiArray state;
        state.data.reserve(m->nq + m->nv + m->na + m->nu);
        state.data.insert(state.data.end(), d->qpos, d->qpos + m->nq);
        state.data.insert(state.data.end(), d->qvel, d->qvel + m->nv);
        state.data.insert(state.data.end(), d->act, d->act + m->na);
        state.data.insert(state.data.end(), d->actuator_force, d->actuator_force + m->nu);
        impl_->pub->publish(state);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  });
}

void Ros2Backend::stop() {
  if (!impl_ || !impl_->running.exchange(false)) return;
  if (impl_->thread.joinable()) impl_->thread.join();
  if (rclcpp::ok()) rclcpp::shutdown();
}
}  // namespace mujoco_sim
