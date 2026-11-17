#include "lqr_controller.h"

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <array>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
struct ModelDeleter {
  void operator()(mjModel* model) const { mj_deleteModel(model); }
};
struct DataDeleter {
  void operator()(mjData* data) const { mj_deleteData(data); }
};

class LqrControllerNode final : public rclcpp::Node {
 public:
  LqrControllerNode() : Node("lqr_controller") {
    const auto robot = declare_parameter<std::string>("robot", "inverted_pendulum");
    const auto model_path = declare_parameter<std::string>("model_path", "");
    const auto q_weights = declare_parameter<std::vector<double>>("q_weights", {-1.0});
    const auto r_weights = declare_parameter<std::vector<double>>("r_weights", {-1.0});
    if (robot.empty() || robot.find('/') != std::string::npos) {
      throw std::invalid_argument("robot must be a nonempty ROS name component");
    }
    if (model_path.empty()) {
      throw std::invalid_argument("model_path ROS parameter is required");
    }
    configure_from_xml(model_path, q_weights, r_weights);
    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/" + robot + "/command", 10);
    state_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/" + robot + "/state", 10,
        [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr state) {
          on_state(*state);
        });
    RCLCPP_INFO(get_logger(), "LQR ready: model=%s, nx=%d, nu=%d",
                model_path.c_str(), nx_, nu_);
  }

 private:
  void configure_from_xml(const std::filesystem::path& xml_path,
                          const std::vector<double>& q_input,
                          const std::vector<double>& r_input) {
    if (!std::filesystem::is_regular_file(xml_path)) {
      throw std::runtime_error("MuJoCo XML does not exist: " + xml_path.string());
    }
    std::array<char, 1024> error{};
    model_.reset(mj_loadXML(xml_path.c_str(), nullptr, error.data(), error.size()));
    if (!model_) {
      throw std::runtime_error("Could not load MuJoCo XML: " +
                               xml_path.string() + ": " + error.data());
    }
    const mjModel* m = model_.get();
    nq_ = m->nq;
    nv_ = m->nv;
    nu_ = m->nu;
    na_ = m->na;
    nx_ = 2 * nv_ + na_;
    if (nv_ == 0 || nu_ == 0) {
      throw std::runtime_error("LQR requires model degrees of freedom and actuators");
    }
    reference_qpos_.assign(m->qpos0, m->qpos0 + nq_);
    lower_.resize(nu_);
    upper_.resize(nu_);
    limited_.resize(nu_);
    for (int i = 0; i < nu_; ++i) {
      limited_[i] = m->actuator_ctrllimited[i];
      lower_[i] = m->actuator_ctrlrange[2 * i];
      upper_[i] = m->actuator_ctrlrange[2 * i + 1];
    }

    // MuJoCo's finite-difference transition API cannot linearize RK4.
    // This private model can use Euler while the simulator retains its integrator.
    model_->opt.integrator = mjINT_EULER;
    std::unique_ptr<mjData, DataDeleter> data(mj_makeData(m));
    if (!data) throw std::runtime_error("Could not allocate MuJoCo linearization data");
    mj_resetData(m, data.get());
    mj_forward(m, data.get());
    std::vector<mjtNum> a_values(nx_ * nx_);
    std::vector<mjtNum> b_values(nx_ * nu_);
    mjd_transitionFD(m, data.get(), 1e-6, 1,
                     a_values.data(), b_values.data(), nullptr, nullptr);
    const Eigen::Map<const Eigen::Matrix<mjtNum, Eigen::Dynamic, Eigen::Dynamic,
                                        Eigen::RowMajor>> a(a_values.data(), nx_, nx_);
    const Eigen::Map<const Eigen::Matrix<mjtNum, Eigen::Dynamic, Eigen::Dynamic,
                                        Eigen::RowMajor>> b(b_values.data(), nx_, nu_);

    Eigen::VectorXd q = Eigen::VectorXd::Ones(nx_);
    // Useful defaults for cart pendulums; other systems should set q_weights.
    for (int i = 0; i < nv_; ++i) q[i] = (i == 0 ? 10.0 : 100.0);
    for (int i = 0; i < nv_; ++i) q[nv_ + i] = (i == 0 ? 1.0 : 10.0);
    Eigen::VectorXd r = Eigen::VectorXd::Ones(nu_);
    if (q_input != std::vector<double>{-1.0}) {
      if (q_input.size() != static_cast<size_t>(nx_))
        throw std::invalid_argument("q_weights length must equal 2*nv+na");
      q = Eigen::Map<const Eigen::VectorXd>(q_input.data(), nx_);
    }
    if (r_input != std::vector<double>{-1.0}) {
      if (r_input.size() != static_cast<size_t>(nu_))
        throw std::invalid_argument("r_weights length must equal nu");
      r = Eigen::Map<const Eigen::VectorXd>(r_input.data(), nu_);
    }
    controller_.configure(a, b, q, r);
  }

  void on_state(const std_msgs::msg::Float64MultiArray& state) {
    // Simulator layout: [qpos(nq), qvel(nv), activation(na), force(nu)].
    const size_t expected = static_cast<size_t>(nq_ + nv_ + na_ + nu_);
    if (state.data.size() != expected) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Expected %zu state values, received %zu",
                           expected, state.data.size());
      return;
    }
    Eigen::VectorXd error(nx_);
    mj_differentiatePos(model_.get(), error.data(), 1.0,
                        reference_qpos_.data(), state.data.data());
    for (int i = 0; i < nv_; ++i) error[nv_ + i] = state.data[nq_ + i];
    for (int i = 0; i < na_; ++i) error[2 * nv_ + i] = state.data[nq_ + nv_ + i];
    const Eigen::VectorXd u = controller_.command(error);
    std_msgs::msg::Float64MultiArray command;
    command.data.assign(3 * nu_, 0.0);
    for (int i = 0; i < nu_; ++i) {
      command.data[2 * nu_ + i] =
          limited_[i] ? std::clamp(u[i], lower_[i], upper_[i]) : u[i];
    }
    command_pub_->publish(command);
  }

  std::unique_ptr<mjModel, ModelDeleter> model_;
  int nq_ = 0, nv_ = 0, nu_ = 0, na_ = 0, nx_ = 0;
  std::vector<double> reference_qpos_, lower_, upper_;
  std::vector<bool> limited_;
  mujoco_sim::LqrController controller_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_sub_;
};
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<LqrControllerNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("lqr_controller"), "%s", error.what());
    result = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
