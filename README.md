# mujoco_sim

通用 ROS 2 MuJoCo 控制器，支持 Go2、G1、pendulum 以及自定义 MuJoCo 模型。项目只使用 ROS 2 后端，不依赖 Unitree SDK2 或 Unitree DDS。

ROS 2 后端使用 `std_msgs/msg/Float64MultiArray`。状态数组布局为 `[qpos(nq), qvel(nv), activation(na), actuator_force(nu)]`；命令数组布局为 `[q_ref(nu), dq_ref(nu), actuator_control(nu)]`，当前后端将最后一段写入 MuJoCo `ctrl`。

## 依赖和构建

需要 ROS 2 Jazzy 和本机安装的 MuJoCo。MuJoCo 动力学库是外部依赖，用户需要自行下载或编译，并通过 `MUJOCO_ROOT` 指向安装目录。该目录需要包含 `include/mujoco/mujoco.h`、`lib/libmujoco.so` 和 `simulate/simulate.cc`。

项目中的 `assets` 目录包含仿真所需的机器人和倒立摆 XML 文件，构建时会安装到 ROS 2 包的 `share/mujoco_sim/assets`。使用项目内置模型时不需要再准备外部模型目录。

例如 MuJoCo 安装在 `~/.mujoco/mujoco-3.17.0`：

```bash
export MUJOCO_ROOT=$HOME/.mujoco/mujoco-3.17.0
source /opt/ros/jazzy/setup.zsh
cd ~/robotics/ros2_ws
colcon build --packages-select mujoco_sim
source install/setup.zsh
```

也可以在构建时直接指定路径：

```bash
colcon build --packages-select mujoco_sim \
  --cmake-args -DMUJOCO_ROOT=$HOME/.mujoco/mujoco-3.17.0
```

## 仿真模型

`sim.launch.py` 会根据 `robot` 名称，在包内 `assets` 目录下解析相对 XML 路径：

```bash
ros2 launch mujoco_sim sim.launch.py robot:=g1
ros2 launch mujoco_sim sim.launch.py robot:=g1_single_leg
ros2 launch mujoco_sim sim.launch.py robot:=inverted_pendulum
ros2 launch mujoco_sim sim.launch.py robot:=inverted_double_pendulum
ros2 launch mujoco_sim sim.launch.py robot:=inverted_triple_pendulum
```

也可以传入自定义 XML 的绝对路径，或指定另一个 assets 目录：

```bash
ros2 launch mujoco_sim sim.launch.py robot:=/path/to/custom_model.xml
ros2 launch mujoco_sim sim.launch.py \
  assets_dir:=/path/to/assets robot:=my_model.xml
```

## LQR 控制

`lqr_controller` 是 ROS 2 节点，仅通过 ROS 2 参数 `robot` 和 `model_path` 配置。它加载 XML，在模型默认位姿及零控制附近用 MuJoCo 数值线性化得到 A/B，然后求解 LQR 增益 K。状态权重和控制权重可用正数数组参数 `q_weights`（长度 `2*nv+na`）和 `r_weights`（长度 `nu`）覆盖默认值。

推荐同时启动仿真和控制节点：

```bash
ros2 launch mujoco_sim lqr_control.launch.py robot:=inverted_pendulum
ros2 launch mujoco_sim lqr_control.launch.py robot:=inverted_double_pendulum
ros2 launch mujoco_sim lqr_control.launch.py robot:=inverted_triple_pendulum
```

单独启动控制节点时，`model_path` 必须是 ROS 2 参数：

```bash
ros2 run mujoco_sim lqr_controller --ros-args \
  -p robot:=inverted_pendulum \
  -p model_path:=$(ros2 pkg prefix mujoco_sim)/share/mujoco_sim/assets/pendulums/inverted_pendulum.xml
```

控制器从 `/<robot>/state` 读取状态，向 `/<robot>/command` 发布控制。对于自由关节，位置误差通过 MuJoCo 切空间差值计算。任意自定义模型需要合适的平衡点和 LQR 权重，不能保证仅更换 XML 就能稳定控制。
