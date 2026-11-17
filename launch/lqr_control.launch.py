from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start_control(context):
    robot = LaunchConfiguration("robot").perform(context)
    assets_dir = Path(LaunchConfiguration("assets_dir").perform(context)).expanduser()
    models = {
        "g1": Path("robots/unitree_g1/xmls/g1.xml"),
        "g1_single_leg": Path("robots/unitree_g1/xmls/g1_single_leg.xml"),
        "inverted_pendulum": Path("pendulums/inverted_pendulum.xml"),
        "inverted_double_pendulum": Path("pendulums/inverted_double_pendulum.xml"),
        "inverted_triple_pendulum": Path("pendulums/inverted_triple_pendulum.xml"),
    }
    model_path = assets_dir / models[robot] if robot in models else Path(robot).expanduser()
    if not model_path.is_absolute():
        model_path = assets_dir / model_path
    topic_robot = robot if robot in models else model_path.stem
    sim_launch = Path(get_package_share_directory("robotics_lab")) / "launch/sim.launch.py"
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim_launch)),
            launch_arguments={"robot": robot, "assets_dir": str(assets_dir)}.items(),
        ),
        Node(
            package="robotics_lab",
            executable="lqr_controller",
            name="lqr_controller",
            output="screen",
            parameters=[{"robot": topic_robot, "model_path": str(model_path)}],
        ),
    ]


def generate_launch_description():
    default_assets = str(
        Path(get_package_share_directory("robotics_lab")) / "assets"
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", default_value="inverted_pendulum",
            description="Model name or path, as in sim.launch.py",
        ),
        DeclareLaunchArgument(
            "assets_dir", default_value=default_assets,
            description="Package assets directory; override for custom assets",
        ),
        OpaqueFunction(function=_start_control),
    ])
