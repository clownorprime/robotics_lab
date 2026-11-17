from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start_sim(context):
    assets_dir = Path(LaunchConfiguration("assets_dir").perform(context)).expanduser()
    robot = LaunchConfiguration("robot").perform(context)

    models = {
        "g1": Path("robots/unitree_g1/xmls/g1.xml"),
        "g1_single_leg": Path("robots/unitree_g1/xmls/g1_single_leg.xml"),
        "inverted_pendulum": Path("pendulums/inverted_pendulum.xml"),
        "inverted_double_pendulum": Path("pendulums/inverted_double_pendulum.xml"),
        "inverted_triple_pendulum": Path("pendulums/inverted_triple_pendulum.xml"),
    }
    scene = assets_dir / models[robot] if robot in models else Path(robot).expanduser()
    topic_robot = robot if robot in models else scene.stem
    if not scene.is_absolute():
        scene = assets_dir / scene

    return [
        Node(
            package="robotics_lab",
            executable="mujoco_sim",
            name="mujoco_sim",
            output="screen",
            arguments=[
                "-r", robot, "-s", str(scene),
                "--command-topic", f"/{topic_robot}/command",
                "--state-topic", f"/{topic_robot}/state",
            ],
        )
    ]


def generate_launch_description():
    default_assets = str(Path(get_package_share_directory("robotics_lab")) / "assets")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "assets_dir",
                default_value=default_assets,
                description="Directory containing MuJoCo robot and pendulum assets",
            ),
            DeclareLaunchArgument(
                "robot",
                default_value="g1",
                description=(
                    "Model name: g1, g1_single_leg, inverted_pendulum, "
                    "inverted_double_pendulum, inverted_triple_pendulum, or an XML/MJB path"
                ),
            ),
            OpaqueFunction(function=_start_sim),
        ]
    )
