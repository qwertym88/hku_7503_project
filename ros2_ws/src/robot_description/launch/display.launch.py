from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros


def generate_launch_description():
    model = PathJoinSubstitution([
        FindPackageShare("robot_description"), "urdf", "dase7503_robot.urdf"
    ])
    robot_description_content = Command(["cat ", model])
    description = {'robot_description': launch_ros.descriptions.ParameterValue(
        robot_description_content, value_type=str)}
    use_gui = LaunchConfiguration("use_gui")
    rviz_config = PathJoinSubstitution([
        FindPackageShare("robot_description"), "config", "display.rviz"
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_gui", default_value="true",
            description="Show the joint_state_publisher GUI (set false on headless systems)"),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             name="joint_state_publisher", parameters=[description],
             condition=IfCondition(use_gui)),
        Node(package="joint_state_publisher", executable="joint_state_publisher",
             name="joint_state_publisher", parameters=[description],
             condition=UnlessCondition(use_gui)),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", parameters=[description]),
        Node(package="rviz2", executable="rviz2",
             name="rviz2", output="screen",
             arguments=["-d", rviz_config]),
    ])
