from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_type = LaunchConfiguration("robot_type")
    robot_controller = LaunchConfiguration("robot_controller")
    start_rviz = LaunchConfiguration("start_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_sn = LaunchConfiguration("robot_sn")
    set_prefix = SetLaunchConfiguration(
        name="prefix",
        value=PythonExpression(["'", robot_sn, "_' if '", robot_sn, "' else ''"]),
    )
    controllers_file = PathJoinSubstitution([FindPackageShare("flexiv_bringup"), "config", "rizon_gazebo_controllers.yaml"])
    xacro_file = PathJoinSubstitution([FindPackageShare("flexiv_hardware"), "urdf", "flexiv.urdf.xacro"])
    robot_description = {"robot_description": ParameterValue(Command([
        FindExecutable(name="xacro"), " ", xacro_file,
        " robot_type:=", robot_type, " robot_sn:=", robot_sn, " ros2_control:=true",
        " sim_gazebo:=true", " gazebo_controllers_file:=", controllers_file,
        " load_mounted_ft_sensor:=false",
    ]), value_type=str), "use_sim_time": use_sim_time}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[robot_description], output="both")
    spawn = Node(package="ros_gz_sim", executable="create", arguments=["-topic", "robot_description", "-name", "rizon"], output="screen")
    jsb = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"], output="screen")
    controller = Node(package="controller_manager", executable="spawner", arguments=[robot_controller], output="screen")
    rviz = Node(package="rviz2", executable="rviz2", arguments=["-d", PathJoinSubstitution([FindPackageShare("flexiv_description"), "rviz", "view_flexiv.rviz"])], parameters=[{"use_sim_time": use_sim_time}], condition=IfCondition(start_rviz))
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )
    return LaunchDescription([
        DeclareLaunchArgument("robot_type", default_value="Rizon4"),
        DeclareLaunchArgument("robot_sn", default_value="Rizon4-063116"),
        DeclareLaunchArgument("robot_controller", default_value="cartesian_compliance_controller"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        set_prefix, gazebo, rsp, TimerAction(period=2.0, actions=[spawn]),
        clock_bridge,
        TimerAction(period=4.0, actions=[jsb]), TimerAction(period=5.0, actions=[controller, rviz]),
    ])
