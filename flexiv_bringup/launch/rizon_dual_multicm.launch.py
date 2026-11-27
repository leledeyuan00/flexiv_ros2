from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
import xacro


SIDES = ("left", "right")
SERIALS = {"left": "Rizon4-063078", "right": "Rizon4-063116"}
RIZON_TYPES = ["Rizon4", "Rizon4M", "Rizon4R", "Rizon4s", "Rizon10", "Rizon10s", "Rizon10R"]


def start_after_success(process, actions, failure_message):
    def on_exit(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            get_logger("rizon_dual_multicm").error(failure_message)
            return []
        return actions

    return RegisterEventHandler(OnProcessExit(target_action=process, on_exit=on_exit))


def arm_nodes(context, side, robot_description, prefix, serial, use_fake_hardware):
    def arg(name):
        return LaunchConfiguration(f"{name}_{side}").perform(context)

    manager = f"/{side}/controller_manager"
    controllers = ParameterFile(
        LaunchConfiguration(f"controllers_file_{side}"), allow_substs=True
    )
    controller_params_path = str(controllers.evaluate(context))
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=side,
        parameters=[controllers],
        output="both",
    )

    # Jazzy reads the URDF from the local robot_description topic. The root
    # state publisher below owns the public TF tree for both arms.
    description_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=side,
        parameters=[robot_description],
        remappings=[("/tf", "~/tf"), ("/tf_static", "~/tf_static")],
        output="both",
    )

    def spawner(*arguments):
        return Node(
            package="controller_manager",
            executable="spawner",
            namespace=side,
            arguments=[
                *arguments,
                "-c", manager,
                "--param-file", controller_params_path,
                "--controller-manager-timeout", "120",
            ],
            output="screen",
        )

    joint_states = spawner("joint_state_broadcaster")
    selected_controller = arg("robot_controller")
    motion_controller = spawner(selected_controller)
    after_joint_states = []
    handlers = []

    if not use_fake_hardware:
        after_joint_states.append(
            spawner("flexiv_robot_states_broadcaster", "gpio_controller")
        )

    if not use_fake_hardware and arg("load_gripper") == "true":
        gripper = GroupAction(
            [
                PushRosNamespace(side),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [
                                FindPackageShare("flexiv_gripper"),
                                "launch",
                                "flexiv_gripper.launch.py",
                            ]
                        )
                    ),
                    launch_arguments={
                        "robot_sn": serial,
                        "gripper_name": arg("gripper_name"),
                        "gripper_joint_names": f"[{prefix}finger_width_joint]",
                        "use_lite_rdk": "true",
                    }.items(),
                ),
            ]
        )
        ready = Node(
            package="flexiv_gripper",
            executable="wait_for_gripper_ready",
            name="wait_for_gripper_ready",
            namespace=side,
            parameters=[{"ready_topic": f"/{side}/flexiv_gripper_node/ready"}],
            output="screen",
        )
        after_joint_states.extend([gripper, ready])
        handlers.append(
            start_after_success(ready, [motion_controller], f"{side} gripper did not report ready")
        )
    else:
        after_joint_states.append(motion_controller)

    if selected_controller != "rizon_arm_controller":
        handlers.append(
            start_after_success(
                motion_controller,
                [spawner("rizon_arm_controller", "--inactive")],
                f"{manager} failed to activate {selected_controller}",
            )
        )
    handlers.append(
        start_after_success(
            joint_states,
            after_joint_states,
            f"{manager} failed to activate joint_state_broadcaster",
        )
    )
    return [description_node, control_node, *handlers, joint_states]


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    use_fake_hardware = arg("use_fake_hardware") == "true"
    serials = {side: arg(f"robot_sn_{side}") for side in SIDES}
    if not use_fake_hardware and (
        not all(serials.values()) or len(set(serials.values())) != len(SIDES)
    ):
        raise ValueError("Real hardware requires two distinct Rizon serial numbers")

    xacro_file = get_package_share_path("flexiv_hardware") / "urdf" / "flexiv.urdf.xacro"
    mappings = {
        "robot_type": "Rizon-Dual",
        "rdk_control_mode": arg("rdk_control_mode"),
        "rdk_realtime_mode": arg("rdk_realtime_mode"),
        "withhold_on_timeliness_failure": arg("withhold_on_timeliness_failure"),
        "use_fake_hardware": arg("use_fake_hardware"),
        "fake_sensor_commands": arg("fake_sensor_commands"),
    }
    if arg("paired_mounting_file"):
        mappings["paired_mounting_file"] = arg("paired_mounting_file")

    state_sources = []
    for side, serial in serials.items():
        robot_name = f"{side}_{serial}" if serial else side
        context.launch_configurations[f"robot_name_{side}"] = robot_name
        context.launch_configurations[f"prefix_{side}"] = f"{robot_name}_"
        context.launch_configurations[f"robot_topic_namespace_{side}"] = (
            robot_name.replace("-", "_")
        )
        mappings.update(
            {
                f"robot_sn_{side}": serial,
                f"arm_type_{side}": arg(f"arm_type_{side}"),
                f"load_gripper_{side}": arg(f"load_gripper_{side}"),
                f"load_mounted_ft_sensor_{side}": arg(f"load_mounted_ft_sensor_{side}"),
                f"kinematics_parameters_file_{side}": arg(f"kinematics_params_file_{side}"),
            }
        )
        state_sources.append(f"/{side}/joint_state_broadcaster/joint_states")
        if not use_fake_hardware and arg(f"load_gripper_{side}") == "true":
            state_sources.append(f"/{side}/flexiv_gripper_node/gripper_joint_states")

    def description(**control_args):
        urdf = xacro.process_file(str(xacro_file), mappings={**mappings, **control_args})
        return {"robot_description": urdf.toxml()}

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[description(ros2_control="false")],
            output="both",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[{"source_list": state_sources, "rate": 30}],
        ),
    ]
    for side in SIDES:
        nodes.extend(
            arm_nodes(
                context,
                side,
                description(ros2_control="true", rizon_dual_control_arm=side),
                context.launch_configurations[f"prefix_{side}"],
                serials[side],
                use_fake_hardware,
            )
        )

    if arg("start_rviz") == "true":
        rviz_config = get_package_share_path("flexiv_description") / "rviz" / "view_flexiv.rviz"
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(rviz_config)],
                output="log",
            )
        )
    return nodes


def generate_launch_description():
    def flag(name, default, description):
        return DeclareLaunchArgument(
            name, default_value=default, choices=["true", "false"], description=description
        )

    arguments = [
        DeclareLaunchArgument(
            "rdk_control_mode", default_value="joint_position",
            choices=["joint_position", "joint_impedance"],
            description="RDK joint mode for both arms.",
        ),
        flag("rdk_realtime_mode", "true", "Stream commands in RDK real-time mode."),
        flag(
            "withhold_on_timeliness_failure", "false", "Withhold motion after RDK timing failures."
        ),
        # DeclareLaunchArgument(
        #     "network_interface_whitelist", default_value="[192.168.2.10/24, 192.168.2.11/24]",
        #     description="Whitelist of allowed IP addresses.",
        # ),
        flag("use_fake_hardware", "false", "Use mock hardware for both arms."),
        flag("fake_sensor_commands", "false", "Expose mock sensor command interfaces."),
        flag("start_rviz", "false", "Start RViz with the combined robot description."),
        DeclareLaunchArgument(
            "paired_mounting_file", default_value="",
            description="Override the dual-arm mounting YAML from flexiv_description.",
        ),
    ]
    for side, default_type in zip(SIDES, ("Rizon4", "Rizon4")):
        arguments.extend(
            [
                DeclareLaunchArgument(
                    f"robot_sn_{side}", default_value=SERIALS[side], description=f"Serial number of the {side} arm."
                ),
                DeclareLaunchArgument(
                    f"arm_type_{side}", default_value=default_type, choices=RIZON_TYPES,
                    description=f"Rizon model of the {side} arm.",
                ),
                DeclareLaunchArgument(
                    f"robot_controller_{side}", default_value="cartesian_compliance_controller",
                    description=f"Controller to activate on the {side} arm.",
                ),
                DeclareLaunchArgument(
                    f"controllers_file_{side}",
                    default_value=PathJoinSubstitution(
                        [
                            FindPackageShare("flexiv_bringup"),
                            "config",
                            f"rizon_controllers_{side}.yaml",
                        ]
                    ),
                    description=f"Path to the {side} arm's controller YAML.",
                ),
                DeclareLaunchArgument(
                    f"kinematics_params_file_{side}", default_value="",
                    description=f"Path to calibration YAML for the {side} arm.",
                ),
                flag(
                    f"load_mounted_ft_sensor_{side}", "true", f"Model the {side} wrist F/T sensor."
                ),
                flag(f"load_gripper_{side}", "true", f"Load the {side} Flexiv Grav gripper."),
                DeclareLaunchArgument(f"gripper_name_{side}", default_value="Flexiv-GN01"),
            ]
        )
    return LaunchDescription([*arguments, OpaqueFunction(function=launch_setup)])
