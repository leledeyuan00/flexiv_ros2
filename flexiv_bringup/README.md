# flexiv_bringup

This package contains launch files: the main driver launcher, the MoveIt launch file and demo examples:

- `rizon.launch.py` - the main launcher: starts *ros2_control* node including hardware interface, runs joint states, Flexiv robot states broadcaster, and a controller, and visualizes the current robot pose in RViZ. The default controller is `rizon_arm_controller`, a joint trajectory controller.
- `rizon_moveit.launch.py` - runs MoveIt together with the main driver. The controller for robot joints started in this launch file is *rizon_arm_controller*. With `start_servo:=true` it also starts MoveIt Servo.
- `test_joint_trajectory_controller.launch.py` - sends joint trajectory goals to the *rizon_arm_controller*.
- `test_cartesian_motion_force_controller.launch.py` - runs the Cartesian motion-force example against the *cartesian_motion_force_controller*, started with `rizon.launch.py robot_controller:=cartesian_motion_force_controller`.

There are also launch files for other robot setups:

- `aico1.launch.py` - the main launcher for Flexiv AICO1 robot.
- `aico1_moveit.launch.py` - runs MoveIt together with the main driver for Flexiv AICO1 robot.
- `aico2.launch.py` - the main launcher for Flexiv AICO2 robot.
- `aico2_moveit.launch.py` - runs MoveIt together with the main driver for Flexiv AICO2 robot.
- `rizon_dual.launch.py` - the main launcher for Flexiv Rizon dual robot setup.
- `rizon_dual_multicm.launch.py` - brings up two Rizon arms with separate controller managers and a shared visualization.
- `rizon_dual_moveit.launch.py` - runs MoveIt together with the main driver for Flexiv Rizon dual robot setup.

**NOTE**: The example launch files run the demo nodes from the `flexiv_test_nodes` package, with the parameters defined in `/config`.

## Rizon dual arm with separate controller managers

```bash
ros2 launch flexiv_bringup rizon_dual_multicm.launch.py \
  robot_sn_left:=Rizon4-000001 robot_sn_right:=Rizon4R-000002 \
  use_fake_hardware:=true start_rviz:=true
```

For real hardware, supply both robot serial numbers and models, and set
`use_fake_hardware:=false`. Each manager runs its own read, update, and write
loop using the single-arm Flexiv hardware interface. The managers are
`/left/controller_manager` and `/right/controller_manager`. Their rates default
to 1000 Hz and can be configured independently in `rizon_controllers_left.yaml`
and `rizon_controllers_right.yaml`, or through YAML overrides passed via
`controllers_file_left` and `controllers_file_right`. Actual loop timing
depends on the host and the robot connection.

Both arms default to `rizon_arm_controller`. The trajectory action names are
`/left/rizon_arm_controller/follow_joint_trajectory` and
`/right/rizon_arm_controller/follow_joint_trajectory`. Set
`robot_controller_left` or `robot_controller_right` to
`cartesian_motion_force_controller` or `cartesian_compliance_controller` to
activate a Cartesian controller for that arm. The trajectory controller is
then configured but starts inactive.

The joint state broadcasters publish under
`/<side>/joint_state_broadcaster/joint_states`. The combined `/joint_states`
and TF tree use the existing Rizon-Dual mounting geometry and
`<side>_<serial>_` joint and frame prefixes. Use `paired_mounting_file` to
supply a custom mount.

```bash
ros2 control list_controllers -c /left/controller_manager
ros2 control list_controllers -c /right/controller_manager
```
