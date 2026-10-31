> SLAM Toolbox is now enabled by default (`/map`, `map -> odom`).
> Select controls with `./scripts/sim.sh up --teleop joystick|keyboard|none`.
> Build Docker with `./scripts/sim.sh build --docker`, then launch with
> `./scripts/sim.sh up --docker --gpu nvidia|none --teleop joystick|keyboard`.
> See README.md for prerequisites and logs.

# Gazebo house sandbox

The simulation uses ROS 2 Jazzy and Gazebo Harmonic. It runs locally, independently
of the legacy Docker/management files in this blueprint. The default environment is the original
[AWS RoboMaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world/tree/ff9631ca6d1db9c1ba656498151464b5ab74aafe),
including its 87 model instances, furniture, lighting, collision meshes and textures.
The former procedural house is no longer used.

## Start

Prerequisites on Ubuntu 24.04: ROS Jazzy, `ros-jazzy-ros-gz`,
`ros-jazzy-robot-state-publisher`, `ros-jazzy-xacro`, `ros-jazzy-joy`,
`ros-jazzy-teleop-twist-joy`, `ros-jazzy-rviz2`, `python3-colcon-common-extensions`,
`python3-yaml`, the C++ build toolchain and Eigen3. This machine already has the
required runtime. RGB-D and GPU lidar require rendering even with the GUI disabled.

```bash
bash scripts/sim.sh assets  # automatic on build/up too; first download is about 82 MiB
bash scripts/sim.sh check
bash scripts/sim.sh build
bash scripts/sim.sh up
# Fully headless sensor test, without GUI or joystick:
bash scripts/sim.sh up gui:=false rviz:=false joystick:=false
```

Stop with Ctrl-C. Build products and logs stay in `.simulation/`.
AWS assets are cached at `gazebo.aws_house_dir` (default `.simulation/assets/aws_small_house`).
The initial download requires network access; subsequent starts work offline.
`sim.sh assets` verifies the downloaded archive SHA-256 and pins revision
`ff9631ca6d1db9c1ba656498151464b5ab74aafe`. The upstream license is retained in
that directory. No AWS account, cloud service, Gazebo Classic installation or
nested Git checkout is needed.

The original Classic world is adapted at launch by adding Harmonic's systems and
configured physics. Static model includes are expanded and their inertial blocks
omitted because some upstream inertias fail Harmonic validation; static meshes,
collisions, lights and placement are retained. The downloader also corrects the
upstream portrait texture paths (one excess `../`). Mesh geometry and photo contents
are unchanged. The original world file is kept in the asset cache.
The default robot spawn is `[-3.5, -0.5, 0.03, 0.0]`, on the open indoor passage.
An explicit `gazebo.world_file` still selects a custom world and skips the AWS download.
The script selects system Python to match ROS binary extensions.
For machines without X11, set `gazebo.headless_rendering: true` in the YAML;
an EGL-capable rendering driver is still required. `LIBGL_ALWAYS_SOFTWARE=1`
can select Mesa software rendering, with reduced throughput.

The default configuration is
`config/simulation.yaml`. Override it with
`PIROBOT_SIM_CONFIG=/absolute/path/simulation.yaml`. Relative paths in a YAML
are resolved against that YAML, so update them when moving the configuration.
`PIROBOT_SIM_WS` changes the build/install/log directory.

The source and tuning YAML for sensor fusion point to this project's
`sandbox/Ros2_sensorFusion` module. The script compiles that
source into this project's build directory; it never changes that module.
Change `components.sensor_fusion.source` and `parameter_files` to select another
checkout. An already-installed package can omit `source`.

Direct ROS launch, after sourcing Jazzy and `.simulation/install/setup.bash`:

```bash
export ROS_DOMAIN_ID=42
export GZ_PARTITION=pirobot_sim_42
ros2 launch simulation_bringup simulation.launch.py \
  config:="$PWD/config/simulation.yaml"
```

The script defaults to domain 42 and a matching Gazebo partition. Other sandbox
modules must use the same domain and `use_sim_time: true`. Do not start physical
drivers or another clock/robot-state/odometry publisher in that domain.

## Joystick and RViz2

`./scripts/sim.sh up` starts Gazebo, the SDL game controller driver, Twist conversion,
the unchanged sandbox teleoperation nodes, sensor fusion and RViz2 together.
The connected Google Stadia Controller is supported by SDL's standard mapping:

- Hold **L1 / left shoulder** (button 9) to enable velocity commands.
- Left stick vertical: forward/reverse, up to 0.20 m/s.
- Right stick horizontal: rotation.
- A (button 0): toggle arming; simulation starts armed.
- Start (button 6): clear motor references.

Mappings, deadzone, device ID, repeat rate and speed are in the `joystick*`
components of `simulation.yaml`. The 30 Hz repeat keeps a held command alive;
releasing L1 sends zero, and the simulated motor watchdog handles lost input.
The pipeline is `/joy` -> `/cmd_vel/raw` (`TwistStamped`) -> sandbox velocity
mixing -> `/cmd_vel/joystick_stamped` -> timestamp-checked conversion ->
`/cmd_vel` (`Twist`). The sandbox packages are built from
`sandbox/Ros2_teleoperation/src` without modifying them.

RViz loads `config/simulation.rviz`, with fixed frame
`odom`, the robot, laser scan, EKF odometry, RGB-D cloud and RGB image.
It does not require SLAM or a `map` frame. Odometry covariance visuals are
disabled: the unchanged planar EKF assigns large uncertainty to unestimated
Z/roll/pitch, which otherwise fills RViz with yellow ellipsoids. Update display topics too if changing
sensor topic names. `{config_dir}` in component arguments resolves to the YAML's
directory, so the RViz preset is selected without absolute machine paths.

`rviz:=false` disables RViz alone; `joystick:=false` disables the complete joystick
group. `gui:=false` only disables the Gazebo GUI. These overrides do not change
persistent YAML defaults. The standalone pipeline test is `./scripts/sim.sh test-teleop`;
it publishes synthetic Joy messages on isolated `/teleop_test/*` topics and
checks forward motion, rotation and deadman release without commanding the robot.
The controller has also been opened on this machine and `/joy` reception checked;
physical stick/button operation still depends on the user moving the controller.

## Configuration and components

`sensors` exposes lidar, IMU, encoder velocity and RGB-D rates independently.
RGB and registered depth share one camera, resolution, intrinsics and timestamp.
Rates are measured in simulation time; wall-clock rates drop if rendering cannot
keep up. Physics timestep limits sampling and encoder interval resolution.

`gazebo` exposes GUI, headless rendering, timestep, real-time factor, random seed,
spawn pose and an optional custom SDF world. Custom worlds must load Physics,
UserCommands, Sensors (Ogre2), Imu and SceneBroadcaster systems, as the house does.

Add arbitrary ROS nodes under `components`; no shell or launch edits are needed:

```yaml
components:
  my_component:
    enabled: true
    package: my_package
    executable: my_node
    source: /path/to/package_or_src  # optional build input
    name: my_node
    namespace: ''
    parameter_files: [/path/to/parameters.yaml]
    parameters: {some_setting: 1.0}
    remappings: {input_scan: /scan, output_command: /cmd_vel}
    arguments: []
```

Run `build` after adding source packages. Enabled components receive
`use_sim_time: true` after their parameter files. Disabled components are neither
built nor launched. Dependencies of additional packages must already be installed.
Launch lifecycle managers as additional components when a stack requires them;
this registry does not automatically activate lifecycle nodes.

## Interfaces and TF

| Interface | Type / meaning |
| --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist`, desired body velocity |
| `/cmd_vel/accepted` | `Twist`, accepted command after arming, timeout and saturation |
| `/enc/twist_meas` | `TwistStamped`, measured wheel-derived body velocity |
| `/imu` | `sensor_msgs/Imu`, noisy body-aligned gyro and acceleration |
| `/scan` | `LaserScan`, 720 samples, frame `laser_scan` |
| `/camera/realsense/color/image_raw` | RGB image |
| `/camera/realsense/aligned_depth_to_color/image_raw` | registered `32FC1` depth, metres |
| `/camera/realsense/color/camera_info` | shared RGB/depth intrinsics |
| `/camera/realsense/depth/color/points` | RGB-D point cloud |
| `/joint_states` | measured wheel angles and angular velocities |
| `/odom` | unchanged EKF output |
| `/sim/ground_truth` | diagnostic physical pose, never fed to the EKF |
| `/control/command` | `robot_interfaces/ControlCommand`, ARM_TOGGLE and RESET |

The public sensor/command names are configurable under `topics`. Defaults match
the current sensor-fusion and SLAM inputs. Compressed `/burger/...` image topics are not generated. Use the raw RGB-D topics directly
or explicit remapping in consumers. USB protocol and compressed transport are not
emulated. RESET clears motor references; restart bringup to reset the whole scene
and EKF together. Gazebo time rewind while keeping the existing EKF alive is not a
supported reset workflow.

Only sensor fusion publishes `odom -> base_link`. Robot state publisher owns the
links beneath `base_link`; an optional SLAM component owns `map -> odom`.
Ground truth has no ROS TF bridge. The blueprint's inverted LD06 `laser_link`
remains part of the physical model, while `laser_scan` is upright at the same
sensor origin. Scan consumers must use the message frame and TF, rather than
assuming `laser_link`. RGB/depth images and camera info use `camera_color_optical_frame` with the
standard optical rotation. Gazebo Harmonic point-cloud XYZ uses X forward, Y left,
Z up: a dedicated bridge overrides only the cloud header to `realsense_link`.
Applying the optical rotation to these coordinates would rotate the scene incorrectly.
No second camera TF publisher is needed.

The EKF retains its original tuning. Bringup changes only its input/output topic,
frame and simulated-clock parameters. Encoder velocity comes from quantized
physical joint angle differences, not from command or ideal odometry. IMU and
encoder stamps remain Gazebo measurement stamps. A bounded reorder queue delays
delivery by `sensors.reorder_window` (30 ms by default), then releases individual
measurements to reduce out-of-sequence rejection in the existing EKF.
DDS does not guarantee ordering across topics: under heavy overload, inspect the
EKF log for discarded measurements. The queue drops samples arriving older than
its last released stamp. The IMU yaw-rate covariance reflects configured noise;
orientation is marked unavailable, as in the physical driver.

## Non-ideal model and its limits

Wheel controllers apply bounded **torque** through Gazebo's force-mode PID.
The motor adapter adds first-order response, acceleration limits, deadband,
left/right gain mismatch, command saturation and timeout. Contact friction,
wheel joint damping, IMU Gaussian noise/static bias/dynamic bias, lidar noise
and encoder quantization/noise are configurable. The blueprint caster is lifted
9 mm in the generated simulation model to remove initial floor penetration.
The original Xacro files are unchanged.

These values are initial engineering estimates, not measured motor or BNO055
calibration. RealSense is represented by Gazebo's generic registered RGB-D camera:
it does not reproduce stereo matching failures, IR interference, rolling shutter,
depth-dependent D435 errors or the device firmware. Camera noise is the simulator's
camera noise model, not a calibrated RealSense depth model. The LD06 model does not
simulate serial packet timing or motion distortion within a revolution. The AWS house supplies textured visual geometry for camera-based experiments.

Implementation references:
[Gazebo force-mode JointController](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointController.html),
[diagnostic OdometryPublisher](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html),
[ROS–Gazebo bridge](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md).

## Verification

```bash
bash scripts/sim.sh test     # contracts, force mode, sensor frames/rates, AWS scene preservation and texture paths
bash scripts/sim.sh smoke    # starts/stops a private run in the configured domain
bash scripts/sim.sh logs     # latest smoke test launch log
```

Run smoke with no other simulator in that domain/partition. It verifies all
required streams, frame IDs, depth encoding, forward motion, encoder response,
physical displacement against EKF displacement, timeout stopping and positive IMU
yaw during a positive rotation command. It prints observed sensor rates and saves
the launch log in `.simulation/smoke.log` and an RGB view in `.simulation/smoke_camera.ppm`. It uses the source checkout's EKF.
SLAM/Nav2 and hardware calibration require their own scenario tests.

With simulation running, `./scripts/sim.sh check-camera` checks synchronized point-cloud/depth/RGB
samples, the optical TF convention, reprojection residuals against configured sensor noise,
and packed RGB values against the image. This catches camera-frame and color regressions.
