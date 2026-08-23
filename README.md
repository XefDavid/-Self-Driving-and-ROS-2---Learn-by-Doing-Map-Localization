# Self-Driving and ROS 2 — Learn by Doing: Map & Localization

A ROS 2 workspace for the **Bumperbot** differential-drive robot, covering the full pipeline from URDF description and Gazebo simulation to odometry, sensor fusion, occupancy-grid mapping (SLAM Toolbox), and localization (AMCL / global localization).

This project follows Antonio Brandi's *"Self-Driving and ROS 2 — Learn by Doing"* course, extended with my own commits on top of the base `bumperbot` starter project (odometry motion models, LiDAR simulation, safety monitoring, action server/client, mapping, and localization sections).

## Package overview

| Package | Purpose |
|---|---|
| `bumperbot_description` | URDF/xacro robot model, meshes, Gazebo worlds, RViz configs |
| `bumperbot_bringup` | Top-level launch files to bring up the simulated or real robot |
| `bumperbot_controller` | Differential-drive controller, odometry, joystick teleop |
| `bumperbot_firmware` | Motor driver / hardware interface for the real robot |
| `bumperbot_localization` | EKF sensor fusion, AMCL, global/local localization launch files |
| `bumperbot_mapping` | SLAM Toolbox configuration and mapping launch files |
| `bumperbot_msgs` | Custom service and action definitions |
| `bumperbot_utils` | Shared utility nodes |
| `bumperbot_cpp_examples` / `bumperbot_py_examples` | Standalone C++/Python examples used throughout the course (topics, services, actions, TF) |

## Prerequisites

- ROS 2 **Humble** or **Iron** (some dependencies switch based on `$ROS_DISTRO`, see each package's `package.xml`)
- Gazebo (via `ros_gz_sim` / `ros_gz_bridge`)
- `slam_toolbox`, `nav2_amcl` (or the `navigation2` stack), `robot_localization`

## Build

```bash
cd <workspace_root>
colcon build --symlink-install
source install/setup.bash
```

## Usage

**Simulate the robot in Gazebo:**
```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py
```

**Build a map (SLAM Toolbox):**
```bash
ros2 launch bumperbot_mapping slam.launch.py
```

**Localize in a known map (AMCL):**
```bash
ros2 launch bumperbot_localization global_localization.launch.py
# or, once localized:
ros2 launch bumperbot_localization local_localization.launch.py
```

**Bring up the real robot:**
```bash
ros2 launch bumperbot_bringup real_robot.launch.py
```

## Gazebo world assets (AWS RoboMaker)

The `small_house.world` and `small_warehouse.world` Gazebo worlds render furniture/prop models from [AWS RoboMaker](https://github.com/aws-robotics). Those assets are provided by Amazon under their own license and are **not vendored in this repository** — download them yourself into `src/bumperbot_description/models/`:

```bash
cd src/bumperbot_description
git clone --depth 1 https://github.com/aws-robotics/aws-robomaker-small-house-world.git /tmp/house
git clone --depth 1 https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git /tmp/warehouse
mkdir -p models photos
cp -r /tmp/house/models/* models/
cp -r /tmp/warehouse/models/* models/
cp /tmp/house/models/aws_robomaker_residential_*/materials/textures/*.jpg photos/ 2>/dev/null
```

`bumperbot_description/launch/gazebo.launch.py` sets `GZ_SIM_RESOURCE_PATH` to that `models/` directory automatically, so no further configuration is needed once the assets are in place. This step is only required for the two populated worlds — everything else (robot description, controllers, mapping, localization) works without them.

## Notes

- Base robot description, firmware interface, and controller packages originate from the course's `bumperbot` project by Antonio Brandi.

## License

This project is licensed under the Apache License 2.0 — see [LICENSE](LICENSE).
