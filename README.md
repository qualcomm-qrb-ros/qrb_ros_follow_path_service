<div align="center">
  <h1>QRB ROS Follow Path Service</h1>
  <p align="center">
   <img src="https://s7d1.scene7.com/is/image/dmqualcommprod/rb3gen2-dev-kits-hero-7" alt="Qualcomm QRB ROS" title="Qualcomm QRB ROS" />
      
  </p>
  <p>ROS2 Package for Follow Path Service on Qualcomm Robotics Platform</p>
  
  <a href="https://ubuntu.com/download/qualcomm-iot" target="_blank"><img src="https://img.shields.io/badge/Qualcomm%20Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white" alt="Qualcomm Ubuntu"></a>
  <a href="https://docs.ros.org/en/jazzy/" target="_blank"><img src="https://img.shields.io/badge/ROS%20Jazzy-1c428a?style=for-the-badge&logo=ros&logoColor=white" alt="Jazzy"></a>
  
</div>

---

## 👋 Overview

[QRB ROS Follow_Path Service](https://github.com/qualcomm-qrb-ros/qrb_ros_follow_path_service) is a ROS2 package suite designed for path following navigation on Qualcomm robotics platform. Key features include:

- Waypoint and virtual path management on the map.
- Path following navigation to target waypoints.
- Obstacle avoidance during path following.

<div align="center">
  <img src="/docs/assets/architecture.png" alt="architecture">
</div>

<br>

[`qrb_ros_follow_path`](https://github.com/qualcomm-qrb-ros/qrb_ros_follow_path_service/tree/main/qrb_ros_follow_path): ROS2 package implementing action/service clients & servers, publishers and subscribers for waypoints, virtual paths, follow path navigation, position and LiDAR data.

[`qrb_follow_path_manager`](https://github.com/qualcomm-qrb-ros/qrb_ros_follow_path_service/tree/main/qrb_follow_path_manager): C++ library providing APIs for waypoints & virtual path management and path following navigation.
- Manager: Library interface.
- Virtual Path Manager: Manages waypoints and virtual paths, computes follow path.
- Follow Path Planner: Sends linear speed & angular speed to Robot Base Controller and check for obstacles on the follow path and support dynamic obstacle avoidance.
- Obstacle Detector: Uses laser scan and grid map to detect dynamic obstacles.
- PID Controller: Computes speed and checks if the target pose is reached.

[`Robot Base & Robot Base Controller](https://github.com/qualcomm-qrb-ros/qrb_ros_robot_base): ROS2 package for AMR base control.

`2D LiDAR SLAM ROS & 2D LiDAR SLAM`: Provides mapping and localization.

`2D LiDAR ROS`: Provides LiDAR data.

## 🔎 Table of contents
  * [APIs](#-apis)
     * [`qrb_ros_follow_path` APIs](#-qrb_ros_follow_path_service-apis)
     * [`qrb_follow_path_manager` APIs](#-qrb_follow_path_manager-apis)
  * [Supported targets](#-supported-targets)
  * [Installation](#-installation)
  * [Build from source](#-build-from-source)
  * [Usage](#-usage)
     * [Starting the follow_path service node](#start-the-follow-path-service-node)
  * [Contributing](#-contributing)
  * [Contributors](#%EF%B8%8F-contributors)
  * [FAQs](#-faqs)
  * [License](#-license)

## ⚓ APIs

### 🔹 `qrb_ros_follow_path_service` APIs

#### ROS interfaces

<table>
  <tr>
    <th>Interface</th>
    <th>Name</th>
    <th>Type</th>
    <td>Description</td>
  </tr>
  <tr>
    <td>Subscriber</td>
    <td>developer_mode</td>
    <td>std_msgs::msg::Int16</td>
    <td>Receives debugging information</td>
  </tr>
  <tr>
    <td>Subscriber</td>
    <td>robot_base_exception</td>
    <td>qrb_ros_robot_base_msgs::msg::Exception</td>
    <td>Receives exception info</td>
  </tr>
  <tr>
    <td>Service Server</td>
    <td>virtual_path</td>
    <td>qrb_ros_navigation_msgs::srv::VirtualPath</td>
    <td>Manages waypoints or virtual path</td>
  </tr>
  <tr>
    <td>Service Server</td>
    <td>compute_follow_path</td>
    <td>qrb_ros_navigation_msgs::srv::ComputeFollowPath</td>
    <td>Computes follow path including the passing through waypoints</td>
  </tr>
  <tr>
    <td>Service Server</td>
    <td>follow_path_sub_cmd</td>
    <td>qrb_ros_amr_msgs::srv::SubCmd</td>
    <td>Cancel, pause or resume follow path</td>
  </tr>
  <tr>
    <td>Action Server</td>
    <td>wfollowpath</td>
    <td>qrb_ros_navigation_msgs::action::FollowPath</td>
    <td>Request a follow path navigation</td>
  </tr>
</table>

#### ROS message parameters

##### developer_mode
<table>
  <tr>
    <th>Name</th>
    <th>Type</th>
    <th>Description</td>
    <th>Default Value</td>
  </tr>
  <tr>
    <td>data</td>
    <td>int16</td>
    <td>The debugging ID</td>
    <td>-</td>
  </tr>
</table>

> [!Note]
> Other modules can send the debugging information to simulate the AMR event when AMR is on developer mode.

##### robot_base_exception
<table>
  <tr>
    <th>Name</th>
    <th>Type</th>
    <th>Description</td>
    <th>Default Value</td>
  </tr>
  <tr>
    <td>type</td>
    <td>uint8</td>
    <td>Exception type</td>
    <td>-</td>
  </tr>
  <tr>
    <td>event</td>
    <td>uint8</td>
    <td>Enter/exit exception</td>
    <td>-</td>
  </tr>
  <tr>
    <td>trigger_sensor</td>
    <td>uint8</td>
    <td>Who triggers this exception</td>
    <td>-</td>
  </tr>
</table>

> [!Note]
> Receives the exception information from Robot base to stop the current follow path.

##### compute_follow_path
<table>
  <tr>
    <th>Name</th>
    <th>Type</th>
    <th>Description</td>
    <th>Default Value</td>
  </tr>
  <tr>
    <td>goal</td>
    <td>uint32</td>
    <td>Target waypoint ID.</td>
    <td>-</td>
  </tr>
  <tr>
    <td>use_start</td>
    <td>bool</td>
    <td>If false, use current base position as start if nearby waypoint.</td>
    <td>-</td>
  </tr>
  <tr>
    <td>passing_waypoint_ids</td>
    <td>uint32[]</td>
    <td>Path must include these waypoints.</td>
    <td>-</td>
  </tr>
  <tr>
    <td>result</td>
    <td>bool</td>
    <td>Result of computing follow path.</td>
    <td>-</td>
  </tr>
  <tr>
    <td>error_code</td>
    <td>uint16</td>
    <td>Error code if computation fails.</td>
    <td>-</td>
  </tr>
  <tr>
    <td>waypoint_id_list</td>
    <td>uint32[]</td>
    <td>All waypoint IDs in the path.</td>
    <td>-</td>
  </tr>
  <tr>
    <td>path</td>
    <td>nav_msgs/Path</td>
    <td>All points of the path.</td>
    <td>-</td>
  </tr>
</table>

> [!Note]
> Other modules can retrieve the path using this message.

##### follow_path_sub_cmd
<table>
  <tr>
    <th>Name</th>
    <th>Type</th>
    <th>Description</td>
    <th>Default Value</td>
  </tr>
  <tr>
    <td>subcommand</td>
    <td>uint8</td>
    <td>Cancel, pause, resume</td>
    <td>-</td>
  </tr>
  <tr>
    <td>result</td>
    <td>bool</td>
    <td>Result of sub-command</td>
    <td>-</td>
  </tr>
</table>

> [!Note]
> Other modules can cancel, pause, or resume the current follow path navigation using this message.

##### wfollowpath
<table>
  <tr>
    <th>Name</th>
    <th>Type</th>
    <th>Description</td>
    <th>Default Value</td>
  </tr>
  <tr>
    <td>goal</td>
    <td>uint32</td>
    <td>Target waypoint id</td>
    <td>-</td>
  </tr>
  <tr>
    <td>passing_waypoint_ids</td>
    <td>uint32[]</td>
    <td>IDs of all passing waypoints</td>
    <td>-</td>
  </tr>
  <tr>
    <td>result</td>
    <td>bool</td>
    <td>Result of follow path</td>
    <td>-</td>
  </tr>
  <tr>
    <td>error_code</td>
    <td>uint16</td>
    <td>Error code if follow path fails</td>
    <td>-</td>
  </tr>
  <tr>
    <td>current_pose</td>
    <td>geometry_msgs/PoseStamped</td>
    <td>Current position of AMR</td>
    <td>-</td>
  </tr>
  <tr>
    <td>passing_waypoint_id</td>
    <td>uint32</td>
    <td>Passing waypoint ID during follow path</td>
    <td>-</td>
  </tr>
  <tr>
    <td>distance_to_goal</td>
    <td>float32</td>
    <td>Distance from current position to goal</td>
    <td>-</td>
  </tr>
</table>

> [!Note]
> Other modules can send this command requests to start a follow path navigation.
  
### 🔹 `qrb_follow_path_manager` APIs

<table>
  <tr>
    <th>Function</th>
    <th>Parameters</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>uint32_t add_waypoint(point_2d & point)</td>
    <td>point: Position (x, y, angle)</td>
    <td>Add a waypoint.</td>
  </tr>
  <tr>
    <td>bool remove_waypoint(uint32_t id)</td>
    <td>id: Waypoint ID</td>
    <td>Remove waypoint by waypoint ID.</td>
  </tr>
  <tr>
    <td>bool get_waypoint_id_list(std::vector<uint32_t> & list)</td>
    <td>list: List of all waypoint IDs</td>
    <td>Get the list of waypoint IDs.</td>
  </tr>
  <tr>
    <td>bool get_waypoint(uint32_t id, point_2d & point)</td>
    <td>id: Waypoint ID, point: Waypoint info</td>
    <td>Get waypoint info by ID.</td>
  </tr>
  <tr>
    <td>bool add_virtual_path(uint32_t id, std::vector<uint32_t> & ids)</td>
    <td>id: Waypoint ID, ids: Set of adjacent waypoint IDs</td>
    <td>Add virtual path relationships.</td>
  </tr>
  <tr>
    <td>bool remove_virtual_path(uint32_t id, std::vector<uint32_t> & ids)</td>
    <td>id: Waypoint ID, ids: Set of adjacent waypoint IDs</td>
    <td>Remove virtual path relationships.</td>
  </tr>
  <tr>
    <td>bool get_virtual_path(uint32_t id, std::vector<uint32_t> & ids)</td>
    <td>id: Waypoint ID, ids: Set of adjacent waypoint IDs</td>
    <td>Get virtual path by waypoint ID.</td>
  </tr>
  <tr>
    <td>void set_bypassing_obstacle(bool val)</td>
    <td>val: Enable/disable obstacle avoidance</td>
    <td>Enable or disable obstacle avoidance.</td>
  </tr>
  <tr>
    <td>bool remove_waypoint_and_virtual_path()</td>
    <td>-</td>
    <td>Remove all waypoints and virtual paths.</td>
  </tr>
  <tr>
    <td>uint64_t request_follow_path(uint32_t goal, std::vector<uint32_t> & passing_ids)</td>
    <td>goal: Target waypoint ID, passing_ids: Passing waypoint IDs</td>
    <td>Start follow path navigation; path must include passing waypoints.</td>
  </tr>
  <tr>
    <td>float get_distance_to_goal(point_2d & current_point, uint32_t passing_id)</td>
    <td>current_point: Current position, passing_id: Passing waypoint ID</td>
    <td>Get distance from current point to goal.</td>
  </tr>
  <tr>
    <td>uint32_t get_passing_waypoint_id(point_2d & current_point)</td>
    <td>current_point: Position (x, y, angle)</td>
    <td>Get last passing waypoint ID for current point.</td>
  </tr>
  <tr>
    <td>void request_stop_follow_path()</td>
    <td>-</td>
    <td>Stop current follow path navigation.</td>
  </tr>
  <tr>
    <td>void register_navigation_callback(navigation_completed_func_t cb)</td>
    <td>cb: Navigation completed callback</td>
    <td>Register navigation completed callback.</td>
  </tr>
  <tr>
    <td>void register_publish_real_path_callback(publish_real_path_func_t cb)</td>
    <td>cb: Real path publishing callback</td>
    <td>Register real path publishing callback.</td>
  </tr>
  <tr>
    <td>void register_publish_global_path_callback(publish_global_path_func_t cb)</td>
    <td>cb: Global path publishing callback</td>
    <td>Register global path publishing callback.</td>
  </tr>
  <tr>
    <td>void register_publish_twist_callback(publish_twist_func_t cb)</td>
    <td>cb: Twist topic publishing callback</td>
    <td>Register twist topic publishing callback.</td>
  </tr>
  <tr>
    <td>void register_get_robot_velocity_callback(get_robot_velocity_func_t cb)</td>
    <td>cb: Robot velocity callback</td>
    <td>Register robot velocity callback.</td>
  </tr>
  <tr>
    <td>void register_get_grid_map_callback(get_grid_map_func_t cb)</td>
    <td>cb: Grid map callback</td>
    <td>Register grid map callback.</td>
  </tr>
  <tr>
    <td>void update_current_pose(point_2d & point)</td>
    <td>point: Position (x, y, angle)</td>
    <td>Update current position.</td>
  </tr>
  <tr>
    <td>void request_pause_follow_path()</td>
    <td>-</td>
    <td>Pause current follow path navigation.</td>
  </tr>
  <tr>
    <td>void request_resume_follow_path()</td>
    <td>-</td>
    <td>Resume current follow path navigation.</td>
  </tr>
  <tr>
    <td>void handle_amr_exception(bool exception)</td>
    <td>exception: Enter/exit exception state</td>
    <td>Enter or exit exception state.</td>
  </tr>
  <tr>
    <td>void handle_emergency(bool enter)</td>
    <td>enter: Enter/exit emergency state</td>
    <td>Enter or exit emergency state.</td>
  </tr>
  <tr>
    <td>void send_debug_event(int16_t event)</td>
    <td>event: Follow path event</td>
    <td>Send debugging information to simulate follow path event.</td>
  </tr>
  <tr>
    <td>bool compute_follow_path(bool use_start, uint32_t start, uint32_t goal, std::vector<uint32_t> & list, std::vector<point_2d> & point_list)</td>
    <td>use_start: Use starting waypoint, start: Start waypoint ID, goal: Target waypoint ID, list: Waypoint IDs, point_list: Path points</td>
    <td>Compute the follow path.</td>
  </tr>
  <tr>
    <td>bool compute_follow_path(bool use_start, uint32_t start, uint32_t goal, std::vector<uint32_t> & passing_ids, std::vector<uint32_t> & list, std::vector<point_2d> & point_list);</td>
    <td>use_start: Use starting waypoint, start: Start waypoint ID, goal: Target waypoint ID, passing_ids: Passing waypoint IDs, list: Waypoint IDs, point_list: Path points</td>
    <td>Compute the follow path including passing waypoints.</td>
  </tr>
</table>

> [!Note]
> These APIs enable integration of the follow path service in non-ROS applications. For ROS packages, please use the qrb_ros_follow_path_service APIs.

## 🎯 Supported targets

<table >
  <tr>
    <th>Development Hardware</th>
    <td>Qualcomm Dragonwing™ RB3 Gen2</td>
    <td>Qualcomm Dragonwing™ IQ-9075 EVK</td>
  </tr>
  <tr>
    <th>Hardware Overview</th>
    <th><a href="https://www.qualcomm.com/developer/hardware/rb3-gen-2-development-kit"><img src="https://s7d1.scene7.com/is/image/dmqualcommprod/rb3-gen2-carousel?fmt=webp-alpha&qlt=85" width="180"/></a></th>
    <th><a href="https://www.qualcomm.com/products/internet-of-things/industrial-processors/iq9-series/iq-9075"><img src="https://s7d1.scene7.com/is/image/dmqualcommprod/dragonwing-IQ-9075-EVK?$QC_Responsive$&fmt=png-alpha" width="160"></a></th>
  </tr>
</table>

---

## ✨ Installation

> [!IMPORTANT]
> **PREREQUISITES**: The following steps need to be run on **Qualcomm Ubuntu** and **ROS Jazzy**.<br>
> Reference [Install Ubuntu on Qualcomm IoT Platforms](https://ubuntu.com/download/qualcomm-iot) and [Install ROS Jazzy](https://docs.ros.org/en/jazzy/index.html) to setup environment. <br>
> For Qualcomm Linux, please check out the [Qualcomm Intelligent Robotics Product SDK](https://docs.qualcomm.com/bundle/publicresource/topics/80-70018-265/introduction_1.html?vproduct=1601111740013072&version=1.4&facet=Qualcomm%20Intelligent%20Robotics%20Product%20(QIRP)%20SDK) documents.

## 👨‍💻 Build from source

Install dependencies:

```bash
sudo apt install ros-jazzy-nav-msgs
sudo apt install ros-jazzy-nav-2d-msgs
```

Clone and build:
```bash
source /opt/ros/jazzy/setup.bash
git clone https://github.com/qualcomm-qrb-ros/qrb_ros_follow_path_service.git
git clone https://github.com/qualcomm-qrb-ros/qrb_ros_interfaces.git
colcon build
```

## 🚀 Usage

### Start the follow path service node

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch qrb_ros_follow_path qrb_ros_follow_path_bringup.launch.py
```

Sample output:

```bash
[INFO] [launch]: All log files can be found below /home/ubuntu/.ros/log/2025-09-17-11-04-38-768025-ubuntu-3557
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [qrb_ros_follow_path-1]: process started with pid [3560]
[qrb_ros_follow_path-1] [INFO] [1758107079.175347749] [navigation_controller]: init_nodes
[qrb_ros_follow_path-1] [virtual_path_manager]: Create VirtualPathManager
[qrb_ros_follow_path-1] [INFO] [1758107079.195297973] [virtual_path_service_server]: Creating
[qrb_ros_follow_path-1] [follow_path_planner]: register_get_robot_velocity_callback
[qrb_ros_follow_path-1] [INFO] [1758107079.224424074] [laser_scan_subscriber]: LaserScanSubscriber
[qrb_ros_follow_path-1] [virtual_path_manager]: register_get_grid_map_callback
[qrb_ros_follow_path-1] [obstacle_detector]: register_get_grid_map_callback
[qrb_ros_follow_path-1] [follow_path_planner]: register_publish_twist_callback
[qrb_ros_follow_path-1] [follow_path_planner]: register_publish_real_path_callback
[qrb_ros_follow_path-1] [follow_path_planner]: register_publish_global_path_callback
[qrb_ros_follow_path-1] [INFO] [1758107079.263415737] [follow_path_action_server]: Creating
[qrb_ros_follow_path-1] [follow_path_planner]: register_navigation_callback
[qrb_ros_follow_path-1] [INFO] [1758107079.273477796] [tf_subscriber]: Creating
[qrb_ros_follow_path-1] [INFO] [1758107079.286161690] [exception_subscriber]: Creating
[qrb_ros_follow_path-1] [INFO] [1758107079.296170160] [navigation_path_service_server]: Creating
[qrb_ros_follow_path-1] [INFO] [1758107079.306287944] [developer_mode_subscriber]: DeveloperModeSubscriber
...
```

## 🤝 Contributing

We love community contributions! Get started by reading our [CONTRIBUTING.md](CONTRIBUTING.md).<br>
Feel free to create an issue for bug report, feature requests or any discussion💡.

## ❤️ Contributors

Thanks to all our contributors who have helped make this project better!

<table>
  <tr>
    <td align="center"><a href="https://github.com/quic-zhanlin"><img src="https://avatars.githubusercontent.com/u/88314584?v=4" width="100" height="100" alt="quic-zhanlin"/><br /><sub><b>quic-zhanlin</b></sub></a></td>
    <td align="center"><a href="https://github.com/xiaowz-robotics"><img src="https://avatars.githubusercontent.com/u/154509668?v=4" width="100" height="100" alt="xiaowz-robotics"/><br /><sub><b>xiaowz-robotics</b></sub></a></td>
  </tr>
</table>

## ❔ FAQs

<details>
<summary>Does it support other AMR?</summary><br>
Yes, it can support other AMRs.
</details>

## 📜 License

Project is licensed under the [BSD-3-Clause](https://spdx.org/licenses/BSD-3-Clause.html) License. See [LICENSE](./LICENSE) for the full license text.



