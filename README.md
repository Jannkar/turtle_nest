![Turtle Nest](images/turtle_nest_logo_large.png)

![Turtle Nest CI Pipeline](https://github.com/jannkar/turtle_nest/actions/workflows/run_pipeline.yaml/badge.svg)

# Turtle Nest

Just as the natural turtle nests are the birthplace for young turtles, ROS 2 Turtle Nest is where new ROS packages are born and brought to life.

Turtle Nest provides an easy graphical user interface for creating new ROS packages, simplifying the package creation process.

## Why to use Turtle Nest?

- **Easy to use** - no need to dig through the ROS 2 documentation for the right commands or manually add things to CMakeLists.txt or setup.py.
- **Creates C++ and/or Python nodes** - which are ready for immediate development.
- **Automatically adds necessary dependencies** - rclpy, rclcpp, and std_msgs.
- **Option to create a launch file** - to run your nodes with a single command.
- **Option to create a parameter file** - with parameter examples for the created nodes.
- **Create C++ and Python nodes within the same package** - combine languages that typically require separate packages.
- **Naming conventions enforced** - no more failing builds due to incorrect package or node names.
- **Remembers the important details** - workspace path, maintainer name, and maintainer email for your future packages.
- **And more!**

## Prerequisites

- Ubuntu 22 or 24
- ROS 2 (any one of the following versions)
  - Humble
  - Iron
  - Jazzy
  - Rolling

## Installation

Create a new ROS 2 workspace if you don't yet have one. Then clone the repository, install dependencies and build the application.
```
mkdir -p $HOME/ros2_ws/src/
cd $HOME/ros2_ws/src/
git clone https://github.com/Jannkar/turtle_nest.git
cd ..
sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}
colcon build
source install/setup.bash
```

## Usage
To run the application, simply execute:

```
turtle-nest
```

Fill in the necessary information in the GUI to create a new ROS 2 package. After the package has been created, build and source the new package:

```
cd $HOME/ros2_ws/
colcon build
source install/setup.bash
```

If you created a node or a launch file, you can run them with the following commands:
* To run a node:
    ```
    ros2 run <package_name> <node_name>
    ```
* To launch the launch file
    ```
    ros2 launch <package_name> <launch_file_name>
    ```

## Build Status

| Distro | Status | 
| :---:  | :---:  |
| Humble | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__turtle_nest__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__turtle_nest__ubuntu_jammy_amd64__binary/)|
| Iron | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Ibin_uJ64__turtle_nest__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Ibin_uJ64__turtle_nest__ubuntu_jammy_amd64__binary/)  |
| Jazzy | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__turtle_nest__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__turtle_nest__ubuntu_noble_amd64__binary/)  |
| Rolling | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rbin_uN64__turtle_nest__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Rbin_uN64__turtle_nest__ubuntu_noble_amd64__binary/)  |

## Screenshots

![Screenshot 1](images/screenshot_p1.png)
![Screenshot 2](images/screenshot_p2.png)
![Screenshot 3](images/screenshot_p3.png)

## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=Jannkar/turtle_nest&type=Date)](https://star-history.com/#Jannkar/turtle_nest&Date)

## Maintainers

- [Janne Karttunen](https://www.linkedin.com/in/janne-karttunen-a22375209/)
