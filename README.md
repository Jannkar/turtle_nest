![Turtle Nest](images/turtle_nest_logo_large.png)

Just as a turtle nest is the birthplace for young turtles, ROS 2 Turtle Nest is where new ROS packages are born and brought to life.

Turtle Nest provides an easy graphical user interface for creating new ROS packages, simplifying the package creation process.

<h2>Why to use Turtle Nest over 'ros2 pkg create' CLI?</h2>
* **Easy to use** - no need to dig through the ROS 2 documentation for the right commands or manually add things to CMakeLists.txt or setup.py to get started.
* **Create C++ and/or Python nodes** - which are ready for further development.
* **Automatically adds necessary dependencies** - rclpy, rclcpp, and std_msgs.
* **Option to create a launch file** - that launches the created node.
* **Supports both C++ and Python** - in a single package.
* **Naming conventions enforced** - no more failing builds due to incorrect package or node names.
* **Remembers the important details** - workspace path, maintainer name, and maintainer email for the future packages.
* **And more!**

<h2>Installation</h2>

Create a new ROS 2 workspace if you don't yet have one, clone the repository, install dependencies, build and run the application.
```
mkdir -p $HOME/ros2_ws/src/
cd $HOME/ros2_ws/src/
git clone https://github.com/Jannkar/turtle_nest.git
cd ..
sudo apt-get update && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}
colcon build && source install/setup.bash
turtle-nest
```

<h2>Screenshots</h2>

![Screenshot 1](images/screenshot_p1.png)
![Screenshot 2](images/screenshot_p2.png)
![Screenshot 3](images/screenshot_p3.png)


´export ROS2_WS=<workspace_path>´
For example 
´export ROS2_WS="/home/user/ros2_ws/src"´


## Maintainers

- [Janne Karttunen](https://www.linkedin.com/in/janne-karttunen-a22375209/)