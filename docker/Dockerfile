ARG BASE_IMAGE=ros:humble
FROM $BASE_IMAGE

# Setup a user
ARG UNAME=user
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} -o ${UNAME}
RUN useradd -m -u ${UID} -g ${GID} -o -s /bin/bash ${UNAME}
RUN usermod -aG sudo ${UNAME}
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER ${UNAME}
WORKDIR /home/user

# Create a custom workspace
ENV ROS2_WS=/home/user/ros2_ws
RUN mkdir -p ${ROS2_WS}/src

# Disable terminal interaction for apt
ENV DEBIAN_FRONTEND=noninteractive

# Make default shell in Dockerfile bash instead of sh
SHELL ["/bin/bash", "-c"]

# Setup cyclone DDS
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install dependencies
# ROS already comes with gtest, but not with libgmock. Install it
RUN sudo apt-get update && \
    sudo apt-get install -y --no-install-recommends \
    libgmock-dev \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

# Copy the Turtle Nest ROS package and build our custom workspace
COPY turtle_nest/ ${ROS2_WS}/src/turtle_nest
RUN cd ${ROS2_WS} && \
    sudo apt-get update && \
    . /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} && \
    colcon build --symlink-install && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

# Add the tests
COPY turtle_nest_tests ${ROS2_WS}/src/turtle_nest_tests
RUN sudo chown -R user:user ${ROS2_WS}/src && \
    cd ${ROS2_WS}/src/turtle_nest_tests && \
    mkdir build && cd build && \
    qmake .. && \
    make

# Create a folder for app configs
RUN mkdir -p /home/user/.config/TurtleNest/

# Source ROS workspace automatically when new terminal is opened
RUN sudo echo ". /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    sudo echo ". ${ROS2_WS}/install/setup.bash" >> ~/.bashrc

# Source ROS in the main terminal
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
