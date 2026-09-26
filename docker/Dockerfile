# Build and test image. Everything needed is already in ros:jazzy-ros-base
# (urdf, kdl_parser, ament_cmake_gtest, launch_testing, Eigen); nothing extra is installed.
FROM ros:jazzy-ros-base
WORKDIR /ws
COPY models /ws/models
COPY ros2_ws/src /ws/ros2_ws/src
ENV YARO_MODELS_DIR=/ws/models
RUN . /opt/ros/jazzy/setup.sh && cd /ws/ros2_ws && \
    colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
CMD ["bash", "-c", ". /ws/ros2_ws/install/setup.sh && cd /ws/ros2_ws && colcon test --event-handlers console_direct+ --return-code-on-test-failure && colcon test-result --verbose"]
