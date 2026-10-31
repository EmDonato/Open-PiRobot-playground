FROM ros:jazzy-ros-base-noble
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential libeigen3-dev python3-colcon-common-extensions python3-yaml \
    ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2 ros-jazzy-slam-toolbox ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy ros-jazzy-teleop-twist-keyboard \
    libgl1-mesa-dri libglx-mesa0 \
    && rm -rf /var/lib/apt/lists/*
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
