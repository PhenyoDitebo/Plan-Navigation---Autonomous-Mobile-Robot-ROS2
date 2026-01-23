FROM osrf/ros:jazzy-desktop

# Use bash everywhere
SHELL ["/bin/bash", "-c"]

# Workspace
WORKDIR /ros2_ws

# Copy source
COPY src src

# Build workspace
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install

# Entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
