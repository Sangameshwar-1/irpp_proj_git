# =============================================================================
#  IRPP Social Navigation — Docker Image
#  ROS 2 Jazzy  ·  Gazebo Harmonic  ·  Ubuntu 24.04
# =============================================================================

FROM osrf/ros:jazzy-desktop-full AS base

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

SHELL ["/bin/bash", "-c"]

# ── System dependencies ──────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Gazebo Harmonic + ROS-Gazebo bridge
    ros-jazzy-ros-gz \
    # Build tools
    python3-colcon-common-extensions \
    python3-pip \
    # Python libraries used by the project
    python3-numpy \
    python3-opencv \
    python3-pil \
    python3-yaml \
    # ROS 2 packages used by nodes
    ros-jazzy-cv-bridge \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-msgs \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-ros-gz-sim \
    # GUI / display
    ros-jazzy-rviz2 \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1 \
    x11-apps \
    # Utilities
    vim \
    nano \
    less \
    tree \
    && rm -rf /var/lib/apt/lists/*

# ── Suppress Gazebo default world / quick-start dialog ───────────────
# The Gazebo Harmonic GUI shows a "Quick Start" dialog on first launch
# that can load the default "shapes" world (9 poles).  Disable it and
# provide a clean server config so only the explicitly-requested world
# is loaded.
RUN mkdir -p /root/.gz/sim/8 && \
    echo '<?xml version="1.0"?>\n\
<dialog name="quick_start" show_again="false"/>\n\
<window>\n\
  <width>1000</width>\n\
  <height>845</height>\n\
  <menus><drawer default="false"/></menus>\n\
  <dialog_on_exit>false</dialog_on_exit>\n\
</window>\n\
<plugin filename="MinimalScene" name="3D View">\n\
  <gz-gui>\n\
    <title>3D View</title>\n\
    <property type="bool" key="showTitleBar">false</property>\n\
    <property type="string" key="state">docked</property>\n\
  </gz-gui>\n\
  <engine>ogre2</engine>\n\
  <scene>scene</scene>\n\
  <ambient_light>0.4 0.4 0.4</ambient_light>\n\
  <background_color>0.8 0.8 0.8</background_color>\n\
  <camera_pose>-6 0 6 0 0.5 0</camera_pose>\n\
</plugin>\n\
<plugin filename="GzSceneManager" name="Scene Manager">\n\
  <gz-gui>\n\
    <property key="resizable" type="bool">false</property>\n\
    <property key="width" type="double">5</property>\n\
    <property key="height" type="double">5</property>\n\
    <property key="state" type="string">floating</property>\n\
    <property key="showTitleBar" type="bool">false</property>\n\
  </gz-gui>\n\
</plugin>\n\
<plugin filename="InteractiveViewControl" name="Interactive view control">\n\
  <gz-gui>\n\
    <property key="resizable" type="bool">false</property>\n\
    <property key="width" type="double">5</property>\n\
    <property key="height" type="double">5</property>\n\
    <property key="state" type="string">floating</property>\n\
    <property key="showTitleBar" type="bool">false</property>\n\
  </gz-gui>\n\
</plugin>\n\
<plugin filename="CameraTracking" name="Camera Tracking">\n\
  <gz-gui>\n\
    <property key="resizable" type="bool">false</property>\n\
    <property key="width" type="double">5</property>\n\
    <property key="height" type="double">5</property>\n\
    <property key="state" type="string">floating</property>\n\
    <property key="showTitleBar" type="bool">false</property>\n\
  </gz-gui>\n\
</plugin>\n\
<plugin filename="MarkerManager" name="Marker manager">\n\
  <gz-gui>\n\
    <property key="resizable" type="bool">false</property>\n\
    <property key="width" type="double">5</property>\n\
    <property key="height" type="double">5</property>\n\
    <property key="state" type="string">floating</property>\n\
    <property key="showTitleBar" type="bool">false</property>\n\
  </gz-gui>\n\
</plugin>\n\
<plugin filename="EntityTree" name="Entity tree">\n\
  <gz-gui>\n\
    <property type="bool" key="showTitleBar">false</property>\n\
    <property type="string" key="state">docked</property>\n\
  </gz-gui>\n\
</plugin>' > /root/.gz/sim/8/gui.config && \
    echo '<?xml version="1.0"?>\n\
<server_config>\n\
  <plugins>\n\
    <plugin entity_name="*" entity_type="world"\n\
            filename="gz-sim-physics-system"\n\
            name="gz::sim::systems::Physics"/>\n\
    <plugin entity_name="*" entity_type="world"\n\
            filename="gz-sim-user-commands-system"\n\
            name="gz::sim::systems::UserCommands"/>\n\
    <plugin entity_name="*" entity_type="world"\n\
            filename="gz-sim-scene-broadcaster-system"\n\
            name="gz::sim::systems::SceneBroadcaster"/>\n\
  </plugins>\n\
</server_config>' > /root/.gz/sim/8/server.config

# ── Create workspace ─────────────────────────────────────────────────
ENV ROS_WS=/root/ros2_ws
RUN mkdir -p ${ROS_WS}/src

# ── Copy project into container ──────────────────────────────────────
COPY scan_project/human_models/ros_humans_ros2 ${ROS_WS}/src/ros_humans_ros2
COPY scan_project/rviz_config.rviz /root/rviz_config.rviz

# Copy shell scripts
COPY run_navigation.sh /root/run_navigation.sh
COPY run_test_case.sh  /root/run_test_case.sh
COPY run_test_scenario.sh /root/run_test_scenario.sh
RUN chmod +x /root/*.sh

# ── Build the ROS 2 workspace ────────────────────────────────────────
WORKDIR ${ROS_WS}
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --packages-select ros_humans_ros2

# ── Create map output directory ──────────────────────────────────────
RUN mkdir -p /root/ros2_maps

# ── Entrypoint ───────────────────────────────────────────────────────
COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["bash"]
