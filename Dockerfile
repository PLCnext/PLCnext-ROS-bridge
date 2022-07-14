FROM registry.gitlab.cc-asp.fraunhofer.de/ipa326/phoenix_bridge/base_image:axcf3x_foxy_1.0

WORKDIR /root/bridge_ws/src
COPY . phoenix_bridge

WORKDIR /root/bridge_ws/
RUN . /opt/ros/foxy/setup.sh \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build --symlink-install\
    && . install/local_setup.bash
