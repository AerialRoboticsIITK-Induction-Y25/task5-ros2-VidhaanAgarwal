# ==========================================
# STAGE 1: The Builder
# ==========================================
FROM ros:jazzy AS builder

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /drone_ws

COPY part1 ./part1
COPY fleet_ws ./fleet_ws

WORKDIR /drone_ws/fleet_ws
RUN rm -rf build install log

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"


# ==========================================
# STAGE 2: The Runtime
# ==========================================
FROM ros:jazzy AS runtime

WORKDIR /drone_ws/fleet_ws

COPY --from=builder /drone_ws/fleet_ws/install ./install

ENV AMENT_PREFIX_PATH=/drone_ws/fleet_ws/install/drone_fleet:/opt/ros/jazzy
ENV LD_LIBRARY_PATH=/drone_ws/fleet_ws/install/drone_fleet/lib:${LD_LIBRARY_PATH}

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && exec ros2 launch drone_fleet fleet.launch.py"]