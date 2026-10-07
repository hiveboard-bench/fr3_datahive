# ROS 1 Noetic + libfranka + franka_ros + DataHive recorder for the FR3.
# The DataHive tools need Python >= 3.10 but Noetic is Python 3.8, so the recorder runs in a
# uv-managed 3.10 venv (/opt/datahive-venv) that borrows the pure-Python ROS packages.
FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    build-essential \
    ffmpeg \
    git \
    curl \
    ca-certificates \
    python3-catkin-tools \
    python3-rosdep \
    v4l-utils \
    libv4l-dev \
    pkg-config \
    usbutils \
    psmisc \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip setuptools wheel \
    && python3 -m pip install "numpy>=1.20.0,<2" "opencv-python-headless==4.10.0.84"

ARG LIBFRANKA_VERSION=0.18.2
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    libpoco-dev \
    libfmt-dev \
    libeigen3-dev \
    build-essential \
    curl \
    ca-certificates \
    gnupg2 \
    && curl -fsSL https://apt.kitware.com/keys/kitware-archive-latest.asc \
    | gpg --dearmor -o /etc/apt/trusted.gpg.d/kitware.gpg \
    && echo "deb https://apt.kitware.com/ubuntu/ focal main" > /etc/apt/sources.list.d/kitware.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
    cmake=3.22.2-0kitware1ubuntu20.04.1 \
    cmake-data=3.22.2-0kitware1ubuntu20.04.1 \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --recursive --branch boost-1.77.0 --depth 1 https://github.com/boostorg/boost.git /tmp/boost \
    && cd /tmp/boost \
    && ./bootstrap.sh --prefix=/opt/franka-deps \
    && ./b2 -j"$(nproc)" install \
    && rm -rf /tmp/boost

RUN git clone --branch 10.0.0 --depth 1 https://github.com/leethomason/tinyxml2.git /tmp/tinyxml2 \
    && cd /tmp/tinyxml2 && mkdir build && cd build \
    && cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/opt/franka-deps -DCMAKE_INSTALL_LIBDIR=/opt/franka-deps/lib \
    && make -j"$(nproc)" && make install \
    && rm -rf /tmp/tinyxml2

RUN git clone --branch 1.0.2 --depth 1 https://github.com/ros/console_bridge.git /tmp/console_bridge \
    && cd /tmp/console_bridge && mkdir build && cd build \
    && cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/opt/franka-deps -DCMAKE_INSTALL_LIBDIR=/opt/franka-deps/lib \
    && make -j"$(nproc)" && make install \
    && rm -rf /tmp/console_bridge

RUN git clone --branch 1.0.5 --depth 1 https://github.com/ros/urdfdom_headers.git /tmp/urdfdom_headers \
    && cd /tmp/urdfdom_headers && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/franka-deps -DCMAKE_INSTALL_LIBDIR=/opt/franka-deps/lib \
    && make -j"$(nproc)" && make install \
    && rm -rf /tmp/urdfdom_headers

RUN curl -fsSL "https://raw.githubusercontent.com/frankarobotics/libfranka/${LIBFRANKA_VERSION}/.ci/urdfdom.patch" -o /tmp/urdfdom.patch \
    && git clone --branch 4.0.0 --depth 1 https://github.com/ros/urdfdom.git /tmp/urdfdom \
    && cd /tmp/urdfdom && git apply /tmp/urdfdom.patch \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_PREFIX_PATH=/opt/franka-deps -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=OFF \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/franka-deps -DCMAKE_INSTALL_LIBDIR=/opt/franka-deps/lib \
    && make -j"$(nproc)" && make install \
    && rm -rf /tmp/urdfdom /tmp/urdfdom.patch

RUN git clone --recursive --branch v5.4.3 --depth 1 https://github.com/assimp/assimp.git /tmp/assimp \
    && cd /tmp/assimp && mkdir build && cd build \
    && cmake .. -DBoost_USE_STATIC_LIBS=ON -DCMAKE_PREFIX_PATH=/opt/franka-deps -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DBUILD_SHARED_LIBS=OFF -DASSIMP_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/opt/franka-deps -DCMAKE_INSTALL_LIBDIR=/opt/franka-deps/lib \
    && make -j"$(nproc)" && make install \
    && rm -rf /tmp/assimp

RUN curl -fsSL "https://raw.githubusercontent.com/frankarobotics/libfranka/${LIBFRANKA_VERSION}/.ci/pinocchio.patch" -o /tmp/pinocchio.patch \
    && git clone --recursive --branch v3.4.0 --depth 1 https://github.com/stack-of-tasks/pinocchio.git /tmp/pinocchio \
    && cd /tmp/pinocchio && git apply /tmp/pinocchio.patch \
    && mkdir build && cd build \
    && cmake .. -DBoost_USE_STATIC_LIBS=ON -DCMAKE_PREFIX_PATH=/opt/franka-deps -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DBUILD_SHARED_LIBS=OFF -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_DOCUMENTATION=OFF -DBUILD_TESTING=OFF \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/franka-deps -DCMAKE_INSTALL_LIBDIR=/opt/franka-deps/lib \
    && make -j"$(nproc)" && make install \
    && rm -rf /tmp/pinocchio /tmp/pinocchio.patch

RUN git clone --recursive --branch ${LIBFRANKA_VERSION} --depth 1 https://github.com/frankarobotics/libfranka.git /tmp/libfranka \
    && cd /tmp/libfranka \
    && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DCMAKE_PREFIX_PATH=/opt/franka-deps .. \
    && cmake --build . -j"$(nproc)" \
    && cmake --build . --target install \
    && ldconfig \
    && rm -rf /tmp/libfranka

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-actionlib \
    ros-noetic-angles \
    ros-noetic-combined-robot-hw \
    ros-noetic-control-msgs \
    ros-noetic-control-toolbox \
    ros-noetic-controller-interface \
    ros-noetic-controller-manager \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-eigen-conversions \
    ros-noetic-hardware-interface \
    ros-noetic-joint-limits-interface \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-joint-state-publisher \
    ros-noetic-kdl-parser \
    ros-noetic-message-generation \
    ros-noetic-message-runtime \
    ros-noetic-pluginlib \
    ros-noetic-realtime-tools \
    ros-noetic-robot-state-publisher \
    ros-noetic-roscpp \
    ros-noetic-rospy \
    ros-noetic-tf \
    ros-noetic-tf-conversions \
    ros-noetic-transmission-interface \
    ros-noetic-urdf \
    ros-noetic-visualization-msgs \
    ros-noetic-xacro \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p "${CATKIN_WS}/src"
WORKDIR ${CATKIN_WS}

ARG FRANKA_ROS_VERSION=0.10.2
COPY docker/patches/franka-gripper-nonblocking-state-publish.patch /tmp/
RUN git clone --depth 1 --branch "${FRANKA_ROS_VERSION}" \
    https://github.com/frankarobotics/franka_ros.git "${CATKIN_WS}/src/franka_ros" \
    && git -C "${CATKIN_WS}/src/franka_ros" apply \
    /tmp/franka-gripper-nonblocking-state-publish.patch \
    && rm /tmp/franka-gripper-nonblocking-state-publish.patch \
    && rm -rf "${CATKIN_WS}/src/franka_ros/franka_gazebo" \
    && rm -rf "${CATKIN_WS}/src/franka_ros/.git" \
    && test "$(grep -Eh '^[[:space:]]*set[[:space:]]*\([[:space:]]*CMAKE_CXX_STANDARD[[:space:]]+14[[:space:]]*\)' \
    "${CATKIN_WS}"/src/franka_ros/*/CMakeLists.txt | wc -l)" -eq 5 \
    && sed -Ei 's/^[[:space:]]*set[[:space:]]*\([[:space:]]*CMAKE_CXX_STANDARD[[:space:]]+14[[:space:]]*\)/set(CMAKE_CXX_STANDARD 17)/' \
    "${CATKIN_WS}"/src/franka_ros/*/CMakeLists.txt \
    && test "$(grep -Fxh 'set(CMAKE_CXX_STANDARD 17)' \
    "${CATKIN_WS}"/src/franka_ros/*/CMakeLists.txt | wc -l)" -eq 5 \
    && ! grep -Eh '^[[:space:]]*set[[:space:]]*\([[:space:]]*CMAKE_CXX_STANDARD[[:space:]]+(11|14|20|23)[[:space:]]*\)' \
    "${CATKIN_WS}"/src/franka_ros/*/CMakeLists.txt

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && cd "${CATKIN_WS}" \
    && catkin config --extend "/opt/ros/${ROS_DISTRO}" >/dev/null \
    && catkin build --no-status \
    && catkin clean --logs --yes >/dev/null

ARG DATAHIVE_SPEC="datahive-tools[video]>=0.1.7"
ENV UV_PYTHON_INSTALL_DIR=/opt/python
RUN python3 -m pip install uv \
    && uv python install 3.10 \
    && uv venv --python 3.10 /opt/datahive-venv \
    && uv pip install --python /opt/datahive-venv/bin/python "${DATAHIVE_SPEC}" \
       rospkg catkin_pkg defusedxml empy PyYAML

COPY ros/fr3_control ${CATKIN_WS}/src/fr3_control
COPY ros/fr3_teleop ${CATKIN_WS}/src/fr3_teleop
COPY ros/fr3_datahive ${CATKIN_WS}/src/fr3_datahive
RUN python3 -m pip install "setuptools<66" \
    && source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && cd "${CATKIN_WS}" && catkin build --no-status fr3_control fr3_teleop fr3_datahive \
    && catkin clean --logs --yes >/dev/null

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && source "${CATKIN_WS}/devel/setup.bash" \
    && /opt/datahive-venv/bin/python -c "import rospy, datahive, cv2, fr3_datahive.recorder, fr3_teleop_support.joystick; \
from franka_msgs.msg import FrankaState; print('recorder imports OK, datahive', datahive.__file__)"

RUN mkdir -p /data/samples /data/recordings /root/.datahive
VOLUME ["/data/samples", "/data/recordings", "/root/.datahive"]
EXPOSE 8000

RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc \
    && echo '[ -f ${CATKIN_WS}/devel/setup.bash ] && source ${CATKIN_WS}/devel/setup.bash' >> /root/.bashrc \
    && ln -s /opt/datahive-venv/bin/datahive /usr/local/bin/datahive

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
