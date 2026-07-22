FROM ros:jazzy-ros-base

ARG USER_ID
ARG GROUP_ID

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV PIP_NO_CACHE_DIR=1

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    build-essential \
    cmake \
    curl \
    git \
    gosu \
    python3-colcon-common-extensions \
    python3-pip \
    python3-setuptools \
    python3-venv \
    python3-yaml \
    libeigen3-dev \
    libpcl-dev \
    nano \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-ros-gz \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-eigen \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-rviz2 && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --ignore-installed PyYAML \
    openai \
    pydantic \
    pydantic-ai

RUN touch /var/lib/lastlog && \
    if getent passwd ${USER_ID}; then userdel -f $(getent passwd ${USER_ID} | cut -d: -f1); fi && \
    if getent group ${GROUP_ID}; then groupdel $(getent group ${GROUP_ID} | cut -d: -f1); fi || true

RUN groupadd -g ${GROUP_ID} user && \
    useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /bin/bash user && \
    echo "user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

WORKDIR /home/user/workspace

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc && \
    echo "if [ -f /home/user/workspace/install/setup.bash ]; then source /home/user/workspace/install/setup.bash; fi" >> /etc/bash.bashrc && \
    echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash'" >> /etc/bash.bashrc && \
    echo "alias cs='source install/setup.bash'" >> /etc/bash.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER user
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]