ARG FROM_IMAGE=ros:jazzy-ros-base
ARG OVERLAY_WS=/opt/ros/overlay_ws

FROM ${FROM_IMAGE} AS builder
ARG OVERLAY_WS

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    libportaudio2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${OVERLAY_WS}
COPY requirements-jazzy.txt ./requirements-jazzy.txt
RUN python3 -m pip install --no-cache-dir --break-system-packages \
    --retries 5 --default-timeout 60 --ignore-installed numpy \
    -r requirements-jazzy.txt

COPY src ./src
COPY third_party ./third_party
COPY config ./config
COPY models ./models

RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select arm_mediapipe dofbot_mediapipe

FROM ${FROM_IMAGE} AS runner
ARG OVERLAY_WS
ENV OVERLAY_WS=${OVERLAY_WS}
ENV PYTHONPATH=${OVERLAY_WS}

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    libgl1 \
    libportaudio2 \
    espeak-ng \
    fontconfig \
    fonts-dejavu-core \
    libx11-xcb1 \
    libxcb1 \
    libxcb-cursor0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-shm0 \
    libxcb-sync1 \
    libxcb-util1 \
    libxcb-xfixes0 \
    libxcb-xinerama0 \
    libxcb-xkb1 \
    libxkbcommon-x11-0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${OVERLAY_WS}
COPY requirements-jazzy.txt ./requirements-jazzy.txt
RUN python3 -m pip install --no-cache-dir --break-system-packages \
    --retries 5 --default-timeout 60 --ignore-installed numpy \
    -r requirements-jazzy.txt

COPY --from=builder ${OVERLAY_WS}/install ./install
COPY src ./src
COPY third_party ./third_party
COPY config ./config
COPY models ./models
COPY RoboControl.py ./RoboControl.py
COPY robocontrol_gui.py ./robocontrol_gui.py
COPY robocontrol_bootstrap.py ./robocontrol_bootstrap.py
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["stack"]
