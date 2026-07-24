ARG FROM_IMAGE=ros:jazzy-ros-base
ARG OVERLAY_WS=/opt/ros/overlay_ws

FROM ${FROM_IMAGE} AS pydeps
ARG OVERLAY_WS

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    libportaudio2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${OVERLAY_WS}
COPY requirements-jazzy.txt ./requirements-jazzy.txt
RUN python3 -m pip install --no-cache-dir --break-system-packages \
    --retries 5 --default-timeout 60 --ignore-installed numpy \
    -r requirements-jazzy.txt

FROM pydeps AS builder
ARG OVERLAY_WS

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

COPY src ./src
COPY third_party ./third_party
COPY config ./config
COPY models ./models

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select arm_mediapipe dofbot_mediapipe"

FROM pydeps AS runner
ARG OVERLAY_WS
ENV OVERLAY_WS=${OVERLAY_WS}
ENV PYTHONPATH=${OVERLAY_WS}
ENV CUDA_VISIBLE_DEVICES=""

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 \
    espeak-ng \
    fontconfig \
    fonts-dejavu-core \
    pulseaudio \
    pulseaudio-utils \
    libasound2-plugins \
    alsa-utils \
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
COPY --from=builder ${OVERLAY_WS}/install ./install
COPY src ./src
COPY third_party ./third_party
COPY config ./config
COPY models ./models
COPY RoboControl.py ./RoboControl.py
COPY robocontrol_gui.py ./robocontrol_gui.py
COPY robocontrol_bootstrap.py ./robocontrol_bootstrap.py
COPY gallery ./gallery
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["stack"]
