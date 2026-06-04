# Yahboom DOFBot Arm

![Yahboom DOFBot Arm](gallery/arm.png)

This repository is Docker-only. The supported workflow is to build and run the ROS 2 Jazzy stack inside Docker and launch the Qt GUI from the container.

## What’s kept

- `docker/` - Dockerfile, compose file, and container entrypoint
- `src/` - ROS 2 packages for the arm bridge and MediaPipe detectors
- `third_party/Arm_Lib/` - vendor arm library copied into the image
- `RoboControl.py` - Qt entrypoint used inside the container
- `requirements-jazzy.txt` - Python dependencies for the image build
- `start_robocontrol_container_gui.sh` - Docker-only launcher for the GUI container
- `.env.example` - optional environment template for device paths and detector flags

## Build and run

```bash
DOFBOT_SERIAL_DEVICE=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
DOFBOT_CAMERA_DEVICE=/dev/video0 \
./start_robocontrol_container_gui.sh
```

That command rebuilds the image, starts the ROS stack in the container, and opens the GUI on the host display through X11.

## License

This project is licensed under the Apache License 2.0. See [LICENSE](LICENSE).

## Container stack only

If you want the ROS stack without the GUI, use:

```bash
docker compose -f docker/docker-compose.yml up -d --build
```

## Arm Control

![Arm Control panel](gallery/panel.png)

## Product Links (DOFBOT SE 6DOF)

- Official product page: https://category.yahboom.net/products/dofbot-se
- Official tutorial page: http://www.yahboom.net/study/DOFBOT_SE
- Yahboom store search: https://category.yahboom.net/search?q=DOFBOT+SE
- AliExpress search: https://de.aliexpress.com/w/wholesale-Yahboom-DOFBOT-SE-6DOF.html


## Notes

- The arm serial device is usually available under `/dev/serial/by-id/...`.
- The camera is published from `/mediapipe/camera/image/compressed`.
- Face detection and pose nodes are controlled through the environment flags in `.env` or the launcher command.
