# Yahboom DOFBot Arm (ROS 2 Jazzy Containerized Control)

![Yahboom DOFBot Arm](gallery/arm.png)

A high-performance, containerized control stack for the **Yahboom DOFBOT SE 6DOF** robotic arm built on **ROS 2 Jazzy**, **MediaPipe**, **PyQt5**, and **Vosk / Ollama** AI integration.

The system is optimized for **Windows CPU execution** (no NVIDIA CUDA GPU required) using **Podman** (or Docker Desktop) and **WSL2**.

---

## 🌟 Key Features

- **Dual Camera Stream Architecture**:
  - **Left Window (`Camera Stream Webcam`)**: Renders host PC Webcam feed with live MediaPipe **Hand Landmark Skeletons** and gesture recognition.
  - **Right Window (`FaceDetector Preview Arm`)**: Renders Arm Camera feed with live MediaPipe **Face Detection** bounding boxes and keypoints.
- **Hardware Integration**: DirectShow host camera streamer for zero-latency 30 FPS video bypass, and automated USB serial forwarding for the CH340 servo controller (`/dev/ttyUSB0`).
- **Offline Speech Control & Local LLM Integration**:
  - Offline **Vosk** speech recognition (German & English grammar modes).
  - Integration with **Ollama** LLMs (e.g. `qwen3.5:9b`, `codegemma:7b`, `llama3.1:8b`) for natural language robot instruction execution.
  - Windows microphone input bridged seamlessly into the container via **WSLg PulseAudio**.
- **PyQt5 Control GUI**: Interactive GUI with continuous press-and-hold servo movement, dynamic speed control, telemetry feedback, and mode switching (`GUI`, `LLM`, `AUTO`).

> 📖 **Architecture Documentation**: For detailed technical diagrams, sequence charts, and Agent Tool Control specifications, see [Architecture Documentation](doc/Architecture.md).

---

## ⚙️ Prerequisites & System Setup (Windows)

Before running the container stack on Windows, ensure the following prerequisites are installed and configured:

### 1. Windows Subsystem for Linux (WSL2) & WSLg
Make sure WSL2 is installed and updated to the latest version:
```powershell
wsl --update
```
*Note: WSLg includes native audio and GUI support (`/mnt/wslg/PulseServer` and `/tmp/.X11-unix`).*

### 2. Container Engine (Podman or Docker)
Install **Podman Desktop** (recommended) or **Docker Desktop**. Ensure the container engine is running and accessible in PowerShell.

### 3. X11 Display Server
Install an X11 server for Windows (such as **VcXsrv** or **Xming**) or use native WSLg graphics:
- **VcXsrv / Xming**: Start with **"Disable access control"** checked (`DISPLAY=172.20.240.1:0`).

### 4. USBIPD for Windows (Serial Port Forwarding)
The robotic arm uses a **CH340 USB-to-Serial Controller** (`VID_1A86&PID_7523`). Install `usbipd-win` to attach the serial hardware to WSL:
```powershell
winget install --exact --id dslab.usbipd-win
```
After attaching once or rebooting Windows, run the provided helper script in PowerShell/CMD:
```cmd
.\attach_dofbot_usb.bat
```
This binds `1a86:7523` and attaches it as `/dev/ttyUSB0` inside WSL.

### 5. Host Ollama LLM (Optional for AI Control)
Install [Ollama](https://ollama.com/) on your Windows host if you plan to use LLM voice/natural language commands.
Pull a supported model (e.g., `qwen3.5:9b`):
```powershell
ollama pull qwen3.5:9b
```
*The launcher script automatically detects installed host models (`qwen3.5:9b`, `codegemma:7b`, `llama3.1:8b`).*

---

## 🚀 Quick Start Guide

Launch the containerized GUI with a single command from PowerShell. The script automatically starts the background host camera bridge, binds audio/serial devices, builds the container image if needed, and opens the GUI.

### 1. German Preset with Gesture Detection (Recommended)
```powershell
.\start_robocontrol_container_gui.ps1 -Preset German -EnableGestureDetection
```

### 2. English Preset
```powershell
.\start_robocontrol_container_gui.ps1 -Preset English -EnableGestureDetection
```

### 3. Custom Model or Specific Engine
```powershell
.\start_robocontrol_container_gui.ps1 -Preset German -OllamaModel "codegemma:7b" -Engine podman
```

---

## 📋 PowerShell Launcher Parameter Reference

The PowerShell launcher `start_robocontrol_container_gui.ps1` accepts the following parameters:

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `-Preset` | String | `Custom` | Preset profile: `German`, `English`, or `Custom`. Sets default language and LLM flags. |
| `-EnableGestureDetection` | Switch | Off | Enables MediaPipe hand gesture detection on the left webcam preview. |
| `-EnableFaceDetection` | Switch | On (Presets) | Enables MediaPipe face detection overlay on the right arm camera preview. |
| `-EnableSpeechController`| Switch | On (Presets) | Activates the Vosk offline speech recognition node. |
| `-EnableLlmController` | Switch | On (Presets) | Activates the Ollama LLM natural language instruction execution node. |
| `-OllamaModel` | String | Auto-detected | Specifies the host Ollama model name (e.g. `qwen3.5:9b`, `codegemma:7b`, `llama3.1:8b`). |
| `-OllamaBaseUrl` | String | `http://127.0.0.1:11434` | Base URL for the Ollama LLM endpoint. |
| `-Engine` | String | `auto` | Forces container engine (`podman`, `docker`, or `auto`). |
| `-SpeechLanguage` | String | `de` / `en` | Vosk speech model language (`de` or `en`). |
| `-VoiceOutputEnabled` | Switch | On (Presets) | Enables speech audio feedback (`espeak-ng`). |

---

## 🖥️ PyQt5 GUI Interface & Control Modes

![Arm Control panel](gallery/panel.png)

### Control Modes
- **`GUI`**: Manual control via GUI buttons and sliders. LLM commands are rejected.
- **`LLM`**: Automated control via Ollama LLM and speech input. Manual panel inputs are limited to emergency stop/home.
- **`AUTO`**: Dual control mode. Accepts both LLM commands and GUI input. Manual GUI actions temporarily override LLM execution for safety.

### Main GUI Controls
- **Left Window (`Camera Stream Webcam`)**: Displays the secondary PC webcam feed with green hand landmark skeletons and active gesture labels when **Gesture Control** is active.
- **Right Window (`FaceDetector Preview Arm`)**: Displays the DOFBot arm camera feed with blue face bounding boxes and facial landmark points when **Face Detection** is active.
- **Movement Buttons**: Press and hold direction, stretch (`vor`), shrink (`zurück`), and wrist rotation buttons for continuous movement.
- **Speed Slider & REFRESH**: Dynamically adjusts servo movement duration (`move_duration_ms`). Clicking **REFRESH** syncs the slider value to the arm controller.
- **HOME / POWER**: Powers arm servos on/off or resets all 6 joints to default home posture.

---

## 🎤 Speech Recognition & Voice Commands

Offline speech recognition is powered by **Vosk**. Speech capture is routed from the Windows host USB microphone via WSLg PulseAudio (`/mnt/wslg/PulseServer`).

### Activation & Safety Phrases
1. **Wake Words**: Say **`martin`** (German) or **`robby`** (English) to activate voice command mode.
2. **Motion Stop**: Say **`Halt`** to immediately stop servo movement while keeping speech command mode active.
3. **Full Deactivation**: Say **`Stop`** or **`Stopp`** to halt motion and exit voice command mode.

### Supported Voice Commands
- **Directions**: `hoch` (up), `runter` (down), `links` (left), `rechts` (right)
- **Reach**: `vor` (stretch/forward), `zurück` (shrink/backward)
- **Gripper**: `nimm` / `zu` (close grip), `gib` / `auf` (open grip)
- **Wrist Rotation**: `dreh links` (turn wrist left), `dreh rechts` (turn wrist right)
- **System**: `home`, `an` (power on), `aus` (power off), `halt`, `stop`

---

## 🛠️ Testing & Diagnostic Utilities

Helper diagnostic scripts are located under [`src/testing`](file:///c:/Users/humme/workspace/Yahboom_DOFBot-Arm/src/testing):

- **[attach_dofbot_usb.bat](file:///c:/Users/humme/workspace/Yahboom_DOFBot-Arm/attach_dofbot_usb.bat)**: One-click script to attach the CH340 serial hardware (`1a86:7523`) to WSL.
- **[src/testing/test_win_cam.py](file:///c:/Users/humme/workspace/Yahboom_DOFBot-Arm/src/testing/test_win_cam.py)**: Probes available Windows host USB camera devices and tests resolution/FPS.
- **[src/testing/test_container_cam.py](file:///c:/Users/humme/workspace/Yahboom_DOFBot-Arm/src/testing/test_container_cam.py)**: Tests camera device paths inside the Linux container.
- **[src/testing/vosk_terminal_test.py](file:///c:/Users/humme/workspace/Yahboom_DOFBot-Arm/src/testing/vosk_terminal_test.py)**: Standalone terminal utility to test Vosk microphone input and grammar recognition:
  ```powershell
  podman run --rm -v /mnt/wslg/PulseServer:/tmp/pulse-socket:rw -e PULSE_SERVER=unix:/tmp/pulse-socket dofbot_ros:latest python3 /opt/ros/overlay_ws/src/testing/vosk_terminal_test.py --list-devices
  ```

---

## 📁 Repository Structure

- `start_robocontrol_container_gui.ps1` - Primary Windows launcher script (CPU mode, Podman/Docker auto-detection).
- `host_camera_streamer.py` - Host DirectShow dual camera bridge writing frame buffer files to `docker_data/log/`.
- `attach_dofbot_usb.bat` - USBIPD attachment script for CH340 arm serial controller.
- `robocontrol_gui.py` - PyQt5 dual-preview GUI control interface.
- `RoboControl.py` - Container entrypoint script initializing the ROS 2 node and Qt application.
- `docker/` - Container `Dockerfile`, compose file, and ROS entrypoint script.
- `src/arm_mediapipe/` - ROS 2 package for LLM controller, speech input, and launch files.
- `src/dofbot_mediapipe/` - ROS 2 package for MediaPipe camera driver, face detection, and gesture recognition.
- `src/testing/` - Diagnostic test scripts for cameras, audio, and speech recognition.
- `third_party/Arm_Lib/` - Vendor python library for Yahboom DOFBOT servo serial communication.

---

## 📜 License

This project is licensed under the Apache License 2.0. See [LICENSE](LICENSE).

## 🔗 Official Hardware Links (Yahboom DOFBOT SE)

- [Yahboom DOFBOT SE Product Page](https://category.yahboom.net/products/dofbot-se)
- [Yahboom DOFBOT SE Tutorial & Documentation](http://www.yahboom.net/study/DOFBOT_SE)
