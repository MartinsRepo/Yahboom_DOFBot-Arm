"""Runtime bootstrap helpers for the RoboControl Qt launcher."""

from __future__ import annotations

import os
import sys


def ensure_py312_runtime_for_main(script_path: str) -> None:
    """Verify the runtime is suitable for the container entrypoint."""

    if sys.version_info < (3, 12):
        raise RuntimeError(
            f"{script_path} requires Python 3.12 or newer; got {sys.version.split()[0]}"
        )

    overlay_ws = os.environ.get("OVERLAY_WS", "/opt/ros/overlay_ws")
    if overlay_ws not in sys.path:
        sys.path.insert(0, overlay_ws)

    install_path = os.path.join(overlay_ws, "install")
    if os.path.isdir(install_path) and install_path not in sys.path:
        sys.path.insert(0, install_path)


def configure_qt_runtime_environment() -> None:
    """Set Qt defaults that are expected inside the Docker GUI container."""
    display = os.environ.get("DISPLAY", "")
    
    can_connect_x11 = False
    if display.startswith(":") or "/" in display:
        sock_num = display.split(":")[-1].split(".")[0]
        sock_path = f"/tmp/.X11-unix/X{sock_num}"
        if os.path.exists(sock_path):
            can_connect_x11 = True
    elif ":" in display:
        try:
            parts = display.split(":")
            host = parts[0] or "127.0.0.1"
            port_num = 6000 + int(parts[1].split(".")[0])
            import socket
            with socket.create_connection((host, port_num), timeout=0.5):
                can_connect_x11 = True
        except Exception:
            can_connect_x11 = False

    if not can_connect_x11 and os.environ.get("QT_QPA_PLATFORM") != "offscreen":
        print(
            f"[RoboControl Bootstrap] Notice: Cannot connect to X11 DISPLAY '{display}'. "
            "Falling back to QT_QPA_PLATFORM=offscreen.",
            file=sys.stderr,
        )
        os.environ["QT_QPA_PLATFORM"] = "offscreen"
    else:
        os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

    os.environ.setdefault("QT_X11_NO_MITSHM", "1")
    os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "1")
