#!/usr/bin/env python3
# encoding: utf-8
"""MediaPipe hand-landmark gesture detector for DOFBot arm control.

Subscribes to the compressed camera topic, classifies hand gestures via
MediaPipe Hand Landmarks (21 keypoints per hand, up to 2 hands), and
publishes arm commands on ``roboarm/gesture_command``.

Gesture → Action mapping
========================
Thumbs up          → move_up
Thumbs down        → move_down
Thumbs left        → move_left
Thumbs right       → move_right
Open hand (5 ext)  → refresh  (stop)
Fist (all curled)  → home
V-pose (thumb+idx) → grip_open
Pinch (thumb→idx)  → grip_close
V-pose + wrist L   → turn_left
V-pose + wrist R   → turn_right
Pinch + wrist L    → turn_left
Pinch + wrist R    → turn_right
Diamond open (2h)  → arm_stretch
Diamond closed (2h)→ arm_shrink
"""

import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# ── Python 3.12 runtime guard (shared pattern with other detector scripts) ──
PY312_ENV_MARKER = 'DOFBOT_PY312_REEXEC'


def ensure_py312_runtime():
    if sys.version_info[:2] == (3, 12):
        return
    if os.environ.get(PY312_ENV_MARKER) == '1':
        return

    workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    py312_python = os.path.join(workspace_root, '.venv312', 'bin', 'python')
    if not os.path.exists(py312_python):
        raise RuntimeError(
            f'Python 3.12 runtime not found at {py312_python}. '
            'Create it before running ROS Jazzy detector scripts.'
        )

    new_env = dict(os.environ)
    new_env[PY312_ENV_MARKER] = '1'
    host_lib_dir = os.path.join(workspace_root, '.host-libs')
    if os.path.isdir(host_lib_dir):
        existing_ld_path = new_env.get('LD_LIBRARY_PATH', '')
        new_env['LD_LIBRARY_PATH'] = host_lib_dir if not existing_ld_path else f'{host_lib_dir}:{existing_ld_path}'
    os.execvpe(py312_python, [py312_python, os.path.abspath(__file__), *sys.argv[1:]], new_env)


ensure_py312_runtime()

import cv2 as cv  # noqa: E402
import numpy as np  # noqa: E402
import mediapipe as mp  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import CompressedImage  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

# Throttle interval for periodic info logs (avoid flooding the terminal)
_LOG_INTERVAL_S = 1.0

# ── Topic names ──────────────────────────────────────────────────────────────
CAMERA_TOPIC = os.environ.get('DOFBOT_GESTURE_CAMERA_TOPIC', '/mediapipe/webcam/image/compressed')
GESTURE_COMMAND_TOPIC = 'roboarm/gesture_command'
GESTURE_SUMMARY_TOPIC = '/mediapipe/gesture_summary'
GESTURE_ENABLED_TOPIC = '/robocontrol/gesture_enabled'

# ── Tuning constants ─────────────────────────────────────────────────────────
PINCH_THRESHOLD = 0.06           # normalised distance for thumb-tip ↔ index-tip
V_SPREAD_THRESHOLD = 0.08       # min distance between thumb-tip & index-tip for V-pose
FINGER_CURL_MARGIN = 0.02       # y-offset tolerance for "finger is curled"
THUMB_ANGLE_VERTICAL_DEG = 35   # max angle from vertical to count as up/down
WRIST_ROTATION_THRESHOLD = 25   # degrees from neutral to trigger turn_left/right
DIAMOND_CLOSE_THRESHOLD = 0.08  # normalised dist for "diamond closed"
DIAMOND_OPEN_THRESHOLD = 0.15   # normalised dist for "diamond open"

DEBOUNCE_FRAMES = 3             # consecutive frames required before emitting
COMMAND_COOLDOWN_S = 0.15       # min interval between published commands

SHOW_PREVIEW = os.environ.get('DOFBOT_SHOW_PREVIEW', '0').strip().lower() in ('1', 'true', 'yes', 'on')


# ── Landmark indices (MediaPipe convention) ──────────────────────────────────
WRIST = 0
THUMB_CMC = 1
THUMB_MCP = 2
THUMB_IP = 3
THUMB_TIP = 4
INDEX_MCP = 5
INDEX_PIP = 6
INDEX_DIP = 7
INDEX_TIP = 8
MIDDLE_MCP = 9
MIDDLE_PIP = 10
MIDDLE_DIP = 11
MIDDLE_TIP = 12
RING_MCP = 13
RING_PIP = 14
RING_DIP = 15
RING_TIP = 16
PINKY_MCP = 17
PINKY_PIP = 18
PINKY_DIP = 19
PINKY_TIP = 20


@dataclass
class LandmarkPoint:
    """Simplified 3-D landmark (normalised coords)."""
    x: float
    y: float
    z: float


@dataclass
class GestureResult:
    """Result of the gesture classifier for a single frame."""
    gesture: str = 'none'
    confidence: float = 0.0
    num_hands: int = 0
    wrist_angle_deg: float = 0.0
    detail: str = ''


# ═══════════════════════════════════════════════════════════════════════════════
#  Gesture classifier – pure geometry, no ROS dependency
# ═══════════════════════════════════════════════════════════════════════════════

class GestureClassifier:
    """Stateless classifier that maps hand landmarks to a gesture name."""

    # ── helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _dist(a: LandmarkPoint, b: LandmarkPoint) -> float:
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    @staticmethod
    def _dist3d(a: LandmarkPoint, b: LandmarkPoint) -> float:
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

    @staticmethod
    def _angle_deg(dx: float, dy: float) -> float:
        """Angle in degrees of vector (dx, dy) from positive-x axis, range [-180, 180]."""
        return math.degrees(math.atan2(dy, dx))

    # ── finger state ─────────────────────────────────────────────────────────

    def _is_finger_extended(self, lm: List[LandmarkPoint], tip: int, pip: int) -> bool:
        """A finger is extended when its tip is further from the wrist than its PIP joint."""
        return self._dist(lm[tip], lm[WRIST]) > self._dist(lm[pip], lm[WRIST]) + FINGER_CURL_MARGIN

    def _is_finger_curled(self, lm: List[LandmarkPoint], tip: int, mcp: int) -> bool:
        """A finger is curled when its tip is closer to the wrist than its MCP joint."""
        return self._dist(lm[tip], lm[WRIST]) < self._dist(lm[mcp], lm[WRIST]) + FINGER_CURL_MARGIN

    def _fingers_extended(self, lm: List[LandmarkPoint]) -> Tuple[bool, bool, bool, bool]:
        """Return (index, middle, ring, pinky) extended flags."""
        return (
            self._is_finger_extended(lm, INDEX_TIP, INDEX_PIP),
            self._is_finger_extended(lm, MIDDLE_TIP, MIDDLE_PIP),
            self._is_finger_extended(lm, RING_TIP, RING_PIP),
            self._is_finger_extended(lm, PINKY_TIP, PINKY_PIP),
        )

    def _all_fingers_extended(self, lm: List[LandmarkPoint]) -> bool:
        idx, mid, ring, pinky = self._fingers_extended(lm)
        # Thumb: tip further from wrist than IP joint
        thumb_ext = self._is_finger_extended(lm, THUMB_TIP, THUMB_IP)
        return all([thumb_ext, idx, mid, ring, pinky])

    def _all_fingers_curled(self, lm: List[LandmarkPoint]) -> bool:
        """All four fingers curled AND thumb curled."""
        idx_c = self._is_finger_curled(lm, INDEX_TIP, INDEX_MCP)
        mid_c = self._is_finger_curled(lm, MIDDLE_TIP, MIDDLE_MCP)
        ring_c = self._is_finger_curled(lm, RING_TIP, RING_MCP)
        pinky_c = self._is_finger_curled(lm, PINKY_TIP, PINKY_MCP)
        # Thumb curled: tip is closer to wrist than IP
        thumb_c = self._is_finger_curled(lm, THUMB_TIP, THUMB_IP)
        return all([thumb_c, idx_c, mid_c, ring_c, pinky_c])

    def _non_thumb_index_curled(self, lm: List[LandmarkPoint]) -> bool:
        """Middle, ring, pinky are curled (for thumb-direction & pinch/V gestures)."""
        mid_c = self._is_finger_curled(lm, MIDDLE_TIP, MIDDLE_MCP)
        ring_c = self._is_finger_curled(lm, RING_TIP, RING_MCP)
        pinky_c = self._is_finger_curled(lm, PINKY_TIP, PINKY_MCP)
        return all([mid_c, ring_c, pinky_c])

    # ── thumb direction ──────────────────────────────────────────────────────

    def _classify_thumb_direction(self, lm: List[LandmarkPoint]) -> Optional[str]:
        """Classify thumb pointing direction when other fingers are curled."""
        dx = lm[THUMB_TIP].x - lm[THUMB_MCP].x
        dy = lm[THUMB_TIP].y - lm[THUMB_MCP].y

        # Require a minimum displacement to avoid noise
        magnitude = math.sqrt(dx * dx + dy * dy)
        if magnitude < 0.04:
            return None

        angle = self._angle_deg(dx, -dy)  # negate dy because y-axis is inverted in image coords

        # Up: angle ≈ 90° (pointing up in image = thumb tip has lower y)
        if abs(angle - 90) < THUMB_ANGLE_VERTICAL_DEG:
            return 'thumb_up'
        # Down: angle ≈ -90°
        if abs(angle + 90) < THUMB_ANGLE_VERTICAL_DEG:
            return 'thumb_down'
        # Left: angle ≈ 180° or -180°
        if abs(angle) > (180 - THUMB_ANGLE_VERTICAL_DEG):
            return 'thumb_left'
        # Right: angle ≈ 0°
        if abs(angle) < THUMB_ANGLE_VERTICAL_DEG:
            return 'thumb_right'

        return None

    # ── wrist rotation ───────────────────────────────────────────────────────

    def _wrist_rotation_deg(self, lm: List[LandmarkPoint]) -> float:
        """Angle of the wrist→middle-MCP vector from vertical.

        Positive = rotated clockwise (right), negative = counter-clockwise (left).
        """
        dx = lm[MIDDLE_MCP].x - lm[WRIST].x
        dy = lm[MIDDLE_MCP].y - lm[WRIST].y
        # Angle from vertical (straight up = 0°)
        angle = math.degrees(math.atan2(dx, -dy))
        return angle

    # ── pinch / V-pose ───────────────────────────────────────────────────────

    def _is_pinch(self, lm: List[LandmarkPoint]) -> bool:
        return self._dist(lm[THUMB_TIP], lm[INDEX_TIP]) < PINCH_THRESHOLD

    def _is_v_pose(self, lm: List[LandmarkPoint]) -> bool:
        """Thumb and index spread apart, other fingers curled."""
        spread = self._dist(lm[THUMB_TIP], lm[INDEX_TIP])
        idx_ext = self._is_finger_extended(lm, INDEX_TIP, INDEX_PIP)
        return spread > V_SPREAD_THRESHOLD and idx_ext

    # ── two-hand diamond ─────────────────────────────────────────────────────

    def _classify_diamond(self, lm_left: List[LandmarkPoint], lm_right: List[LandmarkPoint]) -> Optional[str]:
        """Detect diamond shape formed by two hands.

        Diamond: thumbs of both hands touch each other, index fingers of both
        hands touch each other.  Measure the distance between the two touch
        points to determine open vs closed.
        """
        # Average position of thumb tips (the "top" of the diamond)
        thumb_mid_x = (lm_left[THUMB_TIP].x + lm_right[THUMB_TIP].x) / 2
        thumb_mid_y = (lm_left[THUMB_TIP].y + lm_right[THUMB_TIP].y) / 2

        # Average position of index tips (the "bottom" of the diamond)
        index_mid_x = (lm_left[INDEX_TIP].x + lm_right[INDEX_TIP].x) / 2
        index_mid_y = (lm_left[INDEX_TIP].y + lm_right[INDEX_TIP].y) / 2

        # Check that thumbs are close to each other
        thumb_dist = self._dist(lm_left[THUMB_TIP], lm_right[THUMB_TIP])
        index_dist = self._dist(lm_left[INDEX_TIP], lm_right[INDEX_TIP])

        # Both pairs of fingertips must be reasonably close
        if thumb_dist > 0.15 or index_dist > 0.15:
            return None

        # Diamond "height" — distance between thumb-midpoint and index-midpoint
        diamond_size = math.sqrt((thumb_mid_x - index_mid_x) ** 2 + (thumb_mid_y - index_mid_y) ** 2)

        if diamond_size > DIAMOND_OPEN_THRESHOLD:
            return 'diamond_open'
        if diamond_size < DIAMOND_CLOSE_THRESHOLD:
            return 'diamond_closed'
        return None

    # ── main classifier ──────────────────────────────────────────────────────

    def classify(self, hands_landmarks: list) -> GestureResult:
        """Classify gesture from one or two sets of hand landmarks.

        Parameters
        ----------
        hands_landmarks : list of List[LandmarkPoint]
            Each element is a list of 21 LandmarkPoint for one hand.

        Returns
        -------
        GestureResult
        """
        num_hands = len(hands_landmarks)
        if num_hands == 0:
            return GestureResult(gesture='none', num_hands=0)

        # ── Two-hand gestures first ──────────────────────────────────────────
        if num_hands >= 2:
            diamond = self._classify_diamond(hands_landmarks[0], hands_landmarks[1])
            if diamond == 'diamond_open':
                return GestureResult(gesture='diamond_open', confidence=0.85, num_hands=2, detail='arm_stretch')
            if diamond == 'diamond_closed':
                return GestureResult(gesture='diamond_closed', confidence=0.85, num_hands=2, detail='arm_shrink')

        # ── Single-hand gestures (use first detected hand) ───────────────────
        lm = hands_landmarks[0]
        wrist_angle = self._wrist_rotation_deg(lm)

        # 1) Open hand → stop
        if self._all_fingers_extended(lm):
            return GestureResult(gesture='open_hand', confidence=0.9, num_hands=1,
                                 wrist_angle_deg=wrist_angle, detail='stop')

        # 2) Fist → home
        if self._all_fingers_curled(lm):
            return GestureResult(gesture='fist', confidence=0.9, num_hands=1,
                                 wrist_angle_deg=wrist_angle, detail='home')

        # 3) Pinch (thumb + index touching) → grip_close or turn with wrist
        if self._is_pinch(lm) and self._non_thumb_index_curled(lm):
            if abs(wrist_angle) > WRIST_ROTATION_THRESHOLD:
                direction = 'turn_left' if wrist_angle < 0 else 'turn_right'
                return GestureResult(gesture=f'pinch_wrist_{direction}', confidence=0.8,
                                     num_hands=1, wrist_angle_deg=wrist_angle, detail=direction)
            return GestureResult(gesture='pinch', confidence=0.85, num_hands=1,
                                 wrist_angle_deg=wrist_angle, detail='grip_close')

        # 4) V-pose (thumb + index spread) → grip_open or turn with wrist
        if self._is_v_pose(lm) and self._non_thumb_index_curled(lm):
            if abs(wrist_angle) > WRIST_ROTATION_THRESHOLD:
                direction = 'turn_left' if wrist_angle < 0 else 'turn_right'
                return GestureResult(gesture=f'v_wrist_{direction}', confidence=0.8,
                                     num_hands=1, wrist_angle_deg=wrist_angle, detail=direction)
            return GestureResult(gesture='v_pose', confidence=0.85, num_hands=1,
                                 wrist_angle_deg=wrist_angle, detail='grip_open')

        # 5) Thumb directions (other fingers must be curled)
        idx_ext, mid_ext, ring_ext, pinky_ext = self._fingers_extended(lm)
        others_curled = not mid_ext and not ring_ext and not pinky_ext
        if others_curled:
            thumb_dir = self._classify_thumb_direction(lm)
            if thumb_dir is not None:
                action_map = {
                    'thumb_up': 'move_up',
                    'thumb_down': 'move_down',
                    'thumb_left': 'move_left',
                    'thumb_right': 'move_right',
                }
                action = action_map.get(thumb_dir, 'none')
                return GestureResult(gesture=thumb_dir, confidence=0.8, num_hands=1,
                                     wrist_angle_deg=wrist_angle, detail=action)

        return GestureResult(gesture='unknown', confidence=0.0, num_hands=num_hands,
                             wrist_angle_deg=wrist_angle)


# ── Action mapping ───────────────────────────────────────────────────────────

GESTURE_ACTION_MAP = {
    'thumb_up': 'move_up',
    'thumb_down': 'move_down',
    'thumb_left': 'move_left',
    'thumb_right': 'move_right',
    'open_hand': 'refresh',
    'fist': 'home',
    'v_pose': 'grip_open',
    'pinch': 'grip_close',
    'pinch_wrist_turn_left': 'turn_left',
    'pinch_wrist_turn_right': 'turn_right',
    'v_wrist_turn_left': 'turn_left',
    'v_wrist_turn_right': 'turn_right',
    'diamond_open': 'arm_stretch',
    'diamond_closed': 'arm_shrink',
}


# ═══════════════════════════════════════════════════════════════════════════════
#  ROS 2 node
# ═══════════════════════════════════════════════════════════════════════════════

def _landmarks_to_points(hand_landmarks) -> List[LandmarkPoint]:
    """Convert MediaPipe NormalizedLandmarkList to flat list of LandmarkPoint."""
    return [LandmarkPoint(x=lm.x, y=lm.y, z=lm.z) for lm in hand_landmarks.landmark]


class GestureDetector(Node):
    def __init__(self, min_detection_confidence: float = 0.7, min_tracking_confidence: float = 0.5):
        super().__init__('gesture_detector')

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self.mp_draw = mp.solutions.drawing_utils

        self.classifier = GestureClassifier()

        # Debounce state
        self._prev_gesture: str = 'none'
        self._gesture_streak: int = 0
        self._last_command_time: float = 0.0

        # ROS interfaces
        self.pub_command = self.create_publisher(String, GESTURE_COMMAND_TOPIC, 10)
        self.pub_summary = self.create_publisher(String, GESTURE_SUMMARY_TOPIC, 10)
        self.sub_camera = self.create_subscription(CompressedImage, CAMERA_TOPIC, self._handle_camera_frame, 10)
        self.sub_enabled = self.create_subscription(Bool, GESTURE_ENABLED_TOPIC, self._handle_enabled, 10)

        self.enabled = False
        self._last_log_time = 0.0

    # ── callbacks ────────────────────────────────────────────────────────────

    def _handle_enabled(self, message: Bool):
        was_enabled = self.enabled
        self.enabled = bool(message.data)
        if not self.enabled:
            self._prev_gesture = 'none'
            self._gesture_streak = 0
        if self.enabled != was_enabled:
            state = 'ENABLED' if self.enabled else 'DISABLED'
            self.get_logger().info(f'Gesture detection {state}')

    def _handle_camera_frame(self, message: CompressedImage):
        if not self.enabled:
            return

        data = np.frombuffer(message.data, dtype=np.uint8)
        frame = cv.imdecode(data, cv.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning('Failed decoding compressed camera frame')
            return

        self._process_frame(frame)

    # ── processing pipeline ──────────────────────────────────────────────────

    def _process_frame(self, frame: np.ndarray):
        img_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)

        hands_landmarks: list = []
        if results.multi_hand_landmarks:
            for hand_lm in results.multi_hand_landmarks:
                hands_landmarks.append(_landmarks_to_points(hand_lm))

        gesture_result = self.classifier.classify(hands_landmarks)

        # Publish lightweight JSON summary (GUI draws the landmarks itself)
        self._publish_summary(gesture_result, hands_landmarks)

        # Periodic info log (throttled to avoid flooding)
        now = time.time()
        if now - self._last_log_time >= _LOG_INTERVAL_S:
            self._last_log_time = now
            self.get_logger().info(
                f'Hands: {len(hands_landmarks)} | '
                f'Gesture: {gesture_result.gesture} | '
                f'Action: {gesture_result.detail or "n/a"} | '
                f'Streak: {self._gesture_streak}/{DEBOUNCE_FRAMES} | '
                f'Wrist: {gesture_result.wrist_angle_deg:.1f}\u00b0'
            )

        # Debounce
        if gesture_result.gesture == self._prev_gesture and gesture_result.gesture not in ('none', 'unknown'):
            self._gesture_streak += 1
        else:
            self._prev_gesture = gesture_result.gesture
            self._gesture_streak = 1 if gesture_result.gesture not in ('none', 'unknown') else 0

        # Emit command only after stable streak + cooldown
        if self._gesture_streak >= DEBOUNCE_FRAMES:
            if now - self._last_command_time >= COMMAND_COOLDOWN_S:
                action = GESTURE_ACTION_MAP.get(gesture_result.gesture)
                if action:
                    self._publish_command(action, now)
                    self._last_command_time = now
                    self.get_logger().info(
                        f'>>> COMMAND sent: {action} (gesture={gesture_result.gesture}, '
                        f'streak={self._gesture_streak})'
                    )

        # Optional local preview window (draws landmarks for dev debugging)
        if SHOW_PREVIEW:
            annotated = frame.copy()
            if results.multi_hand_landmarks:
                for hand_lm in results.multi_hand_landmarks:
                    self.mp_draw.draw_landmarks(
                        annotated, hand_lm, self.mp_hands.HAND_CONNECTIONS,
                    )
            label = f'{gesture_result.gesture} ({gesture_result.detail})'
            cv.putText(annotated, label, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv.imshow('gesture_detection', annotated)
            cv.waitKey(1)

    def _publish_command(self, action: str, now: float):
        payload = {
            'action': action,
            'timestamp_s': now,
            'source': 'gesture_detector',
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.pub_command.publish(msg)

    def _publish_summary(self, result: GestureResult, hands_landmarks: list = None):
        landmarks_data = []
        if hands_landmarks:
            for hand_lm in hands_landmarks:
                hand_data = [{'x': round(p.x, 4), 'y': round(p.y, 4), 'z': round(p.z, 4)} for p in hand_lm]
                landmarks_data.append(hand_data)

        payload = {
            'gesture': result.gesture,
            'confidence': round(result.confidence, 3),
            'num_hands': result.num_hands,
            'wrist_angle_deg': round(result.wrist_angle_deg, 1),
            'detail': result.detail,
            'action': GESTURE_ACTION_MAP.get(result.gesture, ''),
            'landmarks': landmarks_data,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.pub_summary.publish(msg)


# ── main ─────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    rclpy.init(args=None)
    min_det_conf = float(os.environ.get('DOFBOT_GESTURE_MIN_DET_CONF', '0.70'))
    min_trk_conf = float(os.environ.get('DOFBOT_GESTURE_MIN_TRK_CONF', '0.50'))
    node = GestureDetector(min_detection_confidence=min_det_conf, min_tracking_confidence=min_trk_conf)
    node.get_logger().info(
        f'Gesture detector started: subscribing to {CAMERA_TOPIC}, '
        f'publishing commands on {GESTURE_COMMAND_TOPIC} '
        f"(preview={'on' if SHOW_PREVIEW else 'off'})"
    )
    try:
        rclpy.spin(node)
    finally:
        cv.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
