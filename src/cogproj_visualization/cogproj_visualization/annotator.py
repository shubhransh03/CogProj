"""Perception Annotator for CogProj.

Renders bounding boxes, persistent track IDs, confidence, tracking states,
and HUD perception/count status panels onto 320x240 camera frames.
"""

from typing import Dict, List, Optional, Tuple
import cv2
import numpy as np

from cogproj_interfaces.msg import OreCountSummary, TrackedOre


class PerceptionAnnotator:
    """Headless OpenCV rendering engine for annotating perception results onto camera frames."""

    # Color palette (BGR)
    COLOR_CONFIRMED = (0, 255, 0)      # Bright Green
    COLOR_TENTATIVE = (0, 180, 255)    # Amber / Orange
    COLOR_LOST = (128, 128, 128)       # Gray
    COLOR_TEXT_BG = (20, 20, 20)       # Dark Charcoal
    COLOR_WHITE = (255, 255, 255)      # White
    COLOR_OK = (100, 255, 100)         # Soft Green
    COLOR_WARN = (0, 220, 255)         # Yellow / Amber

    def __init__(self, font_scale: float = 0.30) -> None:
        """Initialize PerceptionAnnotator."""
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = font_scale

    def annotate_frame(
        self,
        frame_bgr: np.ndarray,
        tracked_ores: Optional[List[TrackedOre]] = None,
        counts_summary: Optional[OreCountSummary] = None,
        status: Optional[Dict[str, str]] = None,
    ) -> np.ndarray:
        """Draw annotations and HUD panels onto a copy of the input frame.

        Args:
            frame_bgr: Input BGR image (e.g. 320x240 numpy array).
            tracked_ores: List of TrackedOre messages from the tracking pipeline.
            counts_summary: Latest OreCountSummary message from the counting pipeline.
            status: Status dictionary (e.g. {"camera": "OK", "detection": "ACTIVE", ...}).

        Returns:
            Annotated BGR image as a new numpy array.
        """
        if frame_bgr is None or frame_bgr.size == 0:
            raise ValueError("Input frame is empty or None.")

        # Operate on a copy to preserve pristine camera data
        canvas = frame_bgr.copy()
        h, w = canvas.shape[:2]

        # 1. Draw Bounding Boxes and Track Labels
        if tracked_ores:
            for track in tracked_ores:
                self._draw_track(canvas, track, w, h)

        # 2. Draw Top Status HUD
        self._draw_status_hud(canvas, status, w)

        # 3. Draw Bottom Count HUD
        self._draw_count_hud(canvas, counts_summary, tracked_ores, w, h)

        return canvas

    def _draw_track(self, canvas: np.ndarray, track: TrackedOre, max_w: int, max_h: int) -> None:
        """Draw individual track bounding box, ID, confidence, and state."""
        # Clamp coordinates to frame boundaries
        x1 = max(0, min(int(round(track.x_min)), max_w - 1))
        y1 = max(0, min(int(round(track.y_min)), max_h - 1))
        x2 = max(0, min(int(round(track.x_max)), max_w - 1))
        y2 = max(0, min(int(round(track.y_max)), max_h - 1))

        # Select color based on track state
        if track.state == "CONFIRMED":
            color = self.COLOR_CONFIRMED
        elif track.state == "TENTATIVE":
            color = self.COLOR_TENTATIVE
        else:
            color = self.COLOR_LOST

        # Draw bounding box
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 1)

        # Build compact label: "#ID Class Conf [State]"
        state_tag = track.state[:4]  # CONF, TENT, LOST
        label = f"#{track.track_id} {track.class_name[:8]} {track.confidence:.2f} [{state_tag}]"

        # Calculate text size
        (tw, th), baseline = cv2.getTextSize(label, self.font, self.font_scale, 1)

        # Label position (above box if space permits, else inside top)
        label_y = y1 - 3 if y1 - th - 4 > 18 else y1 + th + 3
        label_y = max(19 + th, min(label_y, max_h - 25))
        label_x = max(0, min(x1, max_w - tw - 2))

        # Dark background backing for high legibility
        cv2.rectangle(
            canvas,
            (label_x, label_y - th - 2),
            (label_x + tw + 2, label_y + baseline),
            self.COLOR_TEXT_BG,
            -1,
        )

        # Render text
        cv2.putText(
            canvas,
            label,
            (label_x + 1, label_y - 1),
            self.font,
            self.font_scale,
            color,
            1,
            cv2.LINE_AA,
        )

    def _draw_status_hud(self, canvas: np.ndarray, status: Optional[Dict[str, str]], w: int) -> None:
        """Draw top banner showing camera, detector, tracking, and counter statuses."""
        if status is None:
            status = {}

        cam_st = status.get("camera", "WAITING")
        det_st = status.get("detection", "WAITING")
        trk_st = status.get("tracking", "WAITING")
        cnt_st = status.get("counting", "WAITING")

        # Top dark HUD banner (y: 0 to 18)
        cv2.rectangle(canvas, (0, 0), (w, 18), self.COLOR_TEXT_BG, -1)

        status_text = f"CAM:{cam_st} | DET:{det_st} | TRK:{trk_st} | CNT:{cnt_st}"

        # Color: green if active/ok, amber if any waiting
        is_all_ok = (cam_st == "OK" and "WAIT" not in det_st and "WAIT" not in trk_st and "WAIT" not in cnt_st)
        hud_color = self.COLOR_OK if is_all_ok else self.COLOR_WARN

        cv2.putText(
            canvas,
            status_text,
            (4, 13),
            self.font,
            self.font_scale,
            hud_color,
            1,
            cv2.LINE_AA,
        )

    def _draw_count_hud(
        self,
        canvas: np.ndarray,
        counts_summary: Optional[OreCountSummary],
        tracked_ores: Optional[List[TrackedOre]],
        w: int,
        h: int,
    ) -> None:
        """Draw bottom banner displaying total unique counts, active tracks, and per-class counts."""
        # Bottom dark HUD banner (y: h-22 to h)
        cv2.rectangle(canvas, (0, h - 22), (w, h), self.COLOR_TEXT_BG, -1)

        total_count = counts_summary.total_count if counts_summary is not None else 0
        active_tracks = len(tracked_ores) if tracked_ores is not None else 0

        # Build per-class count summary (e.g., "Hem:2 Azu:1")
        class_parts = []
        if counts_summary is not None and len(counts_summary.class_names) > 0:
            for name, count in zip(counts_summary.class_names, counts_summary.class_counts):
                class_parts.append(f"{name[:4]}:{count}")

        class_summary_str = " ".join(class_parts) if class_parts else "none"

        count_text = f"TOTAL:{total_count} | TRKS:{active_tracks} | {class_summary_str}"

        cv2.putText(
            canvas,
            count_text,
            (4, h - 7),
            self.font,
            self.font_scale,
            self.COLOR_WHITE,
            1,
            cv2.LINE_AA,
        )

