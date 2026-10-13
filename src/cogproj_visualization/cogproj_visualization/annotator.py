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
        mode_label: str = "",
        raw_detections_count: Optional[int] = None,
        localized_count: Optional[int] = None,
        localization_status: str = "",
    ) -> np.ndarray:
        """Draw annotations and HUD panels onto a copy of the input frame.

        Args:
            frame_bgr: Input BGR image (e.g. 320x240 or 640x480 numpy array).
            tracked_ores: List of TrackedOre messages from the tracking pipeline.
            counts_summary: Latest OreCountSummary message from the counting pipeline.
            status: Status dictionary (e.g. {"camera": "OK", "detection": "ACTIVE", ...}).
            mode_label: Explicit operational mode tag (e.g. "MOCK / SIMULATED" or "REAL: YOLOv8s").
            raw_detections_count: Number of 2D bounding boxes in the current frame.
            localized_count: Number of objects successfully localized on the SLAM map.
            localization_status: Actionable diagnostic status of spatial localization.

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

        # 2. Draw Top Status HUD with Mode Badge and Actionable Diagnostics
        self._draw_status_hud(canvas, status, mode_label, localization_status, w)

        # 3. Draw Bottom Count HUD distinguishing detections, tracks, session, and map counts
        self._draw_count_hud(
            canvas,
            counts_summary,
            tracked_ores,
            raw_detections_count,
            localized_count,
            w,
            h,
        )

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

    def _draw_status_hud(
        self,
        canvas: np.ndarray,
        status: Optional[Dict[str, str]],
        mode_label: str,
        localization_status: str,
        w: int,
    ) -> None:
        """Draw top banner showing camera, detector, tracking, counting statuses, and mode badge."""
        if status is None:
            status = {}

        cam_st = status.get("camera", "WAITING")
        det_st = status.get("detection", "WAITING")
        trk_st = status.get("tracking", "WAITING")
        cnt_st = status.get("counting", "WAITING")

        # Top dark HUD banner (y: 0 to 20)
        cv2.rectangle(canvas, (0, 0), (w, 20), self.COLOR_TEXT_BG, -1)

        # Build actionable status suffix if any major component is blocked
        alert_parts = []
        if cam_st == "WAITING":
            alert_parts.append("CAM_UNAVAILABLE")
        if det_st == "WAITING":
            alert_parts.append("DET_WAITING")
        if localization_status == "UNCALIBRATED_CAMERA":
            alert_parts.append("CAM_UNCALIBRATED")
        elif localization_status == "NO_LIDAR_SCAN":
            alert_parts.append("LIDAR_UNAVAILABLE")
        elif localization_status == "TF_LOOKUP_FAILED":
            alert_parts.append("TF_UNAVAILABLE")

        alert_str = f" [{','.join(alert_parts)}]" if alert_parts else ""

        status_text = f"CAM:{cam_st} | DET:{det_st} | TRK:{trk_st} | CNT:{cnt_st}{alert_str}"

        # Color: green if active/ok, amber if any waiting or alert
        is_all_ok = (
            cam_st == "OK"
            and "WAIT" not in det_st
            and "WAIT" not in trk_st
            and "WAIT" not in cnt_st
            and not alert_parts
        )
        hud_color = self.COLOR_OK if is_all_ok else self.COLOR_WARN

        # Render left-aligned status text
        cv2.putText(
            canvas,
            status_text,
            (4, 14),
            self.font,
            self.font_scale,
            hud_color,
            1,
            cv2.LINE_AA,
        )

        # Render right-aligned mode badge (e.g. [MOCK / SIMULATED] or [REAL: YOLOv8s])
        if mode_label:
            badge_text = f"[{mode_label}]"
            is_mock = "MOCK" in mode_label.upper() or "SIMULATED" in mode_label.upper()
            badge_color = self.COLOR_WARN if is_mock else self.COLOR_OK
            (bw, bh), _ = cv2.getTextSize(badge_text, self.font, self.font_scale, 1)
            badge_x = max(w - bw - 6, int(w * 0.55))
            # Dark backing for the badge
            cv2.rectangle(canvas, (badge_x - 3, 2), (w - 2, 18), (35, 35, 35), -1)
            cv2.putText(
                canvas,
                badge_text,
                (badge_x, 14),
                self.font,
                self.font_scale,
                badge_color,
                1,
                cv2.LINE_AA,
            )

    def _draw_count_hud(
        self,
        canvas: np.ndarray,
        counts_summary: Optional[OreCountSummary],
        tracked_ores: Optional[List[TrackedOre]],
        raw_detections_count: Optional[int],
        localized_count: Optional[int],
        w: int,
        h: int,
    ) -> None:
        """Draw bottom banner distinguishing raw detections, active tracks, distinct session count, and mapped count."""
        # Bottom dark HUD banner (y: h-28 to h)
        cv2.rectangle(canvas, (0, h - 28), (w, h), self.COLOR_TEXT_BG, -1)

        total_count = counts_summary.total_count if counts_summary is not None else 0
        active_tracks = len(tracked_ores) if tracked_ores is not None else 0
        confirmed_tracks = (
            sum(1 for t in tracked_ores if getattr(t, "state", "") == "CONFIRMED")
            if tracked_ores is not None
            else 0
        )
        det_str = f"{raw_detections_count}" if raw_detections_count is not None else "-"
        map_str = f"{localized_count}" if localized_count is not None else "0"

        # Build per-class count summary (e.g., "res:2 rock:1")
        class_parts = []
        if counts_summary is not None and len(counts_summary.class_names) > 0:
            for name, count in zip(counts_summary.class_names, counts_summary.class_counts):
                short_name = name[:4].lower()
                class_parts.append(f"{short_name}:{count}")

        class_summary_str = " ".join(class_parts) if class_parts else "none"

        # Line 1: Metrics distinguishing the 4 core quantities
        line1_text = (
            f"CNT:{total_count} (Session) | MAP:{map_str} | "
            f"TRK:{active_tracks} (CONF:{confirmed_tracks}) | DET:{det_str}"
        )

        # Line 2: Per-class session totals
        line2_text = f"CLASSES: {class_summary_str}"

        cv2.putText(
            canvas,
            line1_text,
            (4, h - 16),
            self.font,
            self.font_scale * 0.95,
            self.COLOR_WHITE,
            1,
            cv2.LINE_AA,
        )

        cv2.putText(
            canvas,
            line2_text,
            (4, h - 5),
            self.font,
            self.font_scale * 0.90,
            self.COLOR_OK,
            1,
            cv2.LINE_AA,
        )

