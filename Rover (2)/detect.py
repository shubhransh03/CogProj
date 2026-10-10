"""
YOLO detection stream → MediaMTX RTSP
Optimized for Raspberry Pi 5
"""

import cv2
from ultralytics import YOLO
import subprocess
import os
import time

# ── Configuration ─────────────────────────────

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "best.pt")

CAMERA_INDEX = 0
CONF_THRESHOLD = 0.5

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 15
IMGSZ = 320

CLASS_COLORS = {
    "resource": (0,255,0),
    "host_rock": (255,128,0),
}

DEFAULT_COLOR = (0,0,255)


# ── Draw detections ───────────────────────────

def draw_detections(frame, results):

    counts = {"resource":0,"host_rock":0}

    for result in results:

        boxes = result.boxes
        if boxes is None:
            continue

        for box in boxes:

            x1,y1,x2,y2 = map(int, box.xyxy[0])
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])

            cls_name = result.names[cls_id]

            if conf < CONF_THRESHOLD:
                continue

            if cls_name in counts:
                counts[cls_name] += 1

            color = CLASS_COLORS.get(cls_name, DEFAULT_COLOR)

            label = f"{cls_name} {conf:.2f}"

            cv2.rectangle(frame,(x1,y1),(x2,y2),color,1)

            (tw,th),_ = cv2.getTextSize(label,cv2.FONT_HERSHEY_SIMPLEX,0.4,1)

            cv2.rectangle(frame,(x1,y1-th-6),(x1+tw+4,y1),color,-1)

            cv2.putText(
                frame,
                label,
                (x1+2,y1-3),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255,255,255),
                1,
                cv2.LINE_AA
            )

    return frame, counts


# ── HUD overlay ───────────────────────────────

def draw_hud(frame, counts, fps):

    lines = [
        f"Resource: {counts['resource']}",
        f"Host Rock: {counts['host_rock']}",
        f"FPS: {fps:.1f}"
    ]

    y = 18

    for line in lines:

        cv2.putText(frame,line,(9,y),
                    cv2.FONT_HERSHEY_SIMPLEX,0.5,
                    (0,0,0),2,cv2.LINE_AA)

        cv2.putText(frame,line,(8,y),
                    cv2.FONT_HERSHEY_SIMPLEX,0.5,
                    (0,255,255),1,cv2.LINE_AA)

        y += 22


# ── Main ──────────────────────────────────────

def main():

    print("[INFO] Loading YOLO model...")
    model = YOLO(MODEL_PATH)

    print("[INFO] Opening camera...")
    cap = cv2.VideoCapture(CAMERA_INDEX)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("[ERROR] Camera not found")
        return

    print("[INFO] Starting FFmpeg stream to MediaMTX...")

    ffmpeg_cmd = [
        "ffmpeg",
        "-loglevel","quiet",
        "-re",
        "-f","rawvideo",
        "-pix_fmt","bgr24",
        "-s",f"{FRAME_WIDTH}x{FRAME_HEIGHT}",
        "-r",str(FPS),
        "-i","-",
        "-c:v","libx264",
        "-preset","ultrafast",
        "-tune","zerolatency",
        "-f","rtsp",
        "rtsp://localhost:8554/mystream"
    ]

    ffmpeg = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)

    prev = time.time()

    while True:

        cap.grab()
        ret, frame = cap.read()

        if not ret:
            continue

        results = model.predict(
            source=frame,
            conf=CONF_THRESHOLD,
            imgsz=IMGSZ,
            verbose=False
        )

        frame, counts = draw_detections(frame, results)

        now = time.time()
        fps = 1/max(now-prev,1e-6)
        prev = now

        draw_hud(frame, counts, fps)

        ffmpeg.stdin.write(frame.tobytes())


if __name__ == "__main__":
    main()

