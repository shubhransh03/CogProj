import cv2
import numpy as np
import subprocess
import os
import time
from threading import Thread
from queue import Queue
import onnxruntime as ort

# ── CONFIG ───────────────────────────────────

MODEL_PATH = os.path.join(os.path.dirname(__file__), "best.onnx")

CAMERA_INDEX = 0
CONF_THRESHOLD = 0.5
NMS_THRESHOLD = 0.4

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 15
IMGSZ = 320   # try 256 for more FPS

CLASS_NAMES = ["resource", "host_rock"]

CLASS_COLORS = {
    "resource": (0,255,0),
    "host_rock": (255,128,0),
}
DEFAULT_COLOR = (0,0,255)

# ── LOAD ONNX MODEL ──────────────────────────

print("[INFO] Loading ONNX Runtime model...")
session = ort.InferenceSession(
    MODEL_PATH,
    providers=["CPUExecutionProvider"]
)
input_name = session.get_inputs()[0].name

# ── THREAD QUEUES ────────────────────────────

frame_q = Queue(maxsize=2)
result_q = Queue(maxsize=2)

# ── CAMERA THREAD ────────────────────────────

def capture_loop(cap):
    while True:
        ret, frame = cap.read()
        if ret and not frame_q.full():
            frame_q.put(frame)

# ── YOLOv8 ONNX DECODE ───────────────────────

def decode_onnx(output, frame_shape):
    boxes = []
    confidences = []
    class_ids = []

    h, w = frame_shape[:2]

    output = np.squeeze(output)

    for det in output:
        scores = det[4:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]

        if confidence > CONF_THRESHOLD:
            cx, cy, bw, bh = det[:4]

            x = int((cx - bw / 2) * w)
            y = int((cy - bh / 2) * h)
            bw = int(bw * w)
            bh = int(bh * h)

            boxes.append([x, y, bw, bh])
            confidences.append(float(confidence))
            class_ids.append(class_id)

    indices = cv2.dnn.NMSBoxes(boxes, confidences, CONF_THRESHOLD, NMS_THRESHOLD)

    results = []
    if len(indices) > 0:
        for i in indices.flatten():
            results.append((boxes[i], confidences[i], class_ids[i]))

    return results

# ── INFERENCE THREAD ─────────────────────────

def infer_loop():
    while True:
        if not frame_q.empty():
            frame = frame_q.get()

            img = cv2.resize(frame, (IMGSZ, IMGSZ))
            img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR → RGB
            img = np.ascontiguousarray(img, dtype=np.float32) / 255.0
            img = np.expand_dims(img, axis=0)

            outputs = session.run(None, {input_name: img})

            detections = decode_onnx(outputs[0], frame.shape)

            if not result_q.full():
                result_q.put((frame, detections))

# ── DRAW DETECTIONS ──────────────────────────

def draw(frame, detections):
    counts = {"resource":0, "host_rock":0}

    for (box, conf, cls_id) in detections:
        x, y, w, h = box

        cls_name = CLASS_NAMES[cls_id] if cls_id < len(CLASS_NAMES) else "unknown"

        if cls_name in counts:
            counts[cls_name] += 1

        color = CLASS_COLORS.get(cls_name, DEFAULT_COLOR)

        label = f"{cls_name} {conf:.2f}"

        cv2.rectangle(frame, (x,y), (x+w,y+h), color, 1)

        cv2.putText(frame, label, (x, y-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    color, 1, cv2.LINE_AA)

    return frame, counts

# ── HUD ──────────────────────────────────────

def draw_hud(frame, counts, fps):
    lines = [
        f"Resource: {counts['resource']}",
        f"Host Rock: {counts['host_rock']}",
        f"FPS: {fps:.1f}"
    ]

    y = 18
    for line in lines:
        cv2.putText(frame, line, (8,y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0,255,255), 1, cv2.LINE_AA)
        y += 22

# ── MAIN ─────────────────────────────────────

def main():

    print("[INFO] Opening camera...")
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("[ERROR] Camera not found")
        return

    print("[INFO] Starting FFmpeg (HW encoder)...")

    ffmpeg_cmd = [
        "ffmpeg",
        "-loglevel","quiet",
        "-f","rawvideo",
        "-pix_fmt","bgr24",
        "-s",f"{FRAME_WIDTH}x{FRAME_HEIGHT}",
        "-r",str(FPS),
        "-i","-",
        "-c:v","h264_v4l2m2m",
        "-b:v","2M",
        "-f","rtsp",
        "rtsp://localhost:8554/mystream"
    ]

    ffmpeg = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)

    Thread(target=capture_loop, args=(cap,), daemon=True).start()
    Thread(target=infer_loop, daemon=True).start()

    prev = time.time()

    while True:

        if not result_q.empty():
            frame, detections = result_q.get()

            frame, counts = draw(frame, detections)

            now = time.time()
            fps = 1 / max(now - prev, 1e-6)
            prev = now

            draw_hud(frame, counts, fps)

            try:
                ffmpeg.stdin.write(frame.tobytes())
            except:
                break

# ── RUN ──────────────────────────────────────

if __name__ == "__main__":
    main()
