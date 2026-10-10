from ultralytics import YOLO

model = YOLO("yolov8n.pt")

model.train(
    data="data.yaml",
    epochs=50,
    imgsz=416,
    batch=8,
    workers=2,
    device="cpu",
    cache=True
)